#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h> 
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
// #include <zephyr/drivers/adc.h> // CONFIG_ADC=y
// #include <zephyr/drivers/pwm.h> // CONFIG_PWM=y
// #include <zephyr/smf.h> // CONFIG_SMF=y
// #include "ble-lib.h" // BME554 BLE library (remember to add to CMakeLists.txt)

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// define macros 
// #define NOMINAL_BATTERY_VOLT_MV 3000 
#define LED_BLINK_FREQ_HZ 2
#define FREQ_UP_INC_HZ 1
#define FREQ_DOWN_INC_HZ 1
#define ACTION_FREQ_MIN_HZ 1
#define ACTION_FREQ_MAX_HZ 5
#define HEARTBEAT_ON_MS   250
#define HEARTBEAT_OFF_MS  750

#define HEARTBEAT_STACK_SIZE 1024
#define HEARTBEAT_THREAD_PRIO 5

K_THREAD_STACK_DEFINE(heartbeat_stack, HEARTBEAT_STACK_SIZE);
static struct k_thread heartbeat_thread_data;

static void heartbeat_thread(void *a, void *b, void *c);

/* 4-bit button event array */
#define BTN_SLEEP_BIT     BIT(0)
#define BTN_RESET_BIT     BIT(1)
#define BTN_FREQ_UP_BIT   BIT(2)
#define BTN_FREQ_DOWN_BIT BIT(3)

#define BTN_ALL_BITS (BTN_SLEEP_BIT | BTN_RESET_BIT | BTN_FREQ_UP_BIT | BTN_FREQ_DOWN_BIT)

/* Kernel event object */
K_EVENT_DEFINE(button_events);

// declare function prototypes
void action_timer_handler(struct k_timer *t);
void action_timer_stop(struct k_timer *t);
K_TIMER_DEFINE(action_timer, action_timer_handler, action_timer_stop);

// define globals and DT-based hardware structs

static inline uint64_t now_ns(void)
{
    int64_t ticks = k_uptime_ticks();
    return k_ticks_to_ns_near64(ticks);
}

static inline int32_t action_half_period_ms(int freq_hz)
{
    return 1000 / (2 * freq_hz);
}

static const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);
static const struct gpio_dt_spec iv_pump_led = GPIO_DT_SPEC_GET(DT_ALIAS(ivpump), gpios);
static const struct gpio_dt_spec buzzer_led = GPIO_DT_SPEC_GET(DT_ALIAS(buzzer), gpios);
static const struct gpio_dt_spec error_led = GPIO_DT_SPEC_GET(DT_ALIAS(error), gpios);
static const struct gpio_dt_spec sleep_button = GPIO_DT_SPEC_GET(DT_ALIAS(sleepbutton), gpios);
static const struct gpio_dt_spec reset_button = GPIO_DT_SPEC_GET(DT_ALIAS(resetbutton), gpios);
static const struct gpio_dt_spec freq_up_button = GPIO_DT_SPEC_GET(DT_ALIAS(frequpbutton), gpios);
static const struct gpio_dt_spec freq_down_button = GPIO_DT_SPEC_GET(DT_ALIAS(freqdownbutton), gpios);

// define callback functions
void sleep_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void freq_up_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void freq_down_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

// initialize GPIO Callback Structs
static struct gpio_callback sleep_button_cb; 
static struct gpio_callback reset_button_cb;   
static struct gpio_callback freq_up_button_cb;   
static struct gpio_callback freq_down_button_cb; 

// define states for state machine
enum states { INIT, DEFAULTS, AWAKE, SLEEP, ERROR };
static int state = INIT;
static enum states prev_state = INIT;

// action_phase == 0: ivpump ON, buzzer OFF
// action_phase == 1: ivpump OFF, buzzer ON
static bool action_phase = 0;

// Frequency control for action LEDs
static int action_freq_hz = LED_BLINK_FREQ_HZ;          // current
static int stored_action_freq_hz = LED_BLINK_FREQ_HZ;   // for sleep restore
static bool stored_action_phase = 0;                    // for sleep restore

static uint64_t hb_last_ns = 0;
static int32_t stored_action_remaining_ms = 0;
static uint64_t action_last_ns = 0;

int main(void)
{
    while (1) {
        // run the state machine in this indefinite loop
        if (state != prev_state) {
            /* ENTRY actions */
            if (state == ERROR) {
                k_timer_stop(&action_timer);

                gpio_pin_set_dt(&iv_pump_led, 0);
                gpio_pin_set_dt(&buzzer_led, 0);
                gpio_pin_set_dt(&error_led, 1);

                gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_DISABLE);
                gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
                gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_DISABLE);

                LOG_ERR("ERROR: action_freq out of range (%d Hz) @ %llu ns",
                        action_freq_hz, now_ns());
            }

            /* EXIT actions (optional) */
            /* none needed to preserve behavior */

            prev_state = state;
        }
        switch (state) {
            case INIT:
                // check if interface is ready
                if (!device_is_ready(sleep_button.port)) {
                    LOG_ERR("gpio0 interface not ready.");  // logging module output
                    return -1;  // exit code that will exit main()
                }

                // configure GPIO pins
                int err = gpio_pin_configure_dt(&sleep_button, GPIO_INPUT);
                if (err < 0) {
                    LOG_ERR("Cannot configure sleep button.");
                    return err;
                }

                err = gpio_pin_configure_dt(&freq_up_button, GPIO_INPUT);
                if (err < 0) {
                    LOG_ERR("Cannot configure freq_up button.");
                    return err;
                }

                err = gpio_pin_configure_dt(&freq_down_button, GPIO_INPUT);
                if (err < 0) {
                    LOG_ERR("Cannot configure freq_down button.");
                    return err;
                }

                err = gpio_pin_configure_dt(&reset_button, GPIO_INPUT);
                if (err < 0) {
                    LOG_ERR("Cannot configure reset button.");
                    return err;
                }

                err = gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_ACTIVE);
                if (err < 0) {
                    LOG_ERR("Cannot configure heartbeat LED.");
                    return err;
                }

                err = gpio_pin_configure_dt(&iv_pump_led, GPIO_OUTPUT_INACTIVE);
                if (err < 0) {
                    LOG_ERR("Cannot configure iv_pump LED.");
                    return err;
                }

                err = gpio_pin_configure_dt(&buzzer_led, GPIO_OUTPUT_INACTIVE);
                if (err < 0) {
                    LOG_ERR("Cannot configure buzzer LED.");
                    return err;
                }

                err = gpio_pin_configure_dt(&error_led, GPIO_OUTPUT_INACTIVE);
                if (err < 0) {
                    LOG_ERR("Cannot configure error LED.");
                    return err;
                }

                // associate callback with GPIO pin
                // trigger on transition from INACTIVE -> ACTIVE; ACTIVE could be HIGH or LOW
                err = gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE); 
                if (err < 0) {
                    LOG_ERR("Cannot attach callback to sw0.");
                }
                err = gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE); 
                if (err < 0) {
                    LOG_ERR("Cannot attach callback to sw1.");
                }
                err = gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_EDGE_TO_ACTIVE); 
                if (err < 0) {
                    LOG_ERR("Cannot attach callback to sw2.");
                }
                err = gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE); 
                if (err < 0) {
                    LOG_ERR("Cannot attach callback to sw3.");
                }
                // populate CB struct with information about the CB function and pin
                gpio_init_callback(&sleep_button_cb, sleep_button_callback, BIT(sleep_button.pin)); // associate callback with GPIO pin
                err = gpio_add_callback_dt(&sleep_button, &sleep_button_cb);
                if (err < 0) { LOG_ERR("Cannot add sleep button callback."); return err; }

                gpio_init_callback(&reset_button_cb, reset_button_callback, BIT(reset_button.pin)); // associate callback with GPIO pin
                err = gpio_add_callback_dt(&reset_button, &reset_button_cb);
                if (err < 0) { LOG_ERR("Cannot add reset button callback."); return err; }

                gpio_init_callback(&freq_up_button_cb, freq_up_button_callback, BIT(freq_up_button.pin));
                err = gpio_add_callback_dt(&freq_up_button, &freq_up_button_cb);
                if (err < 0) { LOG_ERR("Cannot add freq_up button callback."); return err; }

                gpio_init_callback(&freq_down_button_cb, freq_down_button_callback, BIT(freq_down_button.pin));
                err = gpio_add_callback_dt(&freq_down_button, &freq_down_button_cb);
                if (err < 0) { LOG_ERR("Cannot add freq_down button callback."); return err; }

                k_thread_create(&heartbeat_thread_data,
                    heartbeat_stack,
                    K_THREAD_STACK_SIZEOF(heartbeat_stack),
                    heartbeat_thread,
                    NULL, NULL, NULL,
                    HEARTBEAT_THREAD_PRIO, 0, K_NO_WAIT);

                k_thread_name_set(&heartbeat_thread_data, "heartbeat");
                k_event_clear(&button_events, BTN_ALL_BITS);
                action_last_ns = 0;

                state = DEFAULTS;  // transition to the next state
                break;

            case DEFAULTS:
                // restore default operating parameters
                action_freq_hz = LED_BLINK_FREQ_HZ;   // 2 Hz default
                action_phase = 0;                     // ivpump ON, buzzer OFF baseline

                stored_action_freq_hz = action_freq_hz;
                stored_action_phase = action_phase;
                stored_action_remaining_ms = 0;

                gpio_pin_set_dt(&error_led, 0);

                // set immediate out-of-phase outputs (baseline)
                gpio_pin_set_dt(&iv_pump_led, 1);
                gpio_pin_set_dt(&buzzer_led, 0);

                // initialize timing for future action blinking
                k_timer_stop(&action_timer);
                int32_t hp = action_half_period_ms(action_freq_hz);
                k_timer_start(&action_timer, K_MSEC(hp), K_MSEC(hp));
                action_last_ns = 0;

                LOG_INF("DEFAULTS: action_freq=%d Hz, phase=%d @ %llu ns",
                    action_freq_hz, action_phase, now_ns());

                state = AWAKE;
                break;
           
            case AWAKE: {
                uint32_t ev = k_event_wait(&button_events, BTN_ALL_BITS, true, K_FOREVER);

                /* Reset wins */
                if (ev & BTN_RESET_BIT) {
                    gpio_pin_set_dt(&error_led, 0);

                    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE);
                    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE);
                    gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_EDGE_TO_ACTIVE);

                    k_timer_stop(&action_timer);
                    LOG_INF("Reset pressed -> DEFAULTS @ %llu ns", now_ns());
                    state = DEFAULTS;
                    break;
                }

                /* Sleep toggle */
                if (ev & BTN_SLEEP_BIT) {
                    stored_action_freq_hz = action_freq_hz;
                    stored_action_phase = action_phase;

                    stored_action_remaining_ms = k_timer_remaining_get(&action_timer);

                    k_timer_stop(&action_timer);
                    gpio_pin_set_dt(&iv_pump_led, 0);
                    gpio_pin_set_dt(&buzzer_led, 0);

                    LOG_INF("Entered SLEEP (stored freq=%d, phase=%d, rem=%d ms) @ %llu ns",
                        stored_action_freq_hz, stored_action_phase, stored_action_remaining_ms, now_ns());

                    state = SLEEP;
                    break;
                }

                /* Frequency changes */
                bool freq_changed = false;

                if (ev & BTN_FREQ_UP_BIT) {
                    action_freq_hz += FREQ_UP_INC_HZ;
                    freq_changed = true;
                    LOG_INF("FREQ_UP -> %d Hz @ %llu ns", action_freq_hz, now_ns());
                }

                if (ev & BTN_FREQ_DOWN_BIT) {
                    action_freq_hz -= FREQ_DOWN_INC_HZ;
                    freq_changed = true;
                    LOG_INF("FREQ_DOWN -> %d Hz @ %llu ns", action_freq_hz, now_ns());
                }

                if (action_freq_hz < ACTION_FREQ_MIN_HZ || action_freq_hz > ACTION_FREQ_MAX_HZ) {
                    state = ERROR;
                    break;
                }

                if (freq_changed) {
                    int32_t hp = action_half_period_ms(action_freq_hz);
                    k_timer_start(&action_timer, K_MSEC(hp), K_MSEC(hp));
                    action_last_ns = 0;
                }

                break;
            }
            case SLEEP: {
                uint32_t ev = k_event_wait(&button_events,
                                        BTN_SLEEP_BIT | BTN_RESET_BIT,
                                        true,
                                        K_FOREVER);

                if (ev & BTN_RESET_BIT) {
                    gpio_pin_set_dt(&error_led, 0);

                    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE);
                    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE);
                    gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_EDGE_TO_ACTIVE);

                    k_timer_stop(&action_timer);
                    LOG_INF("Reset pressed -> DEFAULTS @ %llu ns", now_ns());
                    state = DEFAULTS;
                    break;
                }

                /* Wake on Sleep */
                action_freq_hz = stored_action_freq_hz;
                action_phase = stored_action_phase;

                if (action_phase == 0) {
                    gpio_pin_set_dt(&iv_pump_led, 1);
                    gpio_pin_set_dt(&buzzer_led, 0);
                } else {
                    gpio_pin_set_dt(&iv_pump_led, 0);
                    gpio_pin_set_dt(&buzzer_led, 1);
                }

                int32_t hp = action_half_period_ms(action_freq_hz);
                int32_t first_ms = stored_action_remaining_ms;

                if (first_ms <= 0 || first_ms > hp) {
                    first_ms = hp;
                }

                k_timer_start(&action_timer, K_MSEC(first_ms), K_MSEC(hp));
                action_last_ns = 0;

                LOG_INF("Exited SLEEP (freq=%d, phase=%d, first=%d ms, hp=%d ms) @ %llu ns",
                    action_freq_hz, action_phase, first_ms, hp, now_ns());

                state = AWAKE;
                break;
            }
            case ERROR: {
                (void)k_event_wait(&button_events, BTN_RESET_BIT, true, K_FOREVER);

                gpio_pin_set_dt(&error_led, 0);

                gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE);
                gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE);
                gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_EDGE_TO_ACTIVE);

                k_timer_stop(&action_timer);
                LOG_INF("Reset pressed -> DEFAULTS @ %llu ns", now_ns());
                state = DEFAULTS;
                break;
            }
        }
    }
}

// define timer functions
void action_timer_handler(struct k_timer *t)
{
    uint64_t now = now_ns();
    if (action_last_ns != 0) {
        LOG_INF("action toggle period (ns): %llu", now - action_last_ns);
    }
    action_last_ns = now;

    action_phase = !action_phase;
    if (action_phase == 0) {
        gpio_pin_set_dt(&iv_pump_led, 1);
        gpio_pin_set_dt(&buzzer_led, 0);
    } else {
        gpio_pin_set_dt(&iv_pump_led, 0);
        gpio_pin_set_dt(&buzzer_led, 1);
    }
}

void action_timer_stop(struct k_timer *t)
{
    gpio_pin_set_dt(&iv_pump_led, 0);
    gpio_pin_set_dt(&buzzer_led, 0);
}

// define callback functions
void sleep_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&button_events, BTN_SLEEP_BIT);
}

void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&button_events, BTN_RESET_BIT);
}

void freq_up_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&button_events, BTN_FREQ_UP_BIT);
}

void freq_down_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&button_events, BTN_FREQ_DOWN_BIT);
}

// define thread functions
static void heartbeat_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    while (1) {
        uint64_t now = now_ns();
        if (hb_last_ns != 0) {
            LOG_INF("heart toggle period (ns): %llu", now - hb_last_ns);
        }
        hb_last_ns = now;

        gpio_pin_toggle_dt(&heartbeat_led);

        /* duty cycle: 25% on, 75% off */
        if (gpio_pin_get_dt(&heartbeat_led) > 0) {
            k_msleep(HEARTBEAT_ON_MS);
        } else {
            k_msleep(HEARTBEAT_OFF_MS);
        }
    }
}