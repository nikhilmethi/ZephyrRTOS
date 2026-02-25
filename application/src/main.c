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
#define HEARTBEAT_TOGGLE_INTERVAL_MS 500
// #define NOMINAL_BATTERY_VOLT_MV 3000 
#define LED_BLINK_FREQ_HZ 2
#define FREQ_UP_INC_HZ 1
#define FREQ_DOWN_INC_HZ 1
#define ACTION_FREQ_MIN_HZ 1
#define ACTION_FREQ_MAX_HZ 5

// declare function prototypes
void heartbeat_timer_handler(struct k_timer *t);
K_TIMER_DEFINE(heartbeat_timer, heartbeat_timer_handler, NULL);

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

static atomic_t sleep_button_event = ATOMIC_INIT(0);
static atomic_t reset_button_event = ATOMIC_INIT(0);
static atomic_t freq_up_button_event = ATOMIC_INIT(0);
static atomic_t freq_down_button_event = ATOMIC_INIT(0);

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

static int32_t stored_action_remaining_ms = 0;

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

                k_timer_start(&heartbeat_timer,
                K_MSEC(HEARTBEAT_TOGGLE_INTERVAL_MS),
                K_MSEC(HEARTBEAT_TOGGLE_INTERVAL_MS));

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

                LOG_INF("DEFAULTS: action_freq=%d Hz, phase=%d @ %llu ns",
                    action_freq_hz, action_phase, now_ns());

                state = AWAKE;
                break;
           
            case AWAKE: {
                // Frequency button handling (AWAKE only)
                bool freq_changed = false;

                if (atomic_cas(&freq_up_button_event, 1, 0)) {
                    action_freq_hz += FREQ_UP_INC_HZ;
                    freq_changed = true;
                    LOG_INF("FREQ_UP -> %d Hz @ %llu ns", action_freq_hz, now_ns());
                }

                if (atomic_cas(&freq_down_button_event, 1, 0)) {
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
                }
                break;
            }
            
            case SLEEP:
                // No per-loop actions: sleep/wake handled by global button event logic after switch
                break;

            case ERROR:
                /* RUN is intentionally empty; system waits for reset */
                break;
        }

        // Global sleep handling
        if (atomic_cas(&sleep_button_event, 1, 0)) {
            if (state == AWAKE) {
                // Enter sleep
                stored_action_freq_hz = action_freq_hz;
                stored_action_phase = action_phase;

                // Save remaining time in the current half-period BEFORE stopping
                stored_action_remaining_ms = k_timer_remaining_get(&action_timer);

                k_timer_stop(&action_timer);
                gpio_pin_set_dt(&iv_pump_led, 0);
                gpio_pin_set_dt(&buzzer_led, 0);

                LOG_INF("Entered SLEEP (stored freq=%d, phase=%d, rem=%d ms) @ %llu ns",
                    stored_action_freq_hz, stored_action_phase, stored_action_remaining_ms, now_ns());

                state = SLEEP;
            }
            else if (state == SLEEP) {
                // Exit sleep
                action_freq_hz = stored_action_freq_hz;
                action_phase = stored_action_phase;

                // Restore immediate LED state first
                if (action_phase == 0) {
                    gpio_pin_set_dt(&iv_pump_led, 1);
                    gpio_pin_set_dt(&buzzer_led, 0);
                } else {
                    gpio_pin_set_dt(&iv_pump_led, 0);
                    gpio_pin_set_dt(&buzzer_led, 1);
                }

                // Then start the timer
                int32_t hp = action_half_period_ms(action_freq_hz);

                // First interval should be whatever time was left when we went to sleep
                int32_t first_ms = stored_action_remaining_ms;

                // Safety fallback: if it's <=0 or larger than half-period, use half-period
                if (first_ms <= 0 || first_ms > hp) {
                    first_ms = hp;
                }

                k_timer_start(&action_timer, K_MSEC(first_ms), K_MSEC(hp));

                LOG_INF("Exited SLEEP (freq=%d, phase=%d, first=%d ms, hp=%d ms) @ %llu ns",
                    action_freq_hz, action_phase, first_ms, hp, now_ns());

                state = AWAKE;
            }
        }

        // Global reset handling
        if (atomic_cas(&reset_button_event, 1, 0)) {

            // clear error LED
            gpio_pin_set_dt(&error_led, 0);

            // re-enable interrupts
            gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE);
            gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE);
            gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_EDGE_TO_ACTIVE);

            atomic_set(&sleep_button_event, 0);
            atomic_set(&freq_up_button_event, 0);
            atomic_set(&freq_down_button_event, 0);
            
            k_timer_stop(&action_timer);
            LOG_INF("Reset pressed -> DEFAULTS @ %llu ns", now_ns());
            state = DEFAULTS;
        }

        k_msleep(10);  // include a very short sleep statement to allow any LOG messages to be printed
    }
}

// define timer functions
void heartbeat_timer_handler(struct k_timer *t)
{
    gpio_pin_toggle_dt(&heartbeat_led);
    LOG_INF("Heartbeat toggle @ %llu ns", now_ns());
}

void action_timer_handler(struct k_timer *t)
{
    action_phase = !action_phase;

    if (action_phase == 0) {
        gpio_pin_set_dt(&iv_pump_led, 1);
        gpio_pin_set_dt(&buzzer_led, 0);
    } else {
        gpio_pin_set_dt(&iv_pump_led, 0);
        gpio_pin_set_dt(&buzzer_led, 1);
    }

    LOG_INF("ACTION toggle (freq=%d Hz, phase=%d) @ %llu ns",
            action_freq_hz, action_phase, now_ns());
}

void action_timer_stop(struct k_timer *t)
{
    gpio_pin_set_dt(&iv_pump_led, 0);
    gpio_pin_set_dt(&buzzer_led, 0);
}

// define callback functions
void sleep_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    atomic_set(&sleep_button_event, 1);  
}

void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    atomic_set(&reset_button_event, 1); 
}

void freq_up_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    atomic_set(&freq_up_button_event, 1); 
}

void freq_down_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    atomic_set(&freq_down_button_event, 1); 
} 