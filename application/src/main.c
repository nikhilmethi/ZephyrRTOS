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

// define globals and DT-based hardware structs
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

// Timing / LED state setup
typedef struct {
    int64_t next_toggle_ms;   // next scheduled toggle time (k_uptime_get() domain)
} blink_timer_t;

// Heartbeat (independent of state)
static blink_timer_t heartbeat = { .next_toggle_ms = 0 };

// Action LEDs (buzzer + ivpump) are a coordinated out-of-phase pair
static blink_timer_t action = { .next_toggle_ms = 0 };

// action_phase == 0: ivpump ON, buzzer OFF
// action_phase == 1: ivpump OFF, buzzer ON
static bool action_phase = 0;

// Frequency control for action LEDs
static int action_freq_hz = LED_BLINK_FREQ_HZ;          // current
static int stored_action_freq_hz = LED_BLINK_FREQ_HZ;   // for sleep restore
static bool stored_action_phase = 0;                    // for sleep restore

// ERROR state entry latch
static bool error_entered = false;

int main(void)
{
    while (1) {
        // run the state machine in this indefinite loop

        int64_t current_time = k_uptime_get();  // get the current time in milliseconds

        // Heartbeat (independent of state)
        if (current_time - heartbeat.next_toggle_ms > HEARTBEAT_TOGGLE_INTERVAL_MS) {
            gpio_pin_toggle_dt(&heartbeat_led);
            heartbeat.next_toggle_ms = current_time;
            LOG_INF("Heartbeat toggle");
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
                gpio_add_callback_dt(&sleep_button, &sleep_button_cb);

                gpio_init_callback(&reset_button_cb, reset_button_callback, BIT(reset_button.pin)); // associate callback with GPIO pin
                gpio_add_callback_dt(&reset_button, &reset_button_cb);

                gpio_init_callback(&freq_up_button_cb, freq_up_button_callback, BIT(freq_up_button.pin));
                gpio_add_callback_dt(&freq_up_button, &freq_up_button_cb);

                gpio_init_callback(&freq_down_button_cb, freq_down_button_callback, BIT(freq_down_button.pin));
                gpio_add_callback_dt(&freq_down_button, &freq_down_button_cb);


                state = DEFAULTS;  // transition to the next state
                break;

            case DEFAULTS:
                // restore default operating parameters
                action_freq_hz = LED_BLINK_FREQ_HZ;   // 2 Hz default
                action_phase = 0;                     // ivpump ON, buzzer OFF baseline

                stored_action_freq_hz = action_freq_hz;
                stored_action_phase = action_phase;

                // clear error state / indicators
                error_entered = false;
                gpio_pin_set_dt(&error_led, 0);

                // set immediate out-of-phase outputs (baseline)
                gpio_pin_set_dt(&iv_pump_led, 1);
                gpio_pin_set_dt(&buzzer_led, 0);

                // initialize timing for future action blinking
                action.next_toggle_ms = current_time;

                LOG_INF("DEFAULTS: action_freq=%d Hz, phase=%d", action_freq_hz, action_phase);

                state = AWAKE;
                break;


            case AWAKE: {

                // Frequency button handling (AWAKE only)
                if (atomic_cas(&freq_up_button_event, 1, 0)) {
                    action_freq_hz += FREQ_UP_INC_HZ;
                    action.next_toggle_ms = current_time;   // apply new rate immediately
                    LOG_INF("FREQ_UP -> %d Hz", action_freq_hz);
                }

                if (atomic_cas(&freq_down_button_event, 1, 0)) {
                    action_freq_hz -= FREQ_DOWN_INC_HZ;
                    action.next_toggle_ms = current_time;   // apply new rate immediately
                    LOG_INF("FREQ_DOWN -> %d Hz", action_freq_hz);
                }

                // Bounds check: enter ERROR if out of range
                if (action_freq_hz < ACTION_FREQ_MIN_HZ || action_freq_hz > ACTION_FREQ_MAX_HZ) {
                    state = ERROR;
                    break;
                }

                // Action LEDs (out-of-phase)
                int32_t half_period_ms = 1000 / (2 * action_freq_hz);

                if (current_time >= action.next_toggle_ms) {
                    action_phase = !action_phase;

                    if (action_phase == 0) {
                        gpio_pin_set_dt(&iv_pump_led, 1);
                        gpio_pin_set_dt(&buzzer_led, 0);
                    } else {
                        gpio_pin_set_dt(&iv_pump_led, 0);
                        gpio_pin_set_dt(&buzzer_led, 1);
                    }

                    action.next_toggle_ms = current_time + (int64_t) half_period_ms;

                    LOG_INF("ACTION toggle (freq=%d Hz, phase=%d)",
                            action_freq_hz, action_phase);
                }
                break;
            }
            case SLEEP:
                // Handled externally
                break;

            case ERROR:
                if (!error_entered) {
                    // Stop action LEDs
                    gpio_pin_set_dt(&iv_pump_led, 0);
                    gpio_pin_set_dt(&buzzer_led, 0);

                    // Turn error LED on
                    gpio_pin_set_dt(&error_led, 1);

                    // Disable sleep/freq interrupts (reset remains enabled)
                    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_DISABLE);
                    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
                    gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_DISABLE);

                    LOG_ERR("ERROR: action_freq out of range (%d Hz)", action_freq_hz);
                    error_entered = true;
                }
                break;
        }

        // Global sleep handling
        if (atomic_cas(&sleep_button_event, 1, 0)) {
            if (state == AWAKE) {
                // Enter sleep
                stored_action_freq_hz = action_freq_hz;
                stored_action_phase = action_phase;

                gpio_pin_set_dt(&iv_pump_led, 0);
                gpio_pin_set_dt(&buzzer_led, 0);

                LOG_INF("Entered SLEEP (stored freq=%d, phase=%d)",
                        stored_action_freq_hz, stored_action_phase);

                state = SLEEP;
            }
            else if (state == SLEEP) {
                // Exit sleep
                action_freq_hz = stored_action_freq_hz;
                action_phase = stored_action_phase;

                // Restore immediate LED state
                if (action_phase == 0) {
                    gpio_pin_set_dt(&iv_pump_led, 1);
                    gpio_pin_set_dt(&buzzer_led, 0);
                } else {
                    gpio_pin_set_dt(&iv_pump_led, 0);
                    gpio_pin_set_dt(&buzzer_led, 1);
                }

                action.next_toggle_ms = current_time;

                LOG_INF("Exited SLEEP (freq=%d, phase=%d)",
                        action_freq_hz, action_phase);

                state = AWAKE;
            }
        }

        // Global reset handling
        if (atomic_cas(&reset_button_event, 1, 0)) {
            // clear ERROR latch so ERROR actions can run again next time
            error_entered = false;

            // clear error LED
            gpio_pin_set_dt(&error_led, 0);

            // re-enable interrupts
            gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE);
            gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE);
            gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_EDGE_TO_ACTIVE);

            atomic_set(&sleep_button_event, 0);
            atomic_set(&freq_up_button_event, 0);
            atomic_set(&freq_down_button_event, 0);

            LOG_INF("Reset pressed -> DEFAULTS");
            state = DEFAULTS;
        }

        k_msleep(10);  // include a very short sleep statement to allow any LOG messages to be printed
    }
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