#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h> 
#include <zephyr/logging/log.h>  // needs CONFIG_LOG=y in your prj.conf
// #include <zephyr/drivers/adc.h> // CONFIG_ADC=y
// #include <zephyr/drivers/pwm.h> // CONFIG_PWM=y
// #include <zephyr/smf.h> // CONFIG_SMF=y

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// define macros 
#define HEARTBEAT_TOGGLE_INTERVAL_MS 500

// declare function prototypes

// define globals and DT-based hardware structs
//static const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);
static const struct gpio_dt_spec sleep_button = GPIO_DT_SPEC_GET(DT_ALIAS(sleepbutton), gpios);
static const struct gpio_dt_spec reset_button = GPIO_DT_SPEC_GET(DT_ALIAS(resetbutton), gpios);
static const struct gpio_dt_spec heartbeat_led = GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);

bool sleep_button_event = 0;  // flag to indicate that the sleep button has been pressed
bool reset_button_event = 0;  // flag to indicate that the reset button has been pressed

// define callback functions
void sleep_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

// initialize GPIO Callback Structs
static struct gpio_callback sleep_button_cb;  // example; need one per callback (button)  
static struct gpio_callback reset_button_cb;  // example; need one per callback (button)  

// define states for state machine (THESE ARE ONLY PLACEHOLDERS)
enum states { INIT, AWAKE_ENTRY, AWAKE_RUN, AWAKE_EXIT, SLEEP };
int state = INIT; // initial state

struct led {
    int64_t toggle_time;  // int64_t b/c time functions in Zephyr use this type
    bool illuminated; // state of the LED (on/off)
};

struct led heartbeat_led_status = {
    .toggle_time = 0,
    .illuminated = false
};

// placeholder variables
int condition_to_leave_awake_state = 0;
int next_state = SLEEP;

int main(void)
{
    /* below are some functional examples to get you started, but this is 
       just a starting point, not a complete program! */


    while (1) {
        // run the state machine in this indefinite loop

        // some useful functions
        // gpio_pin_toggle_dt() - toggle the state of a pin (e.g. gpio_pin_toggle_dt(&led0))
    
        int64_t current_time = k_uptime_get();  // get the current time in milliseconds

        switch (state) {
            case INIT:
                // check if interface is ready
                if (!device_is_ready(sleep_button.port)) {
                    LOG_ERR("gpio0 interface not ready.");  // logging module output
                    return -1;  // exit code that will exit main()
                }

                // configure GPIO pin
                int err = gpio_pin_configure_dt(&sleep_button, GPIO_INPUT);
                if (err < 0) {
                    LOG_ERR("Cannot configure sw0 pin.");
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

                // associate callback with GPIO pin
                // trigger on transition from INACTIVE -> ACTIVE; ACTIVE could be HIGH or LOW
                err = gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE); 
                if (err < 0) {
                    LOG_ERR("Cannot attach callback to sw0.");
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

                state = AWAKE_ENTRY;  // transition to the next state
                break;

            case AWAKE_ENTRY:
                // do something on upon first entering the awake state
                state = AWAKE_RUN;  // transition to the next state
                break;

            case AWAKE_RUN:
                // do something

                if (current_time - heartbeat_led_status.toggle_time > HEARTBEAT_TOGGLE_INTERVAL_MS) {
                    gpio_pin_toggle_dt(&heartbeat_led);
                    // gpio_pin_set_dt() - explicitly set the state of a pin (e.g. gpio_pin_set_dt(&led0, 1))
                    heartbeat_led_status.toggle_time = current_time;
                }
                break;  // break out of the switch statement without evaluating other cases
            
                if (condition_to_leave_awake_state) {
                    state = AWAKE_EXIT;  // transition to the next state
                }
                break;
            
            case AWAKE_EXIT:
                // do something on exit from the awake state
                state = next_state;  // transition to the next state
                break;

            case SLEEP:
                // want to change what the buttons do in a different state?
                // gpio_remove_callback_dt(button_gpio_struct, &original_button_cb);
                // gpio_add_callback_dt(button_gpio_struct, &new_button_cb);
            
                // OR, want to disable the button entirely?
                // gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_DISABLE);
            default:
                // handle unexpected state
                break;
        }

        // test for the callback event state in your code
        if (sleep_button_event) {
            // do something based on the event
            state = SLEEP;
            sleep_button_event = 0;  // clear the event after taking action
        } 

    
        k_msleep(10);  // include a very short sleep statement to allow any LOG messages to be printed
    }
}

// define callback functions
void sleep_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    sleep_button_event = 1;  // conditional statement in main() can now do something based on the event detection
                             // we can also use actual system kernel event flags, but this is simpler (for now)
}

void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    reset_button_event = 1;  // conditional statement in main() can now do something based on the event detection
                             // we can also use actual system kernel event flags, but this is simpler (for now)
}
