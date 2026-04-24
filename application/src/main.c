#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/smf.h> 
#include <zephyr/drivers/adc.h> 
#include "calc_freq.h"
#include <zephyr/drivers/pwm.h> // CONFIG_PWM=y
#include "ble-lib.h" // BME554 BLE library (remember to add to CMakeLists.txt)
#include <string.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#define ADC_DT_SPEC_GET_BY_ALIAS(adc_alias)                    \
{                                                             \
    .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(adc_alias))),      \
    .channel_id = DT_REG_ADDR(DT_ALIAS(adc_alias)),            \
    ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(adc_alias))          \
}

/* macros */
#define HEARTBEAT_ON_MS 250
#define HEARTBEAT_OFF_MS 750

#define HEARTBEAT_STACK_SIZE 1024
#define HEARTBEAT_THREAD_PRIO 5

#define ADC_MAX_MV 3000

#define SIN_SAMPLE_INTERVAL_US 2500      // 400 Hz
#define SIN_DURATION_MS        2000   // 2 seconds
#define SIN_NUM_SAMPLES        (SIN_DURATION_MS * 1000 / SIN_SAMPLE_INTERVAL_US)

#define LED_PWM_PERIOD_NS PWM_USEC(1000)

extern void heartbeat_thread(void *, void *, void *);

/* heartbeat thread */
K_THREAD_DEFINE(heartbeat_thread_id,
                HEARTBEAT_STACK_SIZE,
                heartbeat_thread,
                NULL, NULL, NULL,
                HEARTBEAT_THREAD_PRIO, 0, 0);

/* button events (main-facing) */
/* application events */
#define APP_EVENT_BUTTON0_IDLE      BIT(0)
#define APP_EVENT_BUTTON1_TEMP      BIT(1)
#define APP_EVENT_BUTTON2_ECG       BIT(2)
#define APP_EVENT_BUTTON3_RESET     BIT(3)
#define APP_EVENT_BATTERY_SAMPLE    BIT(4)
#define APP_EVENT_ECG_SAMPLE        BIT(5)
#define APP_EVENT_ERROR             BIT(6)
#define APP_EVENT_ERROR_TIMEOUT     BIT(7)

#define APP_BUTTON_EVENT_MASK       (APP_EVENT_BUTTON0_IDLE | \
                                     APP_EVENT_BUTTON1_TEMP | \
                                     APP_EVENT_BUTTON2_ECG | \
                                     APP_EVENT_BUTTON3_RESET)

#define APP_MEASUREMENT_EVENT_MASK  (APP_EVENT_BATTERY_SAMPLE | \
                                     APP_EVENT_ECG_SAMPLE)

#define APP_SYSTEM_EVENT_MASK       (APP_EVENT_ERROR | \
                                     APP_EVENT_ERROR_TIMEOUT)

#define APP_EVENT_MASK              (APP_BUTTON_EVENT_MASK | \
                                     APP_MEASUREMENT_EVENT_MASK | \
                                     APP_SYSTEM_EVENT_MASK)

K_EVENT_DEFINE(app_events);

/* error code bits */
#define APP_ERROR_NONE              0U
#define APP_ERROR_GPIO_INIT         BIT(0)
#define APP_ERROR_ADC_INIT          BIT(1)
#define APP_ERROR_ADC_READ          BIT(2)
#define APP_ERROR_ADC_CONVERT       BIT(3)
#define APP_ERROR_PWM_INIT          BIT(4)
#define APP_ERROR_PWM_SET           BIT(5)
#define APP_ERROR_I2C_INIT          BIT(6)
#define APP_ERROR_I2C_READ          BIT(7)
#define APP_ERROR_BLE_INIT          BIT(8)
#define APP_ERROR_BLE_NOTIFY        BIT(9)
#define APP_ERROR_LOW_BATTERY       BIT(10)
#define APP_ERROR_UNKNOWN           BIT(31)

/* helper timing functions */

static inline uint64_t now_ns(void)
{
    int64_t ticks = k_uptime_ticks();
    return k_ticks_to_ns_near64(ticks);
}

static int clamp_mv(int32_t mv)
{
    if (mv < 0) return 0;
    if (mv > ADC_MAX_MV) return ADC_MAX_MV;
    return mv;
}

static uint8_t map_mv_to_duty(int32_t mv)
{
    int32_t mv_clamped = clamp_mv(mv);
    return (uint8_t)((mv_clamped * 100) / ADC_MAX_MV);
}

#define SIN_INPUT_MIN_MV 650
#define SIN_INPUT_MAX_MV 2650

static uint8_t map_diff_mv_to_duty(int32_t mv)
{
    if (mv < SIN_INPUT_MIN_MV) {
        mv = SIN_INPUT_MIN_MV;
    }
    if (mv > SIN_INPUT_MAX_MV) {
        mv = SIN_INPUT_MAX_MV;
    }

    return (uint8_t)(((mv - SIN_INPUT_MIN_MV) * 100) /
                     (SIN_INPUT_MAX_MV - SIN_INPUT_MIN_MV));
}

/* hardware definitions */

static const struct gpio_dt_spec heartbeat_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);

static const struct gpio_dt_spec iv_pump_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(ivpump), gpios);

static const struct gpio_dt_spec error_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(error), gpios);

static const struct gpio_dt_spec sleep_button =
    GPIO_DT_SPEC_GET(DT_ALIAS(sleepbutton), gpios);

static const struct gpio_dt_spec reset_button =
    GPIO_DT_SPEC_GET(DT_ALIAS(resetbutton), gpios);

static const struct gpio_dt_spec freq_up_button =
    GPIO_DT_SPEC_GET(DT_ALIAS(frequpbutton), gpios);

static const struct adc_dt_spec adc_single = 
    ADC_DT_SPEC_GET_BY_ALIAS(vadc_single);

static const struct adc_dt_spec adc_diff =
    ADC_DT_SPEC_GET_BY_ALIAS(vadc_diff);

/* PWM definitions */

static const struct pwm_dt_spec led2_pwm =
    PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));

static const struct pwm_dt_spec led3_pwm =
    PWM_DT_SPEC_GET(DT_ALIAS(pwm_led3));

/* callback prototypes */
void sleep_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void freq_up_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/* callback structs */
static struct gpio_callback sleep_button_cb;
static struct gpio_callback reset_button_cb;
static struct gpio_callback freq_up_button_cb;

/* SMF state machine */
enum app_state {
    STATE_INIT,
    STATE_IDLE,
    STATE_BATTERY,
    STATE_TEMPERATURE,
    STATE_ECG,
    STATE_ERROR,
};

struct app_object {
    struct smf_ctx ctx;

    uint64_t hb_last_ns;

    int16_t adc_raw;
    int32_t adc_mv;

    uint8_t battery_percent;
    int32_t battery_mv;

    int32_t temperature_centi_c;

    bool ecg_active;
    int32_t ecg_mv;
    uint16_t heart_rate_bpm;
    uint16_t heart_rate_avg_bpm;
    uint32_t ecg_sample_count;

    uint32_t error_code;
    bool terminate_requested;
};

static struct app_object s_obj;
static const struct smf_state app_states[];

/* SMF helper prototypes */
static int init_app_object(struct app_object *s);
static void set_led1(bool on);
static void post_error(struct app_object *s, uint32_t error_bit);
static void clear_error(struct app_object *s);

static void enable_button_interrupts(void);
static void disable_measurement_button_interrupts(void);

static int setup_adc_single(void);
static int setup_adc_diff(void);
static int set_led2_duty_cycle(uint8_t duty_percent);
static int set_led3_duty_cycle(uint8_t duty_percent);

static int do_single_sample(struct app_object *s);
static int do_diff_single_sample(int32_t *mv_out);

K_TIMER_DEFINE(sin_sample_timer, sin_sample_timer_handler, sin_sample_timer_stop_fn);

/* SMF state handlers */
static enum smf_state_result init_run(void *o);

static void idle_entry(void *o);
static enum smf_state_result idle_run(void *o);

static void battery_entry(void *o);
static enum smf_state_result battery_run(void *o);

static void temperature_entry(void *o);
static enum smf_state_result temperature_run(void *o);

static void ecg_entry(void *o);
static enum smf_state_result ecg_run(void *o);
static void ecg_exit(void *o);

static void error_entry(void *o);
static enum smf_state_result error_run(void *o);
static void error_exit(void *o);

static int setup_adc_single(void)
{
    int err;

    if (!device_is_ready(adc_single.dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }

    err = adc_channel_setup_dt(&adc_single);
    if (err < 0) {
        LOG_ERR("ADC channel setup failed (%d)", err);
        return err;
    }

    return 0;
}

static int setup_adc_diff(void)
{
    int err;

    if (!device_is_ready(adc_diff.dev)) {
        LOG_ERR("Differential ADC device not ready");
        return -ENODEV;
    }

    err = adc_channel_setup_dt(&adc_diff);
    if (err < 0) {
        LOG_ERR("Differential ADC channel setup failed (%d)", err);
        return err;
    }

    return 0;
}

static void enable_button_interrupts(void)
{
    (void)gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE);
    (void)gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE);
    (void)gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE);
}

static void disable_measurement_button_interrupts(void)
{
    /*
     * Keep BUTTON3/reset enabled. Later, when BUTTON0 is added explicitly,
     * it will also remain enabled during measurement states.
     */
    (void)gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_DISABLE);
    (void)gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
    (void)gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE);
}

static void post_error(struct app_object *s, uint32_t error_bit)
{
    if (s != NULL) {
        s->error_code |= error_bit;
    }

    k_event_post(&app_events, APP_EVENT_ERROR);
}

static void clear_error(struct app_object *s)
{
    if (s != NULL) {
        s->error_code = APP_ERROR_NONE;
    }
}

static void set_led1(bool on)
{
    gpio_pin_set_dt(&iv_pump_led, on ? 1 : 0);
}

static int init_app_object(struct app_object *s)
{
    if (s == NULL) {
        return -EINVAL;
    }

    memset(s, 0, sizeof(*s));

    s->battery_percent = 0U;
    s->battery_mv = 0;
    s->temperature_centi_c = 0;

    s->ecg_active = false;
    s->ecg_mv = 0;
    s->heart_rate_bpm = 0U;
    s->heart_rate_avg_bpm = 0U;
    s->ecg_sample_count = 0U;

    s->error_code = APP_ERROR_NONE;
    s->terminate_requested = false;

    return 0;
}

static enum smf_state_result init_run(void *o)
{
    struct app_object *s = (struct app_object *)o;
    int err;

    if (!device_is_ready(sleep_button.port)) {
        LOG_ERR("gpio0 interface not ready.");
        smf_set_terminate(SMF_CTX(s), -1);
        return SMF_EVENT_HANDLED;
    }

    err = gpio_pin_configure_dt(&sleep_button, GPIO_INPUT);
    if (err < 0) { LOG_ERR("Cannot configure sleep button."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_configure_dt(&reset_button, GPIO_INPUT);
    if (err < 0) { LOG_ERR("Cannot configure reset button."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_configure_dt(&freq_up_button, GPIO_INPUT);
    if (err < 0) { LOG_ERR("Cannot configure freq_up button."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_INACTIVE);
    if (err < 0) { LOG_ERR("Cannot configure heartbeat LED."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_configure_dt(&iv_pump_led, GPIO_OUTPUT_INACTIVE);
    if (err < 0) { LOG_ERR("Cannot configure LED1."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_configure_dt(&error_led, GPIO_OUTPUT_INACTIVE);
    if (err < 0) { LOG_ERR("Cannot configure error LED."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) { LOG_ERR("Cannot attach callback to sw0."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) { LOG_ERR("Cannot attach callback to sw3."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) { LOG_ERR("Cannot attach callback to sw1."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    gpio_init_callback(&sleep_button_cb, sleep_button_callback, BIT(sleep_button.pin));
    err = gpio_add_callback_dt(&sleep_button, &sleep_button_cb);
    if (err < 0) { LOG_ERR("Cannot add sleep button callback."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    gpio_init_callback(&reset_button_cb, reset_button_callback, BIT(reset_button.pin));
    err = gpio_add_callback_dt(&reset_button, &reset_button_cb);
    if (err < 0) { LOG_ERR("Cannot add reset button callback."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    gpio_init_callback(&freq_up_button_cb, freq_up_button_callback, BIT(freq_up_button.pin));
    err = gpio_add_callback_dt(&freq_up_button, &freq_up_button_cb);
    if (err < 0) { LOG_ERR("Cannot add freq_up button callback."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    k_event_clear(&app_events, APP_EVENT_MASK);
    clear_error(s);

    k_thread_name_set(heartbeat_thread_id, "heartbeat");

    err = setup_adc_single();
    if (err < 0) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return SMF_EVENT_HANDLED;
    }

    err = setup_adc_diff();
    if (err < 0) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return SMF_EVENT_HANDLED;
    }

    if (!device_is_ready(led2_pwm.dev)) {
        LOG_ERR("LED2 PWM device not ready");
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return SMF_EVENT_HANDLED;
    }

    err = set_led2_duty_cycle(0);
    if (err < 0) {
        LOG_ERR("Failed to initialize LED2 PWM (%d)", err);
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return SMF_EVENT_HANDLED;
    }

    if (!device_is_ready(led3_pwm.dev)) {
        LOG_ERR("LED3 PWM device not ready");
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return SMF_EVENT_HANDLED;
    }

    err = set_led3_duty_cycle(0);
    if (err < 0) {
        LOG_ERR("Failed to initialize LED3 PWM (%d)", err);
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return SMF_EVENT_HANDLED;
    }

    smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
    return SMF_EVENT_HANDLED;
}

 static void idle_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;

    gpio_pin_set_dt(&error_led, 0);
    set_led1(false);

    s->ecg_active = false;

    enable_button_interrupts();

    LOG_INF("IDLE: waiting for BUTTON1 temperature, BUTTON2 ECG, or BUTTON3 reset");
}

static enum smf_state_result idle_run(void *o)
{
    struct app_object *s = (struct app_object *)o;

    uint32_t events = k_event_wait(&app_events,
                                   APP_BUTTON_EVENT_MASK | APP_SYSTEM_EVENT_MASK,
                                   true,
                                   K_FOREVER);

    LOG_INF("IDLE event mask: 0x%08x", events);

    if (events & APP_EVENT_ERROR) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return SMF_EVENT_HANDLED;
    }

    if (events & APP_EVENT_BUTTON3_RESET) {
        clear_error(s);
        s->ecg_active = false;

        (void)set_led2_duty_cycle(0);
        (void)set_led3_duty_cycle(0);

        LOG_INF("BUTTON3 reset from IDLE: stored measurements preserved");
        smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
        return SMF_EVENT_HANDLED;
    }

    if (events & APP_EVENT_BUTTON1_TEMP) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_TEMPERATURE]);
        return SMF_EVENT_HANDLED;
    }

    if (events & APP_EVENT_BUTTON2_ECG) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_ECG]);
        return SMF_EVENT_HANDLED;
    }

    if (events & APP_EVENT_BATTERY_SAMPLE) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_BATTERY]);
        return SMF_EVENT_HANDLED;
    }

    return SMF_EVENT_HANDLED;
}

static void battery_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;

    disable_measurement_button_interrupts();

    /*
     * Placeholder for Commit 4.
     * This state will measure AIN0, update battery_percent,
     * update LED1 brightness, and check the low-battery threshold.
     */
    LOG_INF("BATTERY state placeholder");

    smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
}

static enum smf_state_result battery_run(void *o)
{
    ARG_UNUSED(o);
    return SMF_EVENT_HANDLED;
}

static void temperature_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;

    disable_measurement_button_interrupts();

    /*
     * Placeholder for Commit 5.
     * This state will read MCP9808 temperature over I2C and notify over BLE.
     */
    LOG_INF("TEMPERATURE state placeholder");

    smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
}

static enum smf_state_result temperature_run(void *o)
{
    ARG_UNUSED(o);
    return SMF_EVENT_HANDLED;
}

static void ecg_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;

    disable_measurement_button_interrupts();

    s->ecg_active = true;
    s->ecg_sample_count = 0U;

    /*
     * Placeholder for Commit 6.
     * This state will start ECG sampling and calculate average heart rate.
     */
    LOG_INF("ECG state placeholder: live ECG measurements active");

    smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
}

static enum smf_state_result ecg_run(void *o)
{
    ARG_UNUSED(o);
    return SMF_EVENT_HANDLED;
}

static void ecg_exit(void *o)
{
    struct app_object *s = (struct app_object *)o;

    s->ecg_active = false;
    LOG_INF("Exited ECG state");
}

static void error_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;

    s->ecg_active = false;

    set_led1(false);
    (void)set_led2_duty_cycle(0);
    (void)set_led3_duty_cycle(0);

    disable_measurement_button_interrupts();
    gpio_pin_set_dt(&error_led, 1);

    LOG_ERR("Entered ERROR state with error_code=0x%08x", s->error_code);

    /*
     * BLE error notification will be added in the BLE commit.
     * Full 4-LED in-phase 50%% error blinking will be added in the LED/error commit.
     */
}

static enum smf_state_result error_run(void *o)
{
    struct app_object *s = (struct app_object *)o;

    uint32_t events = k_event_wait(&app_events,
                                   APP_EVENT_BUTTON3_RESET | APP_EVENT_ERROR_TIMEOUT,
                                   true,
                                   K_FOREVER);

    LOG_INF("ERROR event mask: 0x%08x", events);

    if (events & APP_EVENT_ERROR_TIMEOUT) {
        s->terminate_requested = true;
        LOG_ERR("ERROR timeout reached; requesting graceful termination");
        smf_set_terminate(SMF_CTX(s), -ETIMEDOUT);
        return SMF_EVENT_HANDLED;
    }

    if (events & APP_EVENT_BUTTON3_RESET) {
        clear_error(s);
        s->terminate_requested = false;
        smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
        return SMF_EVENT_HANDLED;
    }

    return SMF_EVENT_HANDLED;
}

static void error_exit(void *o)
{
    ARG_UNUSED(o);

    gpio_pin_set_dt(&error_led, 0);
}

static const struct smf_state app_states[] = {
    [STATE_INIT]        = SMF_CREATE_STATE(NULL,              init_run,         NULL,      NULL, NULL),
    [STATE_IDLE]        = SMF_CREATE_STATE(idle_entry,        idle_run,         NULL,      NULL, NULL),
    [STATE_BATTERY]     = SMF_CREATE_STATE(battery_entry,     battery_run,      NULL,      NULL, NULL),
    [STATE_TEMPERATURE] = SMF_CREATE_STATE(temperature_entry, temperature_run,  NULL,      NULL, NULL),
    [STATE_ECG]         = SMF_CREATE_STATE(ecg_entry,         ecg_run,          ecg_exit,  NULL, NULL),
    [STATE_ERROR]       = SMF_CREATE_STATE(error_entry,       error_run,        error_exit,NULL, NULL),
};

int main(void)
{
    int ret = init_app_object(&s_obj);

    if (ret != 0) {
        LOG_ERR("could not initialize state object (%d)", ret);
        return ret;
    }

    smf_set_initial(SMF_CTX(&s_obj), &app_states[STATE_INIT]);

    while (1) {
        ret = smf_run_state(SMF_CTX(&s_obj));
        if (ret != 0) {
            LOG_ERR("terminating state machine (%d)", ret);
            break;
        }
    }

    return ret;
}

static int set_led2_duty_cycle(uint8_t duty_percent)
{
    if (!device_is_ready(led2_pwm.dev)) {
        LOG_ERR("PWM device not ready");
        return -ENODEV;
    }

    if (duty_percent > 100) {
        duty_percent = 100;
    }

    uint32_t pulse = (LED_PWM_PERIOD_NS * duty_percent) / 100;
    return pwm_set_dt(&led2_pwm, LED_PWM_PERIOD_NS, pulse);
}

static int set_led3_duty_cycle(uint8_t duty_percent)
{
    if (!device_is_ready(led3_pwm.dev)) {
        LOG_ERR("PWM device not ready");
        return -ENODEV;
    }

    if (duty_percent > 100) {
        duty_percent = 100;
    }

    uint32_t pulse = (LED_PWM_PERIOD_NS * duty_percent) / 100;
    return pwm_set_dt(&led3_pwm, LED_PWM_PERIOD_NS, pulse); 
}

static int do_single_sample(struct app_object *s)
{
    int ret;
    int16_t buf = 0;
    int32_t val_mv = 0;

    struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };

    (void)adc_sequence_init_dt(&adc_single, &sequence);

    ret = adc_read(adc_single.dev, &sequence);
    if (ret < 0) {
        LOG_ERR("ADC read failed (%d)", ret);
        return ret;
    }

    s->adc_raw = buf;
    val_mv = buf;

    ret = adc_raw_to_millivolts_dt(&adc_single, &val_mv);
    if (ret < 0) {
        LOG_ERR("ADC conversion to mV failed");
        return ret;
    }

    s->adc_mv = val_mv;
    uint8_t duty = map_mv_to_duty(s->adc_mv);

    LOG_INF("ADC raw=%d, mv=%d, LED2 duty=%d%%",
            s->adc_raw,
            s->adc_mv,
            duty);

    return 0;
}

static int do_diff_single_sample(int32_t *mv_out)
{
    int16_t buf = 0;
    int32_t val_mv = 0;

    struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };

    (void)adc_sequence_init_dt(&adc_diff, &sequence);

    int ret = adc_read(adc_diff.dev, &sequence);
    if (ret < 0) {
        LOG_ERR("Diff ADC read failed (%d)", ret);
        return ret;
    }

    val_mv = buf;

    ret = adc_raw_to_millivolts_dt(&adc_diff, &val_mv);
    if (ret < 0) {
        LOG_ERR("Diff ADC conversion failed");
        return ret;
    }

    *mv_out = val_mv;
    return 0;
}

/* GPIO callbacks */

void sleep_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    k_event_post(&app_events, APP_EVENT_BUTTON1_TEMP);
}

void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    k_event_post(&app_events, APP_EVENT_BUTTON3_RESET);
}

void freq_up_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    k_event_post(&app_events, APP_EVENT_BUTTON2_ECG);
}

void sin_sample_timer_handler(struct k_timer *t)
{
    ARG_UNUSED(t);

    k_event_post(&app_events, APP_EVENT_ECG_SAMPLE);
}

void sin_sample_timer_stop_fn(struct k_timer *t)
{
    ARG_UNUSED(t);
}

/* threads */

void heartbeat_thread(void *, void *, void *)
{
    gpio_pin_set_dt(&heartbeat_led, 0);

    while (1) {
        uint64_t now;

        k_msleep(HEARTBEAT_OFF_MS);
        gpio_pin_toggle_dt(&heartbeat_led);

        now = now_ns();
        if (s_obj.hb_last_ns != 0U) {
            LOG_INF("heart toggle period (ns): %llu",
                    (unsigned long long)(now - s_obj.hb_last_ns));
        }
        s_obj.hb_last_ns = now;

        k_msleep(HEARTBEAT_ON_MS);
        gpio_pin_toggle_dt(&heartbeat_led);

        now = now_ns();
        if (s_obj.hb_last_ns != 0U) {
            LOG_INF("heart toggle period (ns): %llu",
                    (unsigned long long)(now - s_obj.hb_last_ns));
        }
        s_obj.hb_last_ns = now;
    }
}