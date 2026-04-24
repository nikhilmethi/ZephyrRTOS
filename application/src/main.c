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
#include <zephyr/drivers/i2c.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
#define ADC_DT_SPEC_GET_BY_ALIAS(adc_alias)                    \
{                                                             \
    .dev = DEVICE_DT_GET(DT_PARENT(DT_ALIAS(adc_alias))),      \
    .channel_id = DT_REG_ADDR(DT_ALIAS(adc_alias)),            \
    ADC_CHANNEL_CFG_FROM_DT_NODE(DT_ALIAS(adc_alias))          \
}

int32_t temperature_degC = 0;
struct k_event errors;

#define MCP9808_I2C_ADDR 0x18
#define MCP9808_REG_TEMP 0x05

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/* macros */
#define HEARTBEAT_HALF_PERIOD_MS 500
#define ERROR_BLINK_HALF_PERIOD_MS 500
#define ERROR_TIMEOUT_MS (2U * 60U * 1000U)

#define HEARTBEAT_STACK_SIZE 1024
#define HEARTBEAT_THREAD_PRIO 5

#define ADC_MAX_MV 3000
#define BATTERY_SAMPLE_INTERVAL_MS (60U * 1000U)
#define BATTERY_MAX_MV            3000
#define BATTERY_LOW_THRESHOLD_PCT 75U

#define LED_PWM_PERIOD_NS PWM_USEC(1000)

#define ECG_SAMPLE_INTERVAL_US 2500U   // 400 Hz
#define ECG_WINDOW_MS          2000U
#define ECG_SAMPLE_RATE_HZ     400.0
#define ECG_BUF_LEN            ((ECG_WINDOW_MS * 1000U) / ECG_SAMPLE_INTERVAL_US)

#define ECG_MIN_BPM            40U
#define ECG_MAX_BPM            200U

#define HR_LED_MIN_BPM 40U
#define HR_LED_MAX_BPM 200U
#define HR_LED_DUTY_PERCENT 25U
#define MS_PER_MINUTE 60000U

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
#define APP_EVENT_ERROR_BLINK       BIT(8)

#define APP_BUTTON_EVENT_MASK       (APP_EVENT_BUTTON0_IDLE | \
                                     APP_EVENT_BUTTON1_TEMP | \
                                     APP_EVENT_BUTTON2_ECG | \
                                     APP_EVENT_BUTTON3_RESET)

#define APP_MEASUREMENT_EVENT_MASK  (APP_EVENT_BATTERY_SAMPLE | \
                                     APP_EVENT_ECG_SAMPLE)

#define APP_SYSTEM_EVENT_MASK       (APP_EVENT_ERROR | \
                                     APP_EVENT_ERROR_TIMEOUT | \
                                     APP_EVENT_ERROR_BLINK)

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

static uint8_t map_mv_to_percent(int32_t mv)
{
    if (mv < 0) return 0;
    if (mv > BATTERY_MAX_MV) mv = BATTERY_MAX_MV;

    return (uint8_t)((mv * 100) / BATTERY_MAX_MV);
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
    int16_t ecg_buf[ECG_BUF_LEN];
    size_t ecg_buf_index;

    uint32_t error_code;
    bool terminate_requested;
    bool error_state_active;
    bool error_blink_on;

    bool hr_led_active;
    bool hr_led_on;
    uint32_t hr_led_period_ms;
    uint32_t hr_led_on_ms;
    uint32_t hr_led_off_ms;

    bool idle_active;
    bool low_power_requested;
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

static void set_error_blink_outputs(bool on);
void error_blink_timer_handler(struct k_timer *t);
void error_timeout_timer_handler(struct k_timer *t);

static int set_led1_pwm(uint8_t percent);
void battery_timer_handler(struct k_timer *t);

static int mcp9808_init(void);
static int mcp9808_read_temp(int32_t *temp_centi_c);

void ecg_sample_timer_handler(struct k_timer *t);
void ecg_sample_timer_stop_fn(struct k_timer *t);
static int read_ecg_sample_mv(int32_t *mv_out);
static int process_ecg_window(struct app_object *s);

static int update_hr_led_timing(struct app_object *s, uint16_t bpm);
static void stop_hr_led(struct app_object *s);
void hr_led_timer_handler(struct k_timer *t);

static int ble_init_wrapper(void);
static void ble_update_battery_mv(int32_t battery_mv);
static void ble_update_temperature(int32_t temp_centi_c);
static void ble_update_error(uint32_t error_code);

static void idle_exit(void *o);
static void battery_exit(void *o);
static void temperature_exit(void *o);

static void enter_low_power_mode(void);

K_TIMER_DEFINE(ecg_sample_timer, ecg_sample_timer_handler, ecg_sample_timer_stop_fn);
K_TIMER_DEFINE(error_blink_timer, error_blink_timer_handler, NULL);
K_TIMER_DEFINE(error_timeout_timer, error_timeout_timer_handler, NULL);
K_TIMER_DEFINE(battery_timer, battery_timer_handler, NULL);
K_TIMER_DEFINE(hr_led_timer, hr_led_timer_handler, NULL);

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

static void enter_low_power_mode(void)
{
    k_timer_stop(&battery_timer);
    k_timer_stop(&ecg_sample_timer);
    k_timer_stop(&error_blink_timer);
    k_timer_stop(&error_timeout_timer);
    k_timer_stop(&hr_led_timer);

    (void)set_led2_duty_cycle(0U);
    (void)set_led3_duty_cycle(0U);

    (void)gpio_pin_set_dt(&heartbeat_led, 0);
    (void)gpio_pin_set_dt(&iv_pump_led, 0);
    (void)gpio_pin_set_dt(&error_led, 0);

    LOG_INF("Entering low-power terminal state");

    while (1) {
        k_sleep(K_FOREVER);
    }
}

static void idle_exit(void *o)
{
    struct app_object *s = (struct app_object *)o;
    s->idle_active = false;
}

static void battery_exit(void *o)
{
    struct app_object *s = (struct app_object *)o;
    s->idle_active = false;
}

static void temperature_exit(void *o)
{
    struct app_object *s = (struct app_object *)o;
    s->idle_active = false;
}

static int ble_init_wrapper(void)
{
    int ret = bluetooth_init(&bluetooth_callbacks, &remote_service_callbacks);

    if (ret < 0) {
        LOG_ERR("Bluetooth init failed (%d)", ret);
        return ret;
    }

    LOG_INF("Bluetooth initialized");
    return 0;
}

static void ble_update_battery_mv(int32_t battery_mv)
{
    bluetooth_set_battery_level(battery_mv);
}

static void ble_update_temperature(int32_t temp_centi_c)
{
    temperature_degC = temp_centi_c / 100;
}

static void ble_update_error(uint32_t error_code)
{
    errors.events = error_code;
}

static int update_hr_led_timing(struct app_object *s, uint16_t bpm)
{
    if (s == NULL) {
        return -EINVAL;
    }

    if (bpm < HR_LED_MIN_BPM || bpm > HR_LED_MAX_BPM) {
        return -ERANGE;
    }

    s->hr_led_period_ms = MS_PER_MINUTE / bpm;
    s->hr_led_on_ms = (s->hr_led_period_ms * HR_LED_DUTY_PERCENT) / 100U;
    s->hr_led_off_ms = s->hr_led_period_ms - s->hr_led_on_ms;

    if (s->hr_led_on_ms == 0U || s->hr_led_off_ms == 0U) {
        return -EINVAL;
    }

    s->hr_led_active = true;
    s->hr_led_on = true;

    (void)set_led2_duty_cycle(100U);

    k_timer_start(&hr_led_timer,
                  K_MSEC(s->hr_led_on_ms),
                  K_NO_WAIT);

    LOG_INF("LED2 HR blink: bpm=%u period=%u ms on=%u ms off=%u ms",
            bpm,
            s->hr_led_period_ms,
            s->hr_led_on_ms,
            s->hr_led_off_ms);

    return 0;
}

static void stop_hr_led(struct app_object *s)
{
    if (s != NULL) {
        s->hr_led_active = false;
        s->hr_led_on = false;
    }

    k_timer_stop(&hr_led_timer);
    (void)set_led2_duty_cycle(0U);
}

static int process_ecg_window(struct app_object *s)
{
    double freq_hz;
    double bpm_d;
    int ret;

    if (s == NULL) {
        return -EINVAL;
    }

    freq_hz = calc_freq_zero_crossing(s->ecg_buf,
                                      ECG_BUF_LEN,
                                      ECG_SAMPLE_RATE_HZ);

    if (freq_hz <= 0.0) {
        return -EINVAL;
    }

    bpm_d = freq_hz * 60.0;

    if (bpm_d < ECG_MIN_BPM || bpm_d > ECG_MAX_BPM) {
        LOG_WRN("ECG BPM out of expected range: %.1f", bpm_d);
        return -ERANGE;
    }

    s->heart_rate_bpm = (uint16_t)(bpm_d + 0.5);
    s->heart_rate_avg_bpm = s->heart_rate_bpm;

    LOG_INF("ECG: freq=%.2f Hz, HR=%u BPM",
            freq_hz,
            s->heart_rate_avg_bpm);

    ret = update_hr_led_timing(s, s->heart_rate_avg_bpm);
    if (ret < 0) {
        LOG_WRN("Unable to update LED2 HR blink timing (%d)", ret);
        return ret;
    }

    return 0;
}

static int read_ecg_sample_mv(int32_t *mv_out)
{
    if (mv_out == NULL) {
        return -EINVAL;
    }

    return do_diff_single_sample(mv_out);
}

static int mcp9808_init(void)
{
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    return 0;
}

static int mcp9808_read_temp(int32_t *temp_centi_c)
{
    uint8_t reg = MCP9808_REG_TEMP;
    uint8_t buf[2];

    int ret = i2c_write_read(i2c_dev,
                             MCP9808_I2C_ADDR,
                             &reg, 1,
                             buf, 2);
    if (ret < 0) {
        LOG_ERR("I2C read failed (%d)", ret);
        return ret;
    }

    int16_t raw = ((buf[0] << 8) | buf[1]) & 0x1FFF;

    /* sign extend if negative */
    if (raw & 0x1000) {
        raw |= 0xE000;
    }

    /*
     * MCP9808 resolution: 0.0625°C per LSB
     * Convert to centi-degrees C (×100)
     */
    *temp_centi_c = (raw * 625) / 100;

    return 0;
}

static int set_led1_pwm(uint8_t percent)
{
    /*
     * For now: simple threshold mapping (clean + deterministic).
     * Later commit can upgrade to software PWM if desired.
     */
    if (percent == 0) {
        return gpio_pin_set_dt(&iv_pump_led, 0);
    }

    return gpio_pin_set_dt(&iv_pump_led, 1);
}

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
     * BUTTON1 remains enabled for temperature during ECG.
     * BUTTON2 remains enabled to stop ECG.
     * BUTTON3 remains enabled for reset.
     *
     * BUTTON0 will be added explicitly later.
     */
    (void)gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE);
    (void)gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE);
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

static void set_error_blink_outputs(bool on)
{
    int gpio_value = on ? 1 : 0;
    uint8_t pwm_duty = on ? 100U : 0U;

    (void)gpio_pin_set_dt(&heartbeat_led, gpio_value);
    (void)gpio_pin_set_dt(&iv_pump_led, gpio_value);
    (void)gpio_pin_set_dt(&error_led, gpio_value);

    (void)set_led2_duty_cycle(pwm_duty);
    (void)set_led3_duty_cycle(pwm_duty);
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
    s->ecg_buf_index = 0U;
    memset(s->ecg_buf, 0, sizeof(s->ecg_buf));

    s->error_code = APP_ERROR_NONE;
    s->terminate_requested = false;
    s->error_state_active = false;
    s->error_blink_on = false;

    s->hr_led_active = false;
    s->hr_led_on = false;
    s->hr_led_period_ms = 0U;
    s->hr_led_on_ms = 0U;
    s->hr_led_off_ms = 0U;

    s->idle_active = false;
    s->low_power_requested = false;

    return 0;
}

static enum smf_state_result init_run(void *o)
{
    struct app_object *s = (struct app_object *)o;
    int err, ret;

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

    ret = mcp9808_init();
    if (ret < 0) {
        post_error(s, APP_ERROR_I2C_INIT);
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return SMF_EVENT_HANDLED;
    }

    k_event_init(&errors);

    ret = ble_init_wrapper();
    if (ret < 0) {
        post_error(s, APP_ERROR_BLE_INIT);
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

    k_timer_start(&battery_timer,
              K_MSEC(10),
              K_MSEC(BATTERY_SAMPLE_INTERVAL_MS));

    smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
    return SMF_EVENT_HANDLED;
}

 static void idle_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;

    s->idle_active = true;

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

        stop_hr_led(s);
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
    int ret;

    disable_measurement_button_interrupts();

    ret = do_single_sample(s);
    if (ret < 0) {
        post_error(s, APP_ERROR_ADC_READ);
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return;
    }

    s->battery_mv = s->adc_mv;
    s->battery_percent = map_mv_to_percent(s->battery_mv);

    LOG_INF("Battery: %d mV (%u%%)",
            s->battery_mv,
            s->battery_percent);
    
    ble_update_battery_mv(s->battery_mv);

    ret = set_led1_pwm(s->battery_percent);
    if (ret < 0) {
        post_error(s, APP_ERROR_GPIO_INIT);
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return;
    }

    if (s->battery_percent < BATTERY_LOW_THRESHOLD_PCT) {
        post_error(s, APP_ERROR_LOW_BATTERY);

        LOG_ERR("Battery below threshold (%u%% < %u%%). Terminating.",
                s->battery_percent,
                BATTERY_LOW_THRESHOLD_PCT);

        s->terminate_requested = true;
        s->low_power_requested = true;
        smf_set_terminate(SMF_CTX(s), -EFAULT);
        return;
    }

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
    int ret;

    disable_measurement_button_interrupts();

    ret = mcp9808_read_temp(&s->temperature_centi_c);
    if (ret < 0) {
        post_error(s, APP_ERROR_I2C_READ);
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return;
    }

    LOG_INF("Temperature: %d.%02d C",
            s->temperature_centi_c / 100,
            abs(s->temperature_centi_c % 100));

    ble_update_temperature(s->temperature_centi_c);

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
    s->ecg_buf_index = 0U;
    memset(s->ecg_buf, 0, sizeof(s->ecg_buf));

    k_event_clear(&app_events, APP_EVENT_ECG_SAMPLE);

    k_timer_start(&ecg_sample_timer,
                  K_USEC(ECG_SAMPLE_INTERVAL_US),
                  K_USEC(ECG_SAMPLE_INTERVAL_US));

    LOG_INF("ECG: started live sampling at %.1f Hz", ECG_SAMPLE_RATE_HZ);
}

static enum smf_state_result ecg_run(void *o)
{
    struct app_object *s = (struct app_object *)o;

    uint32_t events = k_event_wait(&app_events,
                                   APP_EVENT_ECG_SAMPLE |
                                   APP_EVENT_BUTTON2_ECG |
                                   APP_EVENT_BUTTON1_TEMP |
                                   APP_EVENT_BUTTON3_RESET |
                                   APP_EVENT_ERROR,
                                   true,
                                   K_FOREVER);

    if (events & APP_EVENT_ERROR) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return SMF_EVENT_HANDLED;
    }

    if (events & APP_EVENT_BUTTON3_RESET) {
        s->ecg_active = false;
        stop_hr_led(s);
        smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
        return SMF_EVENT_HANDLED;
    }

    if (events & APP_EVENT_BUTTON2_ECG) {
        s->ecg_active = false;
        LOG_INF("ECG: stopped by BUTTON2");
        stop_hr_led(s);
        smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
        return SMF_EVENT_HANDLED;
    }

    if (events & APP_EVENT_BUTTON1_TEMP) {
        int ret = mcp9808_read_temp(&s->temperature_centi_c);
        if (ret < 0) {
            post_error(s, APP_ERROR_I2C_READ);
            smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
            return SMF_EVENT_HANDLED;
        }

        LOG_INF("Temperature during ECG: %d.%02d C",
                s->temperature_centi_c / 100,
                abs(s->temperature_centi_c % 100));

        ble_update_temperature(s->temperature_centi_c);
    }

    if (events & APP_EVENT_ECG_SAMPLE) {
        int32_t sample_mv;
        int ret = read_ecg_sample_mv(&sample_mv);

        if (ret < 0) {
            post_error(s, APP_ERROR_ADC_READ);
            smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
            return SMF_EVENT_HANDLED;
        }

        s->ecg_mv = sample_mv;

        if (s->ecg_buf_index < ECG_BUF_LEN) {
            s->ecg_buf[s->ecg_buf_index] = (int16_t)clamp_mv(sample_mv);
            s->ecg_buf_index++;
            s->ecg_sample_count++;
        }

        if (s->ecg_buf_index >= ECG_BUF_LEN) {
            ret = process_ecg_window(s);
            if (ret < 0) {
                LOG_WRN("ECG: unable to calculate valid heart rate from current window");
            }

            s->ecg_buf_index = 0U;
            memset(s->ecg_buf, 0, sizeof(s->ecg_buf));
        }
    }

    return SMF_EVENT_HANDLED;
}

static void ecg_exit(void *o)
{
    struct app_object *s = (struct app_object *)o;

    k_timer_stop(&ecg_sample_timer);
    k_event_clear(&app_events, APP_EVENT_ECG_SAMPLE);

    s->ecg_active = false;
    stop_hr_led(s);

    LOG_INF("ECG: exited");
}

static void error_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;

    s->ecg_active = false;
    s->error_state_active = true;
    s->error_blink_on = false;

    stop_hr_led(s);

    disable_measurement_button_interrupts();

    set_error_blink_outputs(false);

    k_timer_start(&error_blink_timer,
                  K_NO_WAIT,
                  K_MSEC(ERROR_BLINK_HALF_PERIOD_MS));

    k_timer_start(&error_timeout_timer,
                  K_MSEC(ERROR_TIMEOUT_MS),
                  K_NO_WAIT);

    LOG_ERR("Entered ERROR state with error_code=0x%08x", s->error_code);

    ble_update_error(s->error_code);
}

static enum smf_state_result error_run(void *o)
{
    struct app_object *s = (struct app_object *)o;

    uint32_t events = k_event_wait(&app_events,
                                   APP_EVENT_BUTTON3_RESET |
                                   APP_EVENT_ERROR_TIMEOUT |
                                   APP_EVENT_ERROR_BLINK,
                                   true,
                                   K_FOREVER);

    LOG_INF("ERROR event mask: 0x%08x", events);

    if (events & APP_EVENT_ERROR_BLINK) {
        s->error_blink_on = !s->error_blink_on;
        set_error_blink_outputs(s->error_blink_on);
        return SMF_EVENT_HANDLED;
    }

    if (events & APP_EVENT_ERROR_TIMEOUT) {
        s->terminate_requested = true;
        s->low_power_requested = true;
        LOG_ERR("ERROR timeout reached; requesting graceful termination");
        smf_set_terminate(SMF_CTX(s), -ETIMEDOUT);
        return SMF_EVENT_HANDLED;
    }

    if (events & APP_EVENT_BUTTON3_RESET) {
        clear_error(s);
        ble_update_error(0);
        s->terminate_requested = false;
        smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
        return SMF_EVENT_HANDLED;
    }

    return SMF_EVENT_HANDLED;
}

static void error_exit(void *o)
{
    struct app_object *s = (struct app_object *)o;

    k_timer_stop(&error_blink_timer);
    k_timer_stop(&error_timeout_timer);

    s->error_state_active = false;
    s->error_blink_on = false;

    set_error_blink_outputs(false);
}

static const struct smf_state app_states[] = {
    [STATE_INIT]        = SMF_CREATE_STATE(NULL,              init_run,         NULL,             NULL, NULL),
    [STATE_IDLE]        = SMF_CREATE_STATE(idle_entry,        idle_run,         idle_exit,        NULL, NULL),
    [STATE_BATTERY]     = SMF_CREATE_STATE(battery_entry,     battery_run,      battery_exit,     NULL, NULL),
    [STATE_TEMPERATURE] = SMF_CREATE_STATE(temperature_entry, temperature_run,  temperature_exit, NULL, NULL),
    [STATE_ECG]         = SMF_CREATE_STATE(ecg_entry,         ecg_run,          ecg_exit,         NULL, NULL),
    [STATE_ERROR]       = SMF_CREATE_STATE(error_entry,       error_run,        error_exit,       NULL, NULL),
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

    if (s_obj.low_power_requested) {
        enter_low_power_mode();
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

void ecg_sample_timer_handler(struct k_timer *t)
{
    ARG_UNUSED(t);

    k_event_post(&app_events, APP_EVENT_ECG_SAMPLE);
}

void ecg_sample_timer_stop_fn(struct k_timer *t)
{
    ARG_UNUSED(t);
}

void error_blink_timer_handler(struct k_timer *t)
{
    ARG_UNUSED(t);

    k_event_post(&app_events, APP_EVENT_ERROR_BLINK);
}

void error_timeout_timer_handler(struct k_timer *t)
{
    ARG_UNUSED(t);

    k_event_post(&app_events, APP_EVENT_ERROR_TIMEOUT);
}

void battery_timer_handler(struct k_timer *t)
{
    ARG_UNUSED(t);

    if (s_obj.idle_active) {
        k_event_post(&app_events, APP_EVENT_BATTERY_SAMPLE);
    }
}

void hr_led_timer_handler(struct k_timer *t)
{
    ARG_UNUSED(t);

    if (!s_obj.hr_led_active) {
        return;
    }

    if (s_obj.hr_led_on) {
        s_obj.hr_led_on = false;
        (void)set_led2_duty_cycle(0U);

        k_timer_start(&hr_led_timer,
                      K_MSEC(s_obj.hr_led_off_ms),
                      K_NO_WAIT);
    } else {
        s_obj.hr_led_on = true;
        (void)set_led2_duty_cycle(100U);

        k_timer_start(&hr_led_timer,
                      K_MSEC(s_obj.hr_led_on_ms),
                      K_NO_WAIT);
    }
}

/* threads */

void heartbeat_thread(void *, void *, void *)
{
    gpio_pin_set_dt(&heartbeat_led, 0);

    while (1) {
        uint64_t now;

        /*
         * In ERROR, LED0 is controlled by the ERROR blink state so that
         * all four LEDs blink in phase.
         */
        if (s_obj.error_state_active) {
            k_msleep(HEARTBEAT_HALF_PERIOD_MS);
            continue;
        }

        gpio_pin_toggle_dt(&heartbeat_led);

        now = now_ns();
        if (s_obj.hb_last_ns != 0U) {
            LOG_INF("heartbeat toggle period ns: %llu",
                    (unsigned long long)(now - s_obj.hb_last_ns));
        }
        s_obj.hb_last_ns = now;

        k_msleep(HEARTBEAT_HALF_PERIOD_MS);
    }
}