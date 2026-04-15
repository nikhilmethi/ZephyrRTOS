#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/smf.h> 
#include <zephyr/drivers/adc.h> 
// #include "calc_freq.h"
#include <zephyr/drivers/pwm.h> // CONFIG_PWM=y
// #include "ble-lib.h" // BME554 BLE library (remember to add to CMakeLists.txt)

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

#define SIN_SAMPLE_INTERVAL_MS 5      // 200 Hz
#define SIN_DURATION_MS        2000   // 2 seconds
#define SIN_NUM_SAMPLES        (SIN_DURATION_MS / SIN_SAMPLE_INTERVAL_MS)

extern void heartbeat_thread(void *, void *, void *);

/* heartbeat thread */
K_THREAD_DEFINE(heartbeat_thread_id,
                HEARTBEAT_STACK_SIZE,
                heartbeat_thread,
                NULL, NULL, NULL,
                HEARTBEAT_THREAD_PRIO, 0, 0);

/* button events (main-facing) */
#define BTN_SINGLE_SAMPLE_EVENT BIT(0)
#define BTN_RESET_EVENT         BIT(1)
#define BTN_SINUSOID_EVENT      BIT(2)
#define BTN_EVENT_MASK          (BTN_SINGLE_SAMPLE_EVENT | BTN_RESET_EVENT | BTN_SINUSOID_EVENT)

K_EVENT_DEFINE(button_events);

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

/* Placeholder for future PWM mapping */
static uint8_t map_mv_to_duty(int32_t mv)
{
    int32_t mv_clamped = clamp_mv(mv);
    return (uint8_t)((mv_clamped * 100) / ADC_MAX_MV);
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

static const struct adc_dt_spec adc_single = 
    ADC_DT_SPEC_GET_BY_ALIAS(vadc_single);

static const struct adc_dt_spec adc_diff = 
    ADC_DT_SPEC_GET_BY_ALIAS(vadc_diff);

static const struct gpio_dt_spec freq_up_button = 
    GPIO_DT_SPEC_GET(DT_ALIAS(frequpbutton), gpios);

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
    STATE_SINGLE_SAMPLE,
    STATE_SINUSOID,
    STATE_ERROR,
};

struct app_object {
    struct smf_ctx ctx;

    uint64_t hb_last_ns;

    int16_t adc_raw;
    int32_t adc_mv;

    uint32_t sin_sample_count;
    bool sin_active;
};

static struct app_object s_obj;
static const struct smf_state app_states[];

/* SMF helper prototypes */
static int init_app_object(struct app_object *s);
static void set_led1(bool on);
static void enable_single_sample_button(void);
static void disable_single_sample_button(void);
static int do_single_sample(struct app_object *s);
static int setup_adc_diff(void);
static void enable_sinusoid_button(void);
static void disable_sinusoid_button(void);
static int set_led2_duty_cycle(uint8_t duty_percent);
static int set_led3_duty_cycle(uint8_t duty_percent);
static int do_diff_single_sample(int32_t *mv_out);

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

static void enable_sinusoid_button(void)
{
    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE);
}

static void disable_sinusoid_button(void)
{
    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
}

/* SMF state handlers */
static void idle_entry(void *o);
static enum smf_state_result idle_run(void *o);
static void single_sample_entry(void *o);
static enum smf_state_result single_sample_run(void *o);
static void sinusoid_entry(void *o);
static enum smf_state_result sinusoid_run(void *o);

static int init_app_object(struct app_object *s)
{
    if (s == NULL) {
        return -1;
    }

    s->hb_last_ns = 0;

    s->adc_raw = 0;
    s->adc_mv = 0;

    s->sin_sample_count = 0;
    s->sin_active = false;

    return 0;
}

static void set_led1(bool on)
{
    gpio_pin_set_dt(&iv_pump_led, on ? 1 : 0);
}

static void enable_single_sample_button(void)
{
    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE);
}

static void disable_single_sample_button(void)
{
    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_DISABLE);
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

    k_event_clear(&button_events, BTN_EVENT_MASK);

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
    ARG_UNUSED(o);

    gpio_pin_set_dt(&error_led, 0);
    set_led1(false);
    enable_single_sample_button();
    disable_sinusoid_button();

    LOG_INF("IDLE: BUTTON1 updates LED2 PWM, BUTTON2 starts 2 second LED3 sinusoidal PWM");
}

static enum smf_state_result idle_run(void *o)
{
    struct app_object *s = (struct app_object *)o;
    uint32_t events = k_event_wait(&button_events, BTN_EVENT_MASK, true, K_FOREVER);

    LOG_INF("Button Event Posted: %u", events);

    if (events & BTN_RESET_EVENT) {
        int ret;

        ret = set_led2_duty_cycle(0);
        if (ret < 0) {
            LOG_ERR("Failed to reset LED2 PWM (%d)", ret);
            smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
            return SMF_EVENT_HANDLED;
        }

        ret = set_led3_duty_cycle(0);
        if (ret < 0) {
            LOG_ERR("Failed to reset LED3 PWM (%d)", ret);
            smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
            return SMF_EVENT_HANDLED;
        }

        LOG_INF("Reset event: LED2 and LED3 PWM set to 0%% duty");
        smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
        return SMF_EVENT_HANDLED;
    }

    if (events & BTN_SINGLE_SAMPLE_EVENT) {
        LOG_INF("Single sample event: updating LED2 PWM from AIN0");
        smf_set_state(SMF_CTX(s), &app_states[STATE_SINGLE_SAMPLE]);
        return SMF_EVENT_HANDLED;
    }

    if (events & BTN_SINUSOID_EVENT) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_SINUSOID]);
        return SMF_EVENT_HANDLED;
    }

    return SMF_EVENT_HANDLED;
}

static void single_sample_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;
    int ret;
    uint8_t duty;

    disable_single_sample_button();
    disable_sinusoid_button();

    ret = do_single_sample(s);
    if (ret < 0) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return;
    }

    duty = map_mv_to_duty(s->adc_mv);

    ret = set_led2_duty_cycle(duty);
    if (ret < 0) {
        LOG_ERR("Failed to set LED2 duty cycle (%d)", ret);
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return;
    }

    LOG_INF("LED2 PWM updated to %u%% duty", duty);

    smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
}

static enum smf_state_result single_sample_run(void *o)
{
    ARG_UNUSED(o);
    return SMF_EVENT_HANDLED;
}

static void sinusoid_entry(void *o)
{
    struct app_object *s = o;

    disable_single_sample_button();
    disable_sinusoid_button();

    s->sin_sample_count = 0;
    s->sin_active = true;

    LOG_INF("BUTTON2 pressed: starting 2 second sinusoidal PWM modulation");
}

static enum smf_state_result sinusoid_run(void *o)
{
    struct app_object *s = o;

    uint32_t events = k_event_wait(&button_events, BTN_RESET_EVENT, true, K_NO_WAIT);
    if (events & BTN_RESET_EVENT) {
        int ret = set_led3_duty_cycle(0);
        if (ret < 0) {
            LOG_ERR("Failed to reset LED3 PWM (%d)", ret);
            smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
            return SMF_EVENT_HANDLED;
        }

        s->sin_active = false;
        LOG_INF("Sinusoidal PWM interrupted by reset");
        smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
        return SMF_EVENT_HANDLED;
    }

    if (!s->sin_active) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
        return SMF_EVENT_HANDLED;
    }

    if (s->sin_sample_count >= SIN_NUM_SAMPLES) {
        int ret = set_led3_duty_cycle(0);
        if (ret < 0) {
            LOG_ERR("Failed to stop LED3 PWM (%d)", ret);
            smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
            return SMF_EVENT_HANDLED;
        }

        s->sin_active = false;
        LOG_INF("Sinusoidal PWM complete after %u samples", s->sin_sample_count);
        smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
        return SMF_EVENT_HANDLED;
    }

    int32_t mv;
    int ret = do_diff_single_sample(&mv);
    if (ret < 0) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return SMF_EVENT_HANDLED;
    }

    /* shift bipolar signal into 0–3V range */
    int32_t shifted_mv = mv + (ADC_MAX_MV / 2);
    uint8_t duty = map_mv_to_duty(shifted_mv);

    ret = set_led3_duty_cycle(duty);
    if (ret < 0) {
        LOG_ERR("Failed to set LED3 duty (%d)", ret);
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return SMF_EVENT_HANDLED;
    }

    s->sin_sample_count++;

    k_msleep(SIN_SAMPLE_INTERVAL_MS);

    return SMF_EVENT_HANDLED;
}

static void error_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;

    ARG_UNUSED(s);

    set_led1(false);
    (void)set_led2_duty_cycle(0);
    (void)set_led3_duty_cycle(0);
    disable_single_sample_button();
    gpio_pin_set_dt(&error_led, 1);
    disable_sinusoid_button();

    LOG_ERR("Entered ERROR state");
}

static enum smf_state_result error_run(void *o)
{
    struct app_object *s = (struct app_object *)o;
    uint32_t events = k_event_wait(&button_events, BTN_RESET_EVENT, true, K_FOREVER);

    LOG_INF("Button Event Posted: %u", events);
    smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
    return SMF_EVENT_HANDLED;
}

static void error_exit(void *o)
{
    ARG_UNUSED(o);
    gpio_pin_set_dt(&error_led, 0);
}

static const struct smf_state app_states[] = {
    [STATE_INIT]          = SMF_CREATE_STATE(NULL,               init_run,            NULL, NULL, NULL),
    [STATE_IDLE]          = SMF_CREATE_STATE(idle_entry,         idle_run,            NULL, NULL, NULL),
    [STATE_SINGLE_SAMPLE] = SMF_CREATE_STATE(single_sample_entry, single_sample_run, NULL, NULL, NULL),
    [STATE_SINUSOID] = SMF_CREATE_STATE(sinusoid_entry, sinusoid_run,  NULL, NULL, NULL),
    [STATE_ERROR]         = SMF_CREATE_STATE(error_entry,        error_run,           error_exit, NULL, NULL),
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

    uint32_t period = PWM_USEC(1000);
    uint32_t pulse = (period * (100 - duty_percent)) / 100;

    return pwm_set_dt(&led2_pwm, period, pulse);
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

    uint32_t period = PWM_USEC(1000);
    uint32_t pulse = (period * (100 - duty_percent)) / 100;

    return pwm_set_dt(&led3_pwm, period, pulse);
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
    k_event_post(&button_events, BTN_SINGLE_SAMPLE_EVENT);
}

void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_event_post(&button_events, BTN_RESET_EVENT);
}

void freq_up_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_event_post(&button_events, BTN_SINUSOID_EVENT);
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