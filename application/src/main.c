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
// #include "ble-lib.h" // BME554 BLE library (remember to add to CMakeLists.txt)

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);
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

#define LED1_MIN_HZ 1
#define LED1_MAX_HZ 5
#define ADC_MAX_MV 3000

#define LED1_DUTY_PERCENT 10
#define LED1_ACTIVE_DURATION_MS 5000

#define DIFF_SAMPLE_INTERVAL_US 5000   // 5 ms → 200 Hz sampling
#define DIFF_SIGNAL_FREQ_HZ     10     // expected input signal
#define DIFF_NUM_CYCLES         20

#define DIFF_BUFFER_LEN ((DIFF_NUM_CYCLES * 1000000) / \
                        (DIFF_SIGNAL_FREQ_HZ * DIFF_SAMPLE_INTERVAL_US))

#define ADC_ASYNC_TIMEOUT_MS 2000

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
#define BTN_DIFF_BUFFERED_EVENT BIT(2)
#define BTN_EVENT_MASK          (BTN_SINGLE_SAMPLE_EVENT | BTN_RESET_EVENT | BTN_DIFF_BUFFERED_EVENT)

#define LED1_TIMER_EVENT        BIT(3)
#define LED1_DONE_EVENT         BIT(4)

K_EVENT_DEFINE(button_events);

void led1_blink_timer_handler(struct k_timer *t);
void led1_blink_timer_stop(struct k_timer *t);
void led1_done_timer_handler(struct k_timer *t);
void led1_done_timer_stop(struct k_timer *t);

K_TIMER_DEFINE(led1_blink_timer, led1_blink_timer_handler, led1_blink_timer_stop);
K_TIMER_DEFINE(led1_done_timer, led1_done_timer_handler, led1_done_timer_stop);

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

static int period_ms(double freq)
{
    return (int)(1000.0 / freq);
}

static int on_time_ms(double freq)
{
    int p = period_ms(freq);
    int on = (int)((p * LED1_DUTY_PERCENT) / 100.0);

    if (on < 1) on = 1;
    return on;
}

static int off_time_ms(double freq)
{
    int p = period_ms(freq);
    int off = p - on_time_ms(freq);

    if (off < 1) off = 1;
    return off;
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
    STATE_LED1_ACTIVE,
    STATE_DIFF_BUFFERED,
    STATE_ERROR,
};

struct app_object {
    struct smf_ctx ctx;

    uint64_t hb_last_ns;

    bool led1_is_on;

    int16_t adc_raw;
    int32_t adc_mv;
    double led1_freq_hz;

    /* buffered differential ADC */
    int16_t diff_buf[DIFF_BUFFER_LEN];
    double diff_freq_hz;

    /* async ADC support */
    struct k_poll_signal adc_async_signal;
    struct k_poll_event adc_async_evt;
};

static struct app_object s_obj;
static const struct smf_state app_states[];

/* SMF helper prototypes */
static int init_app_object(struct app_object *s);
static void set_led1(bool on);
static void enable_single_sample_button(void);
static void disable_single_sample_button(void);
static void start_led1_timers(struct app_object *s);
static int do_single_sample(struct app_object *s);
static int setup_adc_diff(void);
static void enable_diff_buffered_button(void);
static void disable_diff_buffered_button(void);
static enum adc_action adc_async_callback(const struct device *dev,
                                          const struct adc_sequence *sequence,
                                          uint16_t sampling_index);
static int do_diff_buffered_sample_async(struct app_object *s);
static void reset_adc_async_poll(struct app_object *s);
static int fail_adc_async(struct app_object *s, int err, const char *msg);

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

static void enable_diff_buffered_button(void)
{
    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE);
}

static void disable_diff_buffered_button(void)
{
    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
}

/* SMF state handlers */
static void idle_entry(void *o);
static enum smf_state_result idle_run(void *o);
static void single_sample_entry(void *o);
static enum smf_state_result single_sample_run(void *o);
static void led1_active_entry(void *o);
static enum smf_state_result led1_active_run(void *o);
static void diff_buffered_entry(void *o);
static enum smf_state_result diff_buffered_run(void *o);

static int init_app_object(struct app_object *s)
{
    if (s == NULL) {
        return -1;
    }

    s->hb_last_ns = 0;

    s->led1_is_on = false;

    s->adc_raw = 0;
    s->adc_mv = 0;
    s->led1_freq_hz = (double)LED1_MIN_HZ;

    s->diff_freq_hz = 0.0;

    for (size_t i = 0; i < DIFF_BUFFER_LEN; i++) {
        s->diff_buf[i] = 0;
    }

    k_poll_signal_init(&s->adc_async_signal);

    s->adc_async_evt = (struct k_poll_event)K_POLL_EVENT_INITIALIZER(
        K_POLL_TYPE_SIGNAL,
        K_POLL_MODE_NOTIFY_ONLY,
        &s->adc_async_signal
    );

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

static void start_led1_timers(struct app_object *s)
{
    int on_ms = on_time_ms(s->led1_freq_hz);
    int off_ms = off_time_ms(s->led1_freq_hz);

    s->led1_is_on = true;
    set_led1(true);

    k_timer_start(&led1_blink_timer, K_MSEC(on_ms), K_NO_WAIT);
    k_timer_start(&led1_done_timer, K_MSEC(LED1_ACTIVE_DURATION_MS), K_NO_WAIT);

    int freq_mhz = (int)(s->led1_freq_hz * 1000.0);

    LOG_INF("LED1 blinking for %d ms at %d.%03d Hz (on=%d ms, off=%d ms)",
            LED1_ACTIVE_DURATION_MS,
            freq_mhz / 1000,
            freq_mhz % 1000,
            on_ms,
            off_ms);
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

    k_event_clear(&button_events, BTN_EVENT_MASK | LED1_TIMER_EVENT | LED1_DONE_EVENT);

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

    LOG_INF("Diff ADC buffer: %d samples, interval=%d us",
        DIFF_BUFFER_LEN, DIFF_SAMPLE_INTERVAL_US);

    LOG_INF("Async ADC timeout: %d ms", ADC_ASYNC_TIMEOUT_MS);

    smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
    return SMF_EVENT_HANDLED;
}

static void idle_entry(void *o)
{
    ARG_UNUSED(o);

    gpio_pin_set_dt(&error_led, 0);
    set_led1(false);
    enable_single_sample_button();
    enable_diff_buffered_button();

    LOG_INF("IDLE");
}

static enum smf_state_result idle_run(void *o)
{
    struct app_object *s = (struct app_object *)o;
    uint32_t events = k_event_wait(&button_events, BTN_EVENT_MASK, true, K_FOREVER);

    LOG_INF("Button Event Posted: %u", events);

    if (events & BTN_RESET_EVENT) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
        return SMF_EVENT_HANDLED;
    }
    /*
    if (events & BTN_SINGLE_SAMPLE_EVENT) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_SINGLE_SAMPLE]);
        return SMF_EVENT_HANDLED;
    }
    */

    if (events & BTN_SINGLE_SAMPLE_EVENT) {
        LOG_INF("Single sample event (PWM not yet implemented)");
        smf_set_state(SMF_CTX(s), &app_states[STATE_SINGLE_SAMPLE]);
        return SMF_EVENT_HANDLED;
    }

    if (events & BTN_DIFF_BUFFERED_EVENT) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_DIFF_BUFFERED]);
        return SMF_EVENT_HANDLED;
    }

    return SMF_EVENT_HANDLED;
}

static void single_sample_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;
    int ret;

    disable_single_sample_button();
    disable_diff_buffered_button();

    ret = do_single_sample(s);
    if (ret < 0) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return;
    }

    /* Placeholder: will drive PWM in next commit */
    smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
}

static enum smf_state_result single_sample_run(void *o)
{
    ARG_UNUSED(o);
    return SMF_EVENT_HANDLED;
}

static void led1_active_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;
    start_led1_timers(s);
}

static enum smf_state_result led1_active_run(void *o)
{
    struct app_object *s = (struct app_object *)o;
    uint32_t events = k_event_wait(&button_events,
                                   BTN_RESET_EVENT | LED1_TIMER_EVENT | LED1_DONE_EVENT,
                                   true,
                                   K_FOREVER);

    if (events & BTN_RESET_EVENT) {
        k_timer_stop(&led1_blink_timer);
        k_timer_stop(&led1_done_timer);
        set_led1(false);
        smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
        return SMF_EVENT_HANDLED;
    }

    if (events & LED1_TIMER_EVENT) {
        int next_ms;

        s->led1_is_on = !s->led1_is_on;
        set_led1(s->led1_is_on);

        if (s->led1_is_on) {
            next_ms = on_time_ms(s->led1_freq_hz);
        } else {
            next_ms = off_time_ms(s->led1_freq_hz);
        }

        k_timer_start(&led1_blink_timer, K_MSEC(next_ms), K_NO_WAIT);
    }

    if (events & LED1_DONE_EVENT) {
        k_timer_stop(&led1_blink_timer);
        set_led1(false);
        smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
        return SMF_EVENT_HANDLED;
    }

    return SMF_EVENT_HANDLED;
}

static void diff_buffered_entry(void *o)
{
    LOG_INF("Starting async buffered differential ADC sequence");

    struct app_object *s = (struct app_object *)o;
    int ret;

    disable_single_sample_button();
    disable_diff_buffered_button();

    ret = do_diff_buffered_sample_async(s);
    if (ret < 0) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return;
    }

    LOG_INF("Async buffered differential ADC sequence complete → returning to IDLE");
    smf_set_state(SMF_CTX(s), &app_states[STATE_IDLE]);
}

static enum smf_state_result diff_buffered_run(void *o)
{
    ARG_UNUSED(o);
    return SMF_EVENT_HANDLED;
}

static void error_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;

    ARG_UNUSED(s);

    k_timer_stop(&led1_blink_timer);
    k_timer_stop(&led1_done_timer);
    set_led1(false);
    disable_single_sample_button();
    gpio_pin_set_dt(&error_led, 1);
    disable_diff_buffered_button();

    LOG_ERR("Entered ERROR state; async/sync ADC flow aborted");
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
    [STATE_LED1_ACTIVE]   = SMF_CREATE_STATE(led1_active_entry,  led1_active_run,     NULL, NULL, NULL),
    [STATE_DIFF_BUFFERED] = SMF_CREATE_STATE(diff_buffered_entry, diff_buffered_run,  NULL, NULL, NULL),
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

    uint32_t period = PWM_USEC(1000); // 1 kHz PWM (placeholder)
    uint32_t pulse = (period * duty_percent) / 100;

    return pwm_set_dt(&led2_pwm, period, pulse);
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

    LOG_INF("ADC raw=%d, mv=%d, duty=%d%%",
            s->adc_raw,
            s->adc_mv,
            duty);

    return 0;
}

static int do_diff_buffered_sample_async(struct app_object *s)
{
    int ret;
    int signaled;
    int poll_result;
    double sample_rate_hz = 1000000.0 / (double)DIFF_SAMPLE_INTERVAL_US;

    struct adc_sequence_options options = {
        .extra_samplings = DIFF_BUFFER_LEN - 1,
        .interval_us = DIFF_SAMPLE_INTERVAL_US,
        .callback = adc_async_callback,
    };

    struct adc_sequence sequence = {
        .options = &options,
        .buffer = s->diff_buf,
        .buffer_size = sizeof(s->diff_buf),
    };

    reset_adc_async_poll(s);

    (void)adc_sequence_init_dt(&adc_diff, &sequence);

    ret = adc_read_async(adc_diff.dev, &sequence, &s->adc_async_signal);
    if (ret < 0) {
        return fail_adc_async(s, ret, "Differential ADC async start failed");
    }

    LOG_INF("Async differential ADC acquisition started");

    ret = k_poll(&s->adc_async_evt, 1, K_MSEC(ADC_ASYNC_TIMEOUT_MS));
    if (ret == -EAGAIN) {
        return fail_adc_async(s, -ETIMEDOUT, "ADC async acquisition timed out");
    }
    if (ret < 0) {
        return fail_adc_async(s, ret, "ADC async poll failed");
    }

    if (s->adc_async_evt.state != K_POLL_STATE_SIGNALED) {
        return fail_adc_async(s, -EIO, "ADC async poll returned without signal");
    }

    k_poll_signal_check(&s->adc_async_signal, &signaled, &poll_result);
    if (!signaled) {
        return fail_adc_async(s, -EIO, "ADC async signal not raised");
    }

    reset_adc_async_poll(s);

    LOG_INF("Async differential ADC read complete");
    LOG_HEXDUMP_INF(s->diff_buf, sizeof(s->diff_buf), "async_diff_buf");

    s->diff_freq_hz = calc_freq_zero_crossing(s->diff_buf,
                                              DIFF_BUFFER_LEN,
                                              sample_rate_hz);

    if (s->diff_freq_hz < 0.0) {
        return fail_adc_async(s, -EINVAL, "Differential ADC frequency calculation failed");
    }

    LOG_INF("Async buffered signal frequency: %.3f Hz", s->diff_freq_hz);

    return 0;
}

static enum adc_action adc_async_callback(const struct device *dev,
                                          const struct adc_sequence *sequence,
                                          uint16_t sampling_index)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(sequence);

#if CONFIG_LOG_DEFAULT_LEVEL >= 4
    LOG_DBG("ADC async sample index: %u", sampling_index);
#endif

    /* For now: continue normally through the sequence */
    return ADC_ACTION_CONTINUE;
}

static void reset_adc_async_poll(struct app_object *s)
{
    unsigned int signaled;
    int result;

    k_poll_signal_check(&s->adc_async_signal, &signaled, &result);
    ARG_UNUSED(signaled);
    ARG_UNUSED(result);

    k_poll_signal_reset(&s->adc_async_signal);
    s->adc_async_evt.state = K_POLL_STATE_NOT_READY;
}

static int fail_adc_async(struct app_object *s, int err, const char *msg)
{
    LOG_ERR("%s (%d)", msg, err);
    reset_adc_async_poll(s);
    return err;
}

/* timer handlers */

void led1_blink_timer_handler(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_event_post(&button_events, LED1_TIMER_EVENT);
}

void led1_blink_timer_stop(struct k_timer *t)
{
    ARG_UNUSED(t);
}

void led1_done_timer_handler(struct k_timer *t)
{
    ARG_UNUSED(t);
    k_event_post(&button_events, LED1_DONE_EVENT);
}

void led1_done_timer_stop(struct k_timer *t)
{
    ARG_UNUSED(t);
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
    k_event_post(&button_events, BTN_DIFF_BUFFERED_EVENT);
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