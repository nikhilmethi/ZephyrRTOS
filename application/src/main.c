#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/smf.h> // CONFIG_SMF=y

// #include <zephyr/drivers/adc.h> // CONFIG_ADC=y
// #include <zephyr/drivers/pwm.h> // CONFIG_PWM=y
// #include "ble-lib.h" // BME554 BLE library (remember to add to CMakeLists.txt)

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* macros */
#define LED_BLINK_FREQ_HZ 2
#define FREQ_UP_INC_HZ 1
#define FREQ_DOWN_INC_HZ 1

#define ACTION_FREQ_MIN_HZ 1
#define ACTION_FREQ_MAX_HZ 5

#define HEARTBEAT_ON_MS 250
#define HEARTBEAT_OFF_MS 750

#define HEARTBEAT_STACK_SIZE 1024
#define HEARTBEAT_THREAD_PRIO 5

extern void heartbeat_thread(void *, void *, void *);

/* heartbeat thread */
K_THREAD_DEFINE(heartbeat_thread_id,
                HEARTBEAT_STACK_SIZE,
                heartbeat_thread,
                NULL, NULL, NULL,
                HEARTBEAT_THREAD_PRIO, 0, 0);

/* button events (main-facing) */
#define BTN_SLEEP_EVENT         BIT(0)
#define BTN_RESET_EVENT         BIT(1)
#define BTN_FREQ_UP_EVENT       BIT(2)
#define BTN_FREQ_DOWN_EVENT     BIT(3)

/* double press events (main-facing) */
#define BTN_FREQ_UP_DBL_EVENT   BIT(4)
#define BTN_FREQ_DOWN_DBL_EVENT BIT(5)

#define BTN_EVENT_MASK (BTN_SLEEP_EVENT | BTN_RESET_EVENT | \
                        BTN_FREQ_UP_EVENT | BTN_FREQ_DOWN_EVENT | \
                        BTN_FREQ_UP_DBL_EVENT | BTN_FREQ_DOWN_DBL_EVENT)

K_EVENT_DEFINE(button_events);

/* raw freq-only events (ISR -> doublepress thread) */
#define RAW_FREQ_UP_EVENT   BIT(0)
#define RAW_FREQ_DOWN_EVENT BIT(1)
#define RAW_FREQ_MASK (RAW_FREQ_UP_EVENT | RAW_FREQ_DOWN_EVENT)

K_EVENT_DEFINE(raw_freq_events);

/* double press config */
#define DOUBLE_PRESS_WINDOW_MS 500

#define DBLP_STACK_SIZE 1024
#define DBLP_THREAD_PRIO 5

extern void doublepress_thread(void *, void *, void *);

/* doublepress thread */
K_THREAD_DEFINE(doublepress_thread_id,
                DBLP_STACK_SIZE,
                doublepress_thread,
                NULL, NULL, NULL,
                DBLP_THREAD_PRIO, 0, 0);

/* timer prototypes */
void action_timer_handler(struct k_timer *t);
void action_timer_stop(struct k_timer *t);

K_TIMER_DEFINE(action_timer, action_timer_handler, action_timer_stop);

/* helper timing functions */

static inline uint64_t now_ns(void)
{
    int64_t ticks = k_uptime_ticks();
    return k_ticks_to_ns_near64(ticks);
}

static inline int32_t action_half_period_ms(int freq_hz)
{
    return 1000 / (2 * freq_hz);
}

/* hardware definitions */

static const struct gpio_dt_spec heartbeat_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(heartbeat), gpios);

static const struct gpio_dt_spec iv_pump_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(ivpump), gpios);

static const struct gpio_dt_spec buzzer_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(buzzer), gpios);

static const struct gpio_dt_spec error_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(error), gpios);

static const struct gpio_dt_spec sleep_button =
    GPIO_DT_SPEC_GET(DT_ALIAS(sleepbutton), gpios);

static const struct gpio_dt_spec reset_button =
    GPIO_DT_SPEC_GET(DT_ALIAS(resetbutton), gpios);

static const struct gpio_dt_spec freq_up_button =
    GPIO_DT_SPEC_GET(DT_ALIAS(frequpbutton), gpios);

static const struct gpio_dt_spec freq_down_button =
    GPIO_DT_SPEC_GET(DT_ALIAS(freqdownbutton), gpios);

/* callback prototypes */
void sleep_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void freq_up_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void freq_down_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/* callback structs */
static struct gpio_callback sleep_button_cb;
static struct gpio_callback reset_button_cb;
static struct gpio_callback freq_up_button_cb;
static struct gpio_callback freq_down_button_cb;

/* SMF state machine */
enum app_state {
    STATE_INIT,
    STATE_DEFAULTS,
    STATE_AWAKE,
    STATE_SLEEP,
    STATE_ERROR,
};

struct app_object {
    struct smf_ctx ctx;   /* must be first */

    int action_freq_hz;
    int stored_action_freq_hz;

    bool action_phase;
    bool stored_action_phase;

    int32_t stored_action_remaining_ms;

    uint64_t hb_last_ns;
    uint64_t action_last_ns;

    bool wake_from_sleep;
    bool threads_started;
};

static struct app_object s_obj;
static const struct smf_state app_states[];

/* SMF helper prototypes */
static int init_app_object(struct app_object *s);
static void set_action_outputs_from_phase(bool phase);
static void clear_action_outputs(void);
static void enable_awake_buttons(void);
static void disable_awake_buttons(void);
static void start_action_timer_for_freq(struct app_object *s);
static void restore_action_timer_after_sleep(struct app_object *s);

/* SMF state handlers */
static enum smf_state_result init_run(void *o);
static void defaults_entry(void *o);
static enum smf_state_result defaults_run(void *o);
static void awake_entry(void *o);
static enum smf_state_result awake_run(void *o);
static void sleep_entry(void *o);
static enum smf_state_result sleep_run(void *o);
static void error_entry(void *o);
static enum smf_state_result error_run(void *o);
static void error_exit(void *o);

static int init_app_object(struct app_object *s)
{
    if (s == NULL) {
        return -1;
    }

    s->action_freq_hz = LED_BLINK_FREQ_HZ;
    s->stored_action_freq_hz = LED_BLINK_FREQ_HZ;
    s->action_phase = false;
    s->stored_action_phase = false;
    s->stored_action_remaining_ms = 0;
    s->hb_last_ns = 0;
    s->action_last_ns = 0;
    s->wake_from_sleep = false;
    s->threads_started = false;

    return 0;
}

static void set_action_outputs_from_phase(bool phase)
{
    gpio_pin_set_dt(&iv_pump_led, phase ? 0 : 1);
    gpio_pin_set_dt(&buzzer_led, phase ? 1 : 0);
}

static void clear_action_outputs(void)
{
    gpio_pin_set_dt(&iv_pump_led, 0);
    gpio_pin_set_dt(&buzzer_led, 0);
}

static void enable_awake_buttons(void)
{
    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_EDGE_TO_ACTIVE);
}

static void disable_awake_buttons(void)
{
    gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_DISABLE);
    gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_DISABLE);
}

static void start_action_timer_for_freq(struct app_object *s)
{
    int32_t hp = action_half_period_ms(s->action_freq_hz);
    k_timer_stop(&action_timer);
    k_timer_start(&action_timer, K_MSEC(hp), K_MSEC(hp));
    s->action_last_ns = 0;
}

static void restore_action_timer_after_sleep(struct app_object *s)
{
    int32_t hp = action_half_period_ms(s->action_freq_hz);
    int32_t first_ms = s->stored_action_remaining_ms;

    if (first_ms <= 0 || first_ms > hp) {
        first_ms = hp;
    }

    set_action_outputs_from_phase(s->action_phase);
    k_timer_start(&action_timer, K_MSEC(first_ms), K_MSEC(hp));
    s->action_last_ns = 0;

    LOG_INF("Exited SLEEP (freq=%d, phase=%d, first=%d ms, hp=%d ms) @ %llu ns",
            s->action_freq_hz, s->action_phase, first_ms, hp, now_ns());
}

static enum smf_state_result init_run(void *o){
    struct app_object *s = (struct app_object *)o;
    int err;

    if (!device_is_ready(sleep_button.port)) {
        LOG_ERR("gpio0 interface not ready.");
        smf_set_terminate(SMF_CTX(s), -1);
        return SMF_EVENT_HANDLED;;
    }

    err = gpio_pin_configure_dt(&sleep_button, GPIO_INPUT);
    if (err < 0) { LOG_ERR("Cannot configure sleep button."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_configure_dt(&freq_up_button, GPIO_INPUT);
    if (err < 0) { LOG_ERR("Cannot configure freq_up button."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_configure_dt(&freq_down_button, GPIO_INPUT);
    if (err < 0) { LOG_ERR("Cannot configure freq_down button."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_configure_dt(&reset_button, GPIO_INPUT);
    if (err < 0) { LOG_ERR("Cannot configure reset button."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_configure_dt(&heartbeat_led, GPIO_OUTPUT_INACTIVE);
    if (err < 0) { LOG_ERR("Cannot configure heartbeat LED."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_configure_dt(&iv_pump_led, GPIO_OUTPUT_INACTIVE);
    if (err < 0) { LOG_ERR("Cannot configure iv_pump LED."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_configure_dt(&buzzer_led, GPIO_OUTPUT_INACTIVE);
    if (err < 0) { LOG_ERR("Cannot configure buzzer LED."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_configure_dt(&error_led, GPIO_OUTPUT_INACTIVE);
    if (err < 0) { LOG_ERR("Cannot configure error LED."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_interrupt_configure_dt(&sleep_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) { LOG_ERR("Cannot attach callback to sw0."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_interrupt_configure_dt(&freq_up_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) { LOG_ERR("Cannot attach callback to sw1."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_interrupt_configure_dt(&freq_down_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) { LOG_ERR("Cannot attach callback to sw2."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    err = gpio_pin_interrupt_configure_dt(&reset_button, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) { LOG_ERR("Cannot attach callback to sw3."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    gpio_init_callback(&sleep_button_cb, sleep_button_callback, BIT(sleep_button.pin));
    err = gpio_add_callback_dt(&sleep_button, &sleep_button_cb);
    if (err < 0) { LOG_ERR("Cannot add sleep button callback."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    gpio_init_callback(&reset_button_cb, reset_button_callback, BIT(reset_button.pin));
    err = gpio_add_callback_dt(&reset_button, &reset_button_cb);
    if (err < 0) { LOG_ERR("Cannot add reset button callback."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    gpio_init_callback(&freq_up_button_cb, freq_up_button_callback, BIT(freq_up_button.pin));
    err = gpio_add_callback_dt(&freq_up_button, &freq_up_button_cb);
    if (err < 0) { LOG_ERR("Cannot add freq_up button callback."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    gpio_init_callback(&freq_down_button_cb, freq_down_button_callback, BIT(freq_down_button.pin));
    err = gpio_add_callback_dt(&freq_down_button, &freq_down_button_cb);
    if (err < 0) { LOG_ERR("Cannot add freq_down button callback."); smf_set_terminate(SMF_CTX(s), err); return SMF_EVENT_HANDLED; }

    k_event_clear(&button_events, BTN_EVENT_MASK);
    k_event_clear(&raw_freq_events, RAW_FREQ_MASK);

    k_timer_user_data_set(&action_timer, s);

    if (!s->threads_started) {
        k_thread_name_set(heartbeat_thread_id, "heartbeat");
        k_thread_name_set(doublepress_thread_id, "doublepress");
        k_thread_start(heartbeat_thread_id);
        k_thread_start(doublepress_thread_id);
        s->threads_started = true;
    }

    smf_set_state(SMF_CTX(s), &app_states[STATE_DEFAULTS]);
    return SMF_EVENT_HANDLED;
}

static void defaults_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;

    s->action_freq_hz = LED_BLINK_FREQ_HZ;
    s->action_phase = false;
    s->stored_action_freq_hz = s->action_freq_hz;
    s->stored_action_phase = s->action_phase;
    s->stored_action_remaining_ms = 0;
    s->wake_from_sleep = false;

    gpio_pin_set_dt(&error_led, 0);
    enable_awake_buttons();
    set_action_outputs_from_phase(s->action_phase);
    start_action_timer_for_freq(s);

    LOG_INF("DEFAULTS: action_freq=%d Hz, phase=%d @ %llu ns",
            s->action_freq_hz, s->action_phase, now_ns());
}

static enum smf_state_result defaults_run(void *o)
{
    struct app_object *s = (struct app_object *)o;
    smf_set_state(SMF_CTX(s), &app_states[STATE_AWAKE]);
    return SMF_EVENT_HANDLED;
}

static void awake_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;

    gpio_pin_set_dt(&error_led, 0);
    enable_awake_buttons();

    if (s->wake_from_sleep) {
        s->wake_from_sleep = false;
        s->action_freq_hz = s->stored_action_freq_hz;
        s->action_phase = s->stored_action_phase;
        restore_action_timer_after_sleep(s);
    }
}

static enum smf_state_result awake_run(void *o)
{
    struct app_object *s = (struct app_object *)o;
    uint32_t events = k_event_wait(&button_events, BTN_EVENT_MASK, true, K_FOREVER);
    int delta = 0;

    LOG_INF("Button Event Posted: %u", events);

    if (events & BTN_RESET_EVENT) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_DEFAULTS]);
        return SMF_EVENT_HANDLED;;
    }

    if (events & BTN_SLEEP_EVENT) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_SLEEP]);
        return SMF_EVENT_HANDLED;;
    }

    if (events & BTN_FREQ_UP_DBL_EVENT)   { delta += (2 * FREQ_UP_INC_HZ); }
    if (events & BTN_FREQ_DOWN_DBL_EVENT) { delta -= (2 * FREQ_DOWN_INC_HZ); }
    if (events & BTN_FREQ_UP_EVENT)       { delta += FREQ_UP_INC_HZ; }
    if (events & BTN_FREQ_DOWN_EVENT)     { delta -= FREQ_DOWN_INC_HZ; }

    if (delta != 0) {
        s->action_freq_hz += delta;
        LOG_INF("FREQ delta %d -> %d Hz @ %llu ns", delta, s->action_freq_hz, now_ns());
    }

    if (s->action_freq_hz < ACTION_FREQ_MIN_HZ || s->action_freq_hz > ACTION_FREQ_MAX_HZ) {
        smf_set_state(SMF_CTX(s), &app_states[STATE_ERROR]);
        return SMF_EVENT_HANDLED;;
    }

    if (delta != 0) {
        start_action_timer_for_freq(s);
    }
    return SMF_EVENT_HANDLED;
}

static void sleep_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;

    s->stored_action_freq_hz = s->action_freq_hz;
    s->stored_action_phase = s->action_phase;
    s->stored_action_remaining_ms = k_timer_remaining_get(&action_timer);
    s->wake_from_sleep = false;

    k_timer_stop(&action_timer);
    clear_action_outputs();

    LOG_INF("Entered SLEEP (stored freq=%d, phase=%d, rem=%d ms) @ %llu ns",
            s->stored_action_freq_hz, s->stored_action_phase,
            s->stored_action_remaining_ms, now_ns());
}

static enum smf_state_result sleep_run(void *o)
{
    struct app_object *s = (struct app_object *)o;
    uint32_t events = k_event_wait(&button_events,
                                   BTN_SLEEP_EVENT | BTN_RESET_EVENT,
                                   true,
                                   K_FOREVER);

    LOG_INF("Button Event Posted: %u", events);

    if (events & BTN_RESET_EVENT) {
        s->wake_from_sleep = false;
        smf_set_state(SMF_CTX(s), &app_states[STATE_DEFAULTS]);
        return SMF_EVENT_HANDLED;;
    }

    s->wake_from_sleep = true;
    smf_set_state(SMF_CTX(s), &app_states[STATE_AWAKE]);
    return SMF_EVENT_HANDLED;
}

static void error_entry(void *o)
{
    struct app_object *s = (struct app_object *)o;

    k_timer_stop(&action_timer);
    clear_action_outputs();
    gpio_pin_set_dt(&error_led, 1);
    disable_awake_buttons();

    LOG_ERR("ERROR: action_freq out of range (%d Hz) @ %llu ns",
            s->action_freq_hz, now_ns());
}

static enum smf_state_result error_run(void *o)
{
    struct app_object *s = (struct app_object *)o;
    uint32_t events = k_event_wait(&button_events, BTN_RESET_EVENT, true, K_FOREVER);

    LOG_INF("Button Event Posted: %u", events);
    smf_set_state(SMF_CTX(s), &app_states[STATE_DEFAULTS]);
    return SMF_EVENT_HANDLED;
}

static void error_exit(void *o)
{
    ARG_UNUSED(o);
    gpio_pin_set_dt(&error_led, 0);
}

static const struct smf_state app_states[] = {
    [STATE_INIT]     = SMF_CREATE_STATE(NULL,           init_run,     NULL, NULL, NULL),
    [STATE_DEFAULTS] = SMF_CREATE_STATE(defaults_entry, defaults_run, NULL, NULL, NULL),
    [STATE_AWAKE]    = SMF_CREATE_STATE(awake_entry,    awake_run,    NULL, NULL, NULL),
    [STATE_SLEEP]    = SMF_CREATE_STATE(sleep_entry,    sleep_run,    NULL, NULL, NULL),
    [STATE_ERROR]    = SMF_CREATE_STATE(error_entry,    error_run,    error_exit, NULL, NULL),
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

/* timer handlers */

void action_timer_handler(struct k_timer *t)
{
    struct app_object *s = (struct app_object *)k_timer_user_data_get(t);
    uint64_t now = now_ns();

    if (s == NULL) {
        return;
    }

    if (s->action_last_ns != 0U) {
        LOG_INF("action toggle period (ns): %llu",
                (unsigned long long)(now - s->action_last_ns));
    }
    s->action_last_ns = now;

    s->action_phase = !s->action_phase;
    set_action_outputs_from_phase(s->action_phase);
}

void action_timer_stop(struct k_timer *t)
{
    ARG_UNUSED(t);
    clear_action_outputs();
}

/* GPIO callbacks */

void sleep_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&button_events, BTN_SLEEP_EVENT);
}

void reset_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&button_events, BTN_RESET_EVENT);
}

void freq_up_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&raw_freq_events, RAW_FREQ_UP_EVENT);
}

void freq_down_button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_event_post(&raw_freq_events, RAW_FREQ_DOWN_EVENT);
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

void doublepress_thread(void *, void *, void *)
{
    uint32_t pending = 0;

    while (1) {
        uint32_t first = pending;
        pending = 0;

        if (first == 0) {
            first = k_event_wait(&raw_freq_events, RAW_FREQ_MASK, true, K_FOREVER);

            /* If both events arrived together, treat DOWN as pending */
            if ((first & RAW_FREQ_UP_EVENT) && (first & RAW_FREQ_DOWN_EVENT)) {
                pending = RAW_FREQ_DOWN_EVENT;
                first = RAW_FREQ_UP_EVENT;
            }
            else if (first & RAW_FREQ_UP_EVENT) {
                first = RAW_FREQ_UP_EVENT;
            }
            else if (first & RAW_FREQ_DOWN_EVENT) {
                first = RAW_FREQ_DOWN_EVENT;
            }
            else {
                continue;
            }
        }

        uint32_t second = k_event_wait(&raw_freq_events,
                                       RAW_FREQ_MASK,
                                       true,
                                       K_MSEC(DOUBLE_PRESS_WINDOW_MS));

        if (second == 0) {
            if (first == RAW_FREQ_UP_EVENT) {
                k_event_post(&button_events, BTN_FREQ_UP_EVENT);
            } else {
                k_event_post(&button_events, BTN_FREQ_DOWN_EVENT);
            }
            continue;
        }

        if (second & RAW_FREQ_UP_EVENT) {
            second = RAW_FREQ_UP_EVENT;
        } else if (second & RAW_FREQ_DOWN_EVENT) {
            second = RAW_FREQ_DOWN_EVENT;
        } else {
            continue;
        }

        if (second == first) {
            if (first == RAW_FREQ_UP_EVENT) {
                k_event_post(&button_events, BTN_FREQ_UP_DBL_EVENT);
            } else {
                k_event_post(&button_events, BTN_FREQ_DOWN_DBL_EVENT);
            }
        } else {
            if (first == RAW_FREQ_UP_EVENT) {
                k_event_post(&button_events, BTN_FREQ_UP_EVENT);
            } else {
                k_event_post(&button_events, BTN_FREQ_DOWN_EVENT);
            }
            pending = second;
        }
    }
}