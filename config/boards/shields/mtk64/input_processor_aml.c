/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * Auto Mouse Layer (AML) input processor.
 *
 * takashicompany 氏の miniZone / Keyball 向けファームウェア
 * (https://zenn.dev/takashicompany/articles/69b87160cda4b9) の
 * 「自動マウスレイヤー」の状態遷移を ZMK 向けに移植したもの。
 *
 *   NONE ──(ボール移動)──▶ WAITING ──(移動量 >= activation-movement)──▶ CLICKABLE
 *     ▲                       │ (activation-window-ms 経過後に動きが途切れたらリセット)
 *     │                       ▼
 *     └──────────────────── NONE
 *
 *   CLICKABLE: ボール操作が timeout(param2) ms 無ければレイヤー解除
 *              修飾キー以外のキーを押すとレイヤー解除
 *   CLICKING : マウスボタン押下中。タイムアウトせず、キー入力でも解除しない。
 *              押下直後 click-lock-movement カウント分のボール移動を無視する。
 *   SCROLLING: scroll-layer が有効な間。タイムアウトせず、キー入力でも解除しない。
 *              スクロールキーを離すと CLICKABLE に戻りタイマーを再始動する。
 *
 * input-processors = <&zip_aml LAYER TIMEOUT_MS>;
 *   - トラックボールのリスナー: REL_X / REL_Y を監視
 *   - &mkp_input_listener      : マウスボタン (EV_KEY) を監視
 */

#define DT_DRV_COMPAT zmk_input_processor_aml

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#include "aml_state_changed.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

ZMK_EVENT_IMPL(zmk_aml_state_changed);

#define AML_THRESHOLD_MIN 5
#define AML_THRESHOLD_MAX 2000

static bool g_aml_enabled = true; // デフォルトは有効
static int16_t g_aml_threshold = -1; // -1: DTS の activation-movement を使う

struct aml_config {
    int16_t require_prior_idle_ms;
    int16_t activation_movement;
    int16_t activation_window_ms;
    int16_t activation_gap_ms;
    int16_t click_lock_movement;
    int16_t scroll_layer;
    const uint16_t *excluded_positions;
    size_t num_positions;
};

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

#include <zmk/keymap.h>
#include <zmk/keys.h>
#include <zmk/behavior.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/layer_state_changed.h>

#define MAX_LAYERS ZMK_KEYMAP_LAYERS_LEN

#ifndef CONFIG_ZMK_INPUT_PROCESSOR_AML_MAX_ACTION_EVENTS
#define CONFIG_ZMK_INPUT_PROCESSOR_AML_MAX_ACTION_EVENTS 4
#endif

struct aml_state {
    uint8_t toggle_layer;
    uint32_t timeout_ms;
    bool is_active;           // AML レイヤーが有効 (CLICKABLE / CLICKING / SCROLLING)
    bool waiting;             // WAITING 状態
    bool scrolling;           // 直近に観測した scroll-layer の状態
    uint8_t held_buttons;     // 押下中のマウスボタン数 (>0 なら CLICKING)
    int32_t movement;         // WAITING 中の累積移動量
    int32_t click_lock;       // クリック直後に無視する残り移動量
    int64_t waiting_start;
    int64_t last_motion;
    int64_t last_tapped_timestamp;
};

struct aml_data {
    const struct device *dev;
    struct k_mutex lock;
    struct aml_state state;
};

static struct k_work_delayable layer_disable_work;

static int16_t current_threshold(const struct aml_config *cfg) {
    return g_aml_threshold > 0 ? g_aml_threshold : cfg->activation_movement;
}

static bool position_is_excluded(const struct aml_config *config, uint32_t position) {
    if (!config->excluded_positions || !config->num_positions) {
        return false;
    }

    const uint16_t *end = config->excluded_positions + config->num_positions;
    for (const uint16_t *pos = config->excluded_positions; pos < end; pos++) {
        if (*pos == position) {
            return true;
        }
    }

    return false;
}

static bool typed_recently(const struct aml_config *config, int64_t last_tapped,
                           int64_t current_time) {
    return config->require_prior_idle_ms > 0 &&
           (last_tapped + config->require_prior_idle_ms) > current_time;
}

static bool is_scrolling(const struct aml_config *cfg) {
    return cfg->scroll_layer >= 0 && zmk_keymap_layer_active(cfg->scroll_layer);
}

/* Layer State Management (lock を保持した状態で呼ぶこと) */
static void update_layer_state(struct aml_state *state, bool activate) {
    if (state->is_active == activate) {
        return;
    }

    state->is_active = activate;
    if (activate) {
        state->waiting = false;
        state->movement = 0;
        zmk_keymap_layer_activate(state->toggle_layer, false);
        LOG_DBG("AML Layer %d activated", state->toggle_layer);
    } else {
        state->held_buttons = 0;
        state->click_lock = 0;
        k_work_cancel_delayable(&layer_disable_work);
        zmk_keymap_layer_deactivate(state->toggle_layer, false);
        LOG_DBG("AML Layer %d deactivated", state->toggle_layer);
    }
}

/* CLICKABLE のタイマーを (再) 始動する。takashicompany 版の click_timer = timer_read() 相当 */
static void restart_timeout(struct aml_state *state) {
    if (state->timeout_ms > 0) {
        k_work_reschedule(&layer_disable_work, K_MSEC(state->timeout_ms));
    }
}

struct layer_state_action {
    bool activate;
};

K_MSGQ_DEFINE(aml_action_msgq, sizeof(struct layer_state_action),
              CONFIG_ZMK_INPUT_PROCESSOR_AML_MAX_ACTION_EVENTS, 4);

static void layer_action_work_cb(struct k_work *work) {
    const struct device *dev = DEVICE_DT_INST_GET(0);
    struct aml_data *data = (struct aml_data *)dev->data;
    const struct aml_config *cfg = dev->config;

    if (k_mutex_lock(&data->lock, K_FOREVER) < 0) {
        return;
    }

    struct layer_state_action action;
    while (k_msgq_get(&aml_action_msgq, &action, K_NO_WAIT) == 0) {
        if (!action.activate) {
            // クリック中・スクロール中はタイムアウトしない
            if (data->state.held_buttons == 0 && !is_scrolling(cfg)) {
                update_layer_state(&data->state, false);
            }
        } else if (g_aml_enabled) {
            update_layer_state(&data->state, true);
            restart_timeout(&data->state);
        }
    }

    k_mutex_unlock(&data->lock);
}

static K_WORK_DEFINE(layer_action_work, layer_action_work_cb);

static void enqueue_action(bool activate) {
    struct layer_state_action action = {.activate = activate};
    if (k_msgq_put(&aml_action_msgq, &action, K_NO_WAIT) == 0) {
        k_work_submit(&layer_action_work);
    } else {
        LOG_ERR("Failed to enqueue AML action (%d)", activate);
    }
}

static void layer_disable_callback(struct k_work *work) { enqueue_action(false); }

/* Event Handlers */
static int handle_layer_state_changed(const struct device *dev, const zmk_event_t *eh) {
    struct aml_data *data = (struct aml_data *)dev->data;
    const struct aml_config *cfg = dev->config;

    if (k_mutex_lock(&data->lock, K_FOREVER) < 0) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (data->state.is_active && !zmk_keymap_layer_active(data->state.toggle_layer)) {
        // 他の要因 (&tog 等) で AML レイヤーが落とされた
        data->state.is_active = false;
        data->state.held_buttons = 0;
        k_work_cancel_delayable(&layer_disable_work);
    }

    bool scrolling = is_scrolling(cfg);
    if (scrolling != data->state.scrolling) {
        data->state.scrolling = scrolling;
        if (data->state.is_active) {
            if (scrolling) {
                // SCROLLING: タイムアウト停止
                k_work_cancel_delayable(&layer_disable_work);
            } else if (data->state.held_buttons == 0) {
                // スクロールキーを離したら CLICKABLE に戻ってタイマー再始動
                restart_timeout(&data->state);
            }
        }
    }

    k_mutex_unlock(&data->lock);
    return ZMK_EV_EVENT_BUBBLE;
}

static int handle_position_state_changed(const struct device *dev, const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (!ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct aml_data *data = (struct aml_data *)dev->data;
    const struct aml_config *cfg = dev->config;

    if (k_mutex_lock(&data->lock, K_FOREVER) < 0) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (data->state.is_active && data->state.held_buttons == 0 && !is_scrolling(cfg) &&
        !position_is_excluded(cfg, ev->position)) {
        update_layer_state(&data->state, false);
    }

    k_mutex_unlock(&data->lock);
    return ZMK_EV_EVENT_BUBBLE;
}

static int handle_keycode_state_changed(const struct device *dev, const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (!ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct aml_data *data = (struct aml_data *)dev->data;
    const struct aml_config *cfg = dev->config;

    if (k_mutex_lock(&data->lock, K_FOREVER) < 0) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    data->state.last_tapped_timestamp = ev->timestamp;

    // 修飾キー (Ctrl/Shift/Alt/GUI) は解除しない (ignore_disable_mouse_layer_keys 相当)
    // クリック中・スクロール中も解除しない
    if (data->state.is_active && !is_mod(ev->usage_page, ev->keycode) &&
        data->state.held_buttons == 0 && !is_scrolling(cfg)) {
        update_layer_state(&data->state, false);
    }

    k_mutex_unlock(&data->lock);
    return ZMK_EV_EVENT_BUBBLE;
}

static int handle_state_changed_dispatcher(const struct device *dev, const zmk_event_t *eh) {
    if (as_zmk_layer_state_changed(eh) != NULL) {
        return handle_layer_state_changed(dev, eh);
    } else if (as_zmk_position_state_changed(eh) != NULL) {
        return handle_position_state_changed(dev, eh);
    } else if (as_zmk_keycode_state_changed(eh) != NULL) {
        return handle_keycode_state_changed(dev, eh);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

#define DISPATCH_EVENT(inst)                                                                       \
    {                                                                                              \
        int err = handle_state_changed_dispatcher(DEVICE_DT_INST_GET(inst), eh);                   \
        if (err < 0) {                                                                             \
            return err;                                                                            \
        }                                                                                          \
    }

static int handle_event_dispatcher(const zmk_event_t *eh) {
    DT_INST_FOREACH_STATUS_OKAY(DISPATCH_EVENT)
    return 0;
}

/* マウスボタン (&mkp) */
static void handle_button(const struct aml_config *cfg, struct aml_state *st,
                          struct input_event *event) {
    if (!st->is_active) {
        // AML レイヤー以外 (レイヤー1/2 等) のクリックは関与しない
        return;
    }

    if (event->value) {
        if (st->held_buttons < UINT8_MAX) {
            st->held_buttons++;
        }
        st->click_lock = cfg->click_lock_movement;
        k_work_cancel_delayable(&layer_disable_work);
    } else {
        if (st->held_buttons > 0) {
            st->held_buttons--;
        }
        if (st->held_buttons == 0 && !is_scrolling(cfg)) {
            restart_timeout(st);
        }
    }
}

/* ボール移動 (REL_X / REL_Y) */
static void handle_motion(const struct aml_config *cfg, struct aml_state *st,
                          struct input_event *event) {
    int32_t amount = abs(event->value);
    if (amount == 0) {
        return;
    }

    int64_t now = k_uptime_get();

    if (st->is_active) {
        if (st->held_buttons > 0) {
            // CLICKING: クリック直後の微小な動きを無視する
            if (st->click_lock > 0) {
                st->click_lock -= amount;
                if (st->click_lock > 0) {
                    event->value = 0;
                }
            }
        } else if (!is_scrolling(cfg)) {
            // CLICKABLE: 動いている限りタイマー延長
            restart_timeout(st);
        }
        st->last_motion = now;
        return;
    }

    if (typed_recently(cfg, st->last_tapped_timestamp, now)) {
        st->waiting = false;
        st->movement = 0;
        st->last_motion = now;
        return;
    }

    // WAITING: window を過ぎてから動きが途切れていたらリセット
    if (st->waiting && (now - st->waiting_start) > cfg->activation_window_ms &&
        (now - st->last_motion) > cfg->activation_gap_ms) {
        st->waiting = false;
    }

    if (!st->waiting) {
        st->waiting = true;
        st->waiting_start = now;
        st->movement = 0;
    }

    st->last_motion = now;
    st->movement += amount;

    if (st->movement >= current_threshold(cfg)) {
        st->movement = 0;
        st->waiting = false;
        enqueue_action(true);
    }
}

/* Driver Implementation */
static int aml_handle_event(const struct device *dev, struct input_event *event,
                            uint32_t param1, uint32_t param2,
                            struct zmk_input_processor_state *state) {
    if (param1 >= MAX_LAYERS) {
        LOG_ERR("Invalid layer index: %d", param1);
        return -EINVAL;
    }

    // AML が無効の場合は何もせず素通し
    if (!g_aml_enabled) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    struct aml_data *data = (struct aml_data *)dev->data;
    const struct aml_config *cfg = dev->config;

    if (k_mutex_lock(&data->lock, K_FOREVER) < 0) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    data->state.toggle_layer = param1;
    data->state.timeout_ms = param2;

    if (event->type == INPUT_EV_KEY) {
        handle_button(cfg, &data->state, event);
    } else if (event->type == INPUT_EV_REL &&
               (event->code == INPUT_REL_X || event->code == INPUT_REL_Y)) {
        handle_motion(cfg, &data->state, event);
    }
    // WHEEL / HWHEEL (スクロールレイヤーで変換済み) は無視

    k_mutex_unlock(&data->lock);

    return ZMK_INPUT_PROC_CONTINUE;
}

static int aml_init(const struct device *dev) {
    struct aml_data *data = (struct aml_data *)dev->data;
    data->dev = dev;
    k_mutex_init(&data->lock);
    k_work_init_delayable(&layer_disable_work, layer_disable_callback);
    return 0;
}

static const struct zmk_input_processor_driver_api aml_driver_api = {
    .handle_event = aml_handle_event,
};

#define NEEDS_POSITION_HANDLERS(n, ...) DT_INST_PROP_HAS_IDX(n, excluded_positions, 0)

ZMK_LISTENER(processor_aml, handle_event_dispatcher);
ZMK_SUBSCRIPTION(processor_aml, zmk_layer_state_changed);
ZMK_SUBSCRIPTION(processor_aml, zmk_keycode_state_changed);

#if DT_INST_FOREACH_STATUS_OKAY_VARGS(NEEDS_POSITION_HANDLERS, ||)
ZMK_SUBSCRIPTION(processor_aml, zmk_position_state_changed);
#endif

#else /* Peripheral side without Central role */

struct aml_data {
    const struct device *dev;
};

static int aml_handle_event(const struct device *dev, struct input_event *event,
                            uint32_t param1, uint32_t param2,
                            struct zmk_input_processor_state *state) {
    ARG_UNUSED(dev);
    ARG_UNUSED(event);
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);
    return ZMK_INPUT_PROC_CONTINUE;
}

static int aml_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

static const struct zmk_input_processor_driver_api aml_driver_api = {
    .handle_event = aml_handle_event,
};

#endif /* !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) */

/* Device Instantiation (Central / Peripheral 共通) */
#define AML_INST(n)                                                                                \
    static struct aml_data processor_aml_data_##n = {};                                            \
    static const uint16_t excluded_positions_##n[] = DT_INST_PROP(n, excluded_positions);         \
    static const struct aml_config processor_aml_config_##n = {                                    \
        .require_prior_idle_ms = DT_INST_PROP_OR(n, require_prior_idle_ms, 0),                    \
        .activation_movement = DT_INST_PROP_OR(n, activation_movement, 1),                        \
        .activation_window_ms = DT_INST_PROP_OR(n, activation_window_ms, 50),                     \
        .activation_gap_ms = DT_INST_PROP_OR(n, activation_gap_ms, 10),                           \
        .click_lock_movement = DT_INST_PROP_OR(n, click_lock_movement, 0),                        \
        .scroll_layer = DT_INST_PROP_OR(n, scroll_layer, -1),                                     \
        .excluded_positions = excluded_positions_##n,                                             \
        .num_positions = DT_INST_PROP_LEN(n, excluded_positions),                                 \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, aml_init, NULL, &processor_aml_data_##n,                             \
                          &processor_aml_config_##n, POST_KERNEL,                                  \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &aml_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AML_INST)

/* Public AML Control APIs */
bool zmk_aml_is_enabled(void) { return g_aml_enabled; }

void zmk_aml_set_enabled(bool enabled) {
    if (g_aml_enabled == enabled) {
        return;
    }
    g_aml_enabled = enabled;
    LOG_INF("AML state changed to %s", g_aml_enabled ? "ENABLED" : "DISABLED");

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    if (!g_aml_enabled) {
        const struct device *dev = DEVICE_DT_INST_GET(0);
        struct aml_data *data = (struct aml_data *)dev->data;
        k_mutex_lock(&data->lock, K_FOREVER);
        data->state.waiting = false;
        data->state.movement = 0;
        if (data->state.is_active) {
            update_layer_state(&data->state, false);
        }
        k_mutex_unlock(&data->lock);
    }

#if IS_ENABLED(CONFIG_SETTINGS)
    uint8_t val = g_aml_enabled ? 1 : 0;
    settings_save_one("aml/enabled", &val, sizeof(val));
#endif
#endif

    raise_zmk_aml_state_changed((struct zmk_aml_state_changed){.enabled = g_aml_enabled});
}

void zmk_aml_toggle(void) { zmk_aml_set_enabled(!g_aml_enabled); }

int16_t zmk_aml_get_threshold(void) {
    if (g_aml_threshold > 0) {
        return g_aml_threshold;
    }
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
    return DT_INST_PROP_OR(0, activation_movement, 1);
#else
    return 1;
#endif
}

/* KC_TO_CLICKABLE_INC / DEC 相当 */
void zmk_aml_adjust_threshold(int16_t delta) {
    int32_t next = (int32_t)zmk_aml_get_threshold() + delta;
    if (next < AML_THRESHOLD_MIN) {
        next = AML_THRESHOLD_MIN;
    } else if (next > AML_THRESHOLD_MAX) {
        next = AML_THRESHOLD_MAX;
    }
    g_aml_threshold = (int16_t)next;
    LOG_INF("AML activation movement: %d", g_aml_threshold);

#if (!IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)) &&                \
    IS_ENABLED(CONFIG_SETTINGS)
    settings_save_one("aml/thr", &g_aml_threshold, sizeof(g_aml_threshold));
#endif
}

#if (!IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)) &&                \
    IS_ENABLED(CONFIG_SETTINGS)
static int aml_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                            void *cb_arg) {
    const char *next;
    if (settings_name_steq(name, "enabled", &next) && !next) {
        if (len == sizeof(uint8_t)) {
            uint8_t val;
            if (read_cb(cb_arg, &val, sizeof(val)) >= 0) {
                g_aml_enabled = (val != 0);
                LOG_INF("Loaded saved AML state: %s", g_aml_enabled ? "ENABLED" : "DISABLED");
                raise_zmk_aml_state_changed(
                    (struct zmk_aml_state_changed){.enabled = g_aml_enabled});
            }
        }
        return 0;
    }
    if (settings_name_steq(name, "thr", &next) && !next) {
        if (len == sizeof(int16_t)) {
            int16_t val;
            if (read_cb(cb_arg, &val, sizeof(val)) >= 0 && val >= AML_THRESHOLD_MIN &&
                val <= AML_THRESHOLD_MAX) {
                g_aml_threshold = val;
                LOG_INF("Loaded saved AML threshold: %d", g_aml_threshold);
            }
        }
        return 0;
    }
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(aml, "aml", NULL, aml_settings_set, NULL, NULL);
#endif
