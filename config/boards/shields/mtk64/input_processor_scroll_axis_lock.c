/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * スクロール時の軸ロック。takashicompany 版の
 *   if (abs(y) * 2 > abs(x)) 縦スクロールのみ, else 横スクロールのみ
 * を再現する。xy_to_scroll_mapper より前に置くこと。
 *
 * センサーは 1 回の読み取りを REL_X (sync=false) → REL_Y (sync=true) の順で報告するので、
 * X を保留して 0 にし、Y の時点でどちらの軸を通すか決める。横が勝った場合は
 * Y イベントを X イベントに書き換えて X の値を載せる。
 */

#define DT_DRV_COMPAT zmk_input_processor_scroll_axis_lock

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

struct scroll_axis_lock_data {
    int32_t pending_x;
    bool has_x;
};

static int scroll_axis_lock_handle_event(const struct device *dev, struct input_event *event,
                                         uint32_t param1, uint32_t param2,
                                         struct zmk_input_processor_state *state) {
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);
    ARG_UNUSED(state);

    struct scroll_axis_lock_data *data = dev->data;

    if (event->type != INPUT_EV_REL) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_X) {
        if (!event->sync) {
            data->pending_x = event->value;
            data->has_x = true;
            event->value = 0;
        }
        // X 単独 (sync=true) の報告はそのまま横スクロール
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (event->code == INPUT_REL_Y) {
        int32_t x = data->has_x ? data->pending_x : 0;
        data->pending_x = 0;
        data->has_x = false;

        if (abs(event->value) * 2 <= abs(x)) {
            event->code = INPUT_REL_X;
            event->value = x;
        }
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static int scroll_axis_lock_init(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

static const struct zmk_input_processor_driver_api scroll_axis_lock_driver_api = {
    .handle_event = scroll_axis_lock_handle_event,
};

#define SCROLL_AXIS_LOCK_INST(n)                                                                   \
    static struct scroll_axis_lock_data scroll_axis_lock_data_##n = {};                            \
    DEVICE_DT_INST_DEFINE(n, scroll_axis_lock_init, NULL, &scroll_axis_lock_data_##n, NULL,        \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                        \
                          &scroll_axis_lock_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SCROLL_AXIS_LOCK_INST)
