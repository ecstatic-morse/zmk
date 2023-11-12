/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_kscan_ptty
#define MAX_CMD_LEN 128

#include <stdlib.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

const struct device *command_uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

static void kscan_ptty_work_handler(struct k_work *work);

struct kscan_ptty_config {
    int delay_ms;
    bool exit_after;
};

struct kscan_ptty_data {
    kscan_callback_t callback;
    struct k_work_delayable work;
    int cmd_idx;
};

static const struct kscan_ptty_config kscan_ptty_config = {
    .delay_ms = DT_INST_PROP(0, event_period),
    .exit_after = DT_INST_PROP(0, exit_after),
};

static struct kscan_ptty_data kscan_ptty_data;

static void kscan_ptty_init(const struct device *dev) {
    k_work_init_delayable(&kscan_ptty_data.work, kscan_ptty_work_handler);
}

static int kscan_ptty_enable_callback(const struct device *dev) {
    k_work_schedule(&kscan_ptty_data.work, K_MSEC(kscan_ptty_config.delay_ms));
    return 0;
}

static int kscan_ptty_disable_callback(const struct device *dev) {
    k_work_cancel_delayable(&kscan_ptty_data.work);
    return 0;
}

static int kscan_ptty_configure(const struct device *dev, kscan_callback_t callback) {
    if (!callback)
        return -EINVAL;

    kscan_ptty_data.callback = callback;
    k_work_schedule(&kscan_ptty_data.work, K_MSEC(kscan_ptty_config.delay_ms));
    return 0;
}

static const struct kscan_driver_api ptty_driver_api = {
    .config = kscan_ptty_configure,
    .enable_callback = kscan_ptty_enable_callback,
    .disable_callback = kscan_ptty_disable_callback,
};

DEVICE_DT_INST_DEFINE(0, kscan_ptty_init, NULL, &kscan_ptty_data, &kscan_ptty_config, POST_KERNEL,
                      CONFIG_KSCAN_INIT_PRIORITY, &ptty_driver_api);

int recv_line(const struct device *uart, char *buf) {
    char *out = buf;
    char c = '\0';

    // Even when stdin has no more data, the POSIX TTY driver seems to always
    // return successfully from `poll_in`. Instead, it outputting NUL for
    // every read. Make sure we handle this case as well.
    while (!uart_poll_in(uart, &c)) {
        if (out - buf >= MAX_CMD_LEN - 1)
            return -EOVERFLOW;
        if (c == '\n' || c == '\0')
            break;
        *out++ = c;
    }

    if (out == buf && c == '\0')
        return -ENODATA;

    *out = '\0';
    return 0;
}

// Parses a command, reads its arguments into `args` and validates them.
//
// If the command is valid, returns the character code of the command.
// Otherwise, returns a negative error code.
static int kscan_parse_command(const char cmd[], int args[2]) {
    int n = 0;
    switch (cmd[0]) {
    case 'p':
        n = sscanf(cmd, "p %d %d", &args[0], &args[1]);
        break;
    case 'r':
        n = sscanf(cmd, "r %d %d", &args[0], &args[1]);
        break;
    case 'w':
        n = sscanf(cmd, "w %d", &args[0]);
        break;
    default:
        return -EINVAL;
    }

    switch (cmd[0]) {
    case 'p':
    case 'r':
        if (n == 1)
            args[1] = 0;
        else if (n != 2)
            return -EINVAL;
        break;
    case 'w':
        if (n != 1)
            return -EINVAL;
        break;
    }

    return cmd[0];
}

static void kscan_ptty_work_handler(struct k_work *work) {
    struct kscan_ptty_data *data = CONTAINER_OF(work, struct kscan_ptty_data, work);
    struct k_work_delayable *q = k_work_delayable_from_work(work);
    char cmd[MAX_CMD_LEN];
    int err;

    switch (err = recv_line(command_uart, cmd)) {
    case -ENODATA:
        LOG_INF("all commands processed, stopping ptty work queue");
        if (kscan_ptty_config.exit_after)
            exit(0);
        else
            return;
    case -EOVERFLOW:
        cmd[MAX_CMD_LEN - 1] = '\0';
        LOG_ERR("command too long: \"%s...\"", cmd);
        exit(1);
    case 0:
        break;
    default:
        LOG_ERR("unknown error while reading command from tty: %d", -err);
        exit(1);
    }

    int args[2] = {-1, -1};
    bool is_press = true;
    switch (kscan_parse_command(cmd, args)) {
    case 'p':
        break;
    case 'r':
        is_press = false;
        break;
    case 'w':
        LOG_DBG("cmd[%d] wait %dms", (data->cmd_idx)++, args[0]);
        k_work_schedule(q, K_MSEC(args[0]));
        return;
    default:
        LOG_ERR("invalid command: %s", cmd);
        return;
    }

    LOG_DBG("cmd[%d] %s row %d col %d", (data->cmd_idx)++, is_press ? "press" : "release", args[0],
            args[1]);
    data->callback(DEVICE_DT_INST_GET(0), args[0], args[1], is_press);
    k_work_schedule(q, K_MSEC(kscan_ptty_config.delay_ms));
}