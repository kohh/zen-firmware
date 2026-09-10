/*
 * Split peripheral advertising retry.
 *
 * ZMK v0.3 restarts directed advertising from the BT `disconnected` /
 * `connected(err = ADV_TIMEOUT)` callbacks. If the stale connection object
 * for the central has not been recycled yet, bt_le_adv_start() fails with
 * -EINVAL ("Found valid connection ... in disconnected state") and ZMK never
 * retries, leaving the peripheral silent until it is power-cycled.
 * ZMK main fixes this with the `recycled` callback, which Zephyr 3.5 lacks.
 *
 * This module retries advertising with the same parameters ZMK uses until
 * advertising is running (-EALREADY) or the central is connected again.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>

#include <zmk/split/bluetooth/peripheral.h>
#include <zmk/split/bluetooth/uuid.h>

LOG_MODULE_REGISTER(zen_adv_retry, CONFIG_ZMK_LOG_LEVEL);

#define RETRY_INTERVAL_MS 500
#define RETRY_MAX 60 /* 30 s */

static const struct bt_data zen_ble_ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_SOME, 0x0f, 0x18 /* Battery Service */),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, ZMK_SPLIT_BT_SERVICE_UUID)};

static struct k_work_delayable retry_work;
static int retry_count;

static void each_bond(const struct bt_bond_info *info, void *user_data) {
    bt_addr_le_t *addr = (bt_addr_le_t *)user_data;
    if (bt_addr_le_cmp(&info->addr, BT_ADDR_LE_NONE) != 0) {
        bt_addr_le_copy(addr, &info->addr);
    }
}

static int try_start_advertising(void) {
    bt_addr_le_t central_addr = bt_addr_le_none;
    bt_foreach_bond(BT_ID_DEFAULT, each_bond, &central_addr);

    if (bt_addr_le_cmp(&central_addr, BT_ADDR_LE_NONE) != 0) {
        struct bt_le_adv_param adv_param = *BT_LE_ADV_CONN_DIR_LOW_DUTY(&central_addr);
        return bt_le_adv_start(&adv_param, NULL, 0, NULL, 0);
    }
    return bt_le_adv_start(BT_LE_ADV_CONN, zen_ble_ad, ARRAY_SIZE(zen_ble_ad), NULL, 0);
}

static void retry_handler(struct k_work *work) {
    if (zmk_split_bt_peripheral_is_connected()) {
        return;
    }

    int err = try_start_advertising();
    if (err == 0) {
        LOG_WRN("Advertising restarted by retry (attempt %d)", retry_count + 1);
        return;
    }
    if (err == -EALREADY) {
        /* ZMK's own advertising is running; nothing to do. */
        return;
    }

    retry_count++;
    if (retry_count < RETRY_MAX) {
        LOG_DBG("Advertising start failed (%d), retrying", err);
        k_work_schedule(&retry_work, K_MSEC(RETRY_INTERVAL_MS));
    } else {
        LOG_ERR("Advertising retry gave up after %d attempts (%d)", retry_count, err);
    }
}

static void schedule_retry(void) {
    retry_count = 0;
    k_work_reschedule(&retry_work, K_MSEC(RETRY_INTERVAL_MS));
}

static void on_connected(struct bt_conn *conn, uint8_t err) {
    if (err != 0) {
        /* Includes BT_HCI_ERR_ADV_TIMEOUT after high-duty directed advertising. */
        schedule_retry();
    }
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason) { schedule_retry(); }

static struct bt_conn_cb zen_conn_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
};

static int zen_adv_retry_init(void) {
    k_work_init_delayable(&retry_work, retry_handler);
    bt_conn_cb_register(&zen_conn_callbacks);
    LOG_INF("Split advertising retry enabled");
    return 0;
}

SYS_INIT(zen_adv_retry_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
