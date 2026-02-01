#ifndef BLE_LIB_H
#define BLE_LIB_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/services/bas.h> // Battery Service
#include <zephyr/bluetooth/services/hrs.h> // Heart Rate Service

/* UUID of the Remote Service */
// Project ID: 065 (3rd entry)
// MFG ID = 0x02DF (4th entry)
#define BT_UUID_REMOTE_SERV_VAL \
        BT_UUID_128_ENCODE(0xe9ea0000, 0xe19b, 0x0065, 0x02DF, 0xc7907585fc48)

#define BT_UUID_REMOTE_SERVICE          BT_UUID_DECLARE_128(BT_UUID_REMOTE_SERV_VAL)

enum bt_data_notifications_enabled {
    BT_DATA_NOTIFICATIONS_ENABLED,
    BT_DATA_NOTIFICATIONS_DISABLED,
};

struct bt_remote_srv_cb {
    void (*notif_changed)(enum bt_data_notifications_enabled status);
    void (*data_rx)(struct bt_conn *conn, const uint8_t *const data, uint16_t len);
};

int bluetooth_init(struct bt_conn_cb *bt_cb, struct bt_remote_srv_cb *remote_cb);
uint8_t bluetooth_get_battery_level(void);
void bluetooth_set_battery_level(int32_t level);
ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
ssize_t read_temperature_data_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
ssize_t read_error_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
void ccc_cfg_changed_cb(const struct bt_gatt_attr *attr, uint16_t value);
void on_connected(struct bt_conn *conn, uint8_t ret);
void on_disconnected(struct bt_conn *conn, uint8_t reason);
void on_data_rx(struct bt_conn *conn, const uint8_t *const data, uint16_t len);
void on_notif_changed(enum bt_data_notifications_enabled status);

extern const struct bt_gatt_attr *attr;
extern struct bt_conn *current_conn;
extern struct bt_conn_cb bluetooth_callbacks;
extern struct bt_remote_srv_cb remote_service_callbacks;
extern struct bt_uuid_128 remote_err_uuid;
extern struct bt_uuid_128 remote_serv_uuid;
extern struct bt_uuid_128 remote_temperature_data_uuid;
extern struct bt_uuid_128 remote_msg_uuid;

#endif
