/*
 * Copyright (c) 2024 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/battery.h>
#include <zmk/usb.h>

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
#include <zmk/split/central.h>
#endif

LOG_MODULE_REGISTER(usb_hid_battery, CONFIG_ZMK_LOG_LEVEL);

/* HID Report IDs - use high IDs to avoid conflicts with ZMK's existing reports */
#define BATTERY_REPORT_ID_LEFT   0x05
#define BATTERY_REPORT_ID_RIGHT  0x06

/* HID Report Descriptor for Battery System
 * Reports two batteries: left half and right half
 */
static const uint8_t battery_hid_report_desc[] = {
    /* Left Half Battery Report */
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x06,       /* Usage (Keyboard) */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, BATTERY_REPORT_ID_LEFT,  /* Report ID */
    
    /* Battery Strength */
    0x05, 0x06,       /* Usage Page (Generic Device Controls) */
    0x09, 0x20,       /* Usage (Battery Strength) */
    0x15, 0x00,       /* Logical Minimum (0) */
    0x26, 0x64, 0x00, /* Logical Maximum (100) */
    0x75, 0x08,       /* Report Size (8 bits) */
    0x95, 0x01,       /* Report Count (1) */
    0x81, 0x02,       /* Input (Data, Variable, Absolute) */
    0xC0,             /* End Collection */

    /* Right Half Battery Report */
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x06,       /* Usage (Keyboard) */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, BATTERY_REPORT_ID_RIGHT,  /* Report ID */
    
    /* Battery Strength */
    0x05, 0x06,       /* Usage Page (Generic Device Controls) */
    0x09, 0x20,       /* Usage (Battery Strength) */
    0x15, 0x00,       /* Logical Minimum (0) */
    0x26, 0x64, 0x00, /* Logical Maximum (100) */
    0x75, 0x08,       /* Report Size (8 bits) */
    0x95, 0x01,       /* Report Count (1) */
    0x81, 0x02,       /* Input (Data, Variable, Absolute) */
    0xC0,             /* End Collection */
};

/* Battery report structures */
struct battery_report {
    uint8_t report_id;
    uint8_t level;
} __packed;

/* Store battery levels for both halves */
static struct battery_report left_battery_report = {
    .report_id = BATTERY_REPORT_ID_LEFT,
    .level = 0
};

static struct battery_report right_battery_report = {
    .report_id = BATTERY_REPORT_ID_RIGHT,
    .level = 0
};

static const struct device *hid_dev;
static bool usb_hid_battery_ready = false;

/* Forward declarations */
static void send_battery_report(struct battery_report *report);

/* HID callbacks */
static void hid_int_ready_cb(const struct device *dev) {
    ARG_UNUSED(dev);
}

static int hid_get_report_cb(const struct device *dev, 
                              struct usb_setup_packet *setup,
                              int32_t *len, uint8_t **data) {
    uint8_t report_id = setup->wValue & 0xFF;
    uint8_t report_type = (setup->wValue >> 8) & 0xFF;
    
    LOG_DBG("Get report: type=%d, id=%d", report_type, report_id);
    
    /* Handle Feature or Input report requests */
    if (report_type == 0x01 || report_type == 0x03) { /* Input or Feature */
        switch (report_id) {
        case BATTERY_REPORT_ID_LEFT:
            /* Return stored value from event listener */
            *data = (uint8_t *)&left_battery_report;
            *len = sizeof(left_battery_report);
            LOG_DBG("Returning left battery: %d%%", left_battery_report.level);
            return 0;
            
        case BATTERY_REPORT_ID_RIGHT:
            /* Return stored value from event listener */
            *data = (uint8_t *)&right_battery_report;
            *len = sizeof(right_battery_report);
            LOG_DBG("Returning right battery: %d%%", right_battery_report.level);
            return 0;
        }
    }
    
    return -ENOTSUP;
}

static const struct hid_ops hid_ops = {
    .int_in_ready = hid_int_ready_cb,
    .get_report = hid_get_report_cb,
};

/* Send battery report over USB interrupt endpoint */
static void send_battery_report(struct battery_report *report) {
    if (!usb_hid_battery_ready || !hid_dev) {
        return;
    }
    
    /* Only send when USB is connected */
    enum zmk_usb_conn_state usb_state = zmk_usb_get_conn_state();
    if (usb_state != ZMK_USB_CONN_HID) {
        return;
    }
    
    int ret = hid_int_ep_write(hid_dev, (uint8_t *)report, sizeof(*report), NULL);
    if (ret < 0) {
        LOG_WRN("Failed to send battery report: %d", ret);
    } else {
        LOG_DBG("Sent battery report: id=%d, level=%d", report->report_id, report->level);
    }
}

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
/* Peripheral battery event listener - this is what we need for dongle setup */
static int peripheral_battery_listener(const zmk_event_t *eh) {
    const struct zmk_peripheral_battery_state_changed *ev = 
        as_zmk_peripheral_battery_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    
    LOG_INF("Peripheral %d battery changed: %d%%", ev->source, ev->state_of_charge);
    
    /* source 0 = first peripheral (left), source 1 = second peripheral (right) */
    if (ev->source == 0) {
        left_battery_report.level = ev->state_of_charge;
        send_battery_report(&left_battery_report);
    } else if (ev->source == 1) {
        right_battery_report.level = ev->state_of_charge;
        send_battery_report(&right_battery_report);
    }
    
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(usb_hid_battery_peripheral, peripheral_battery_listener);
ZMK_SUBSCRIPTION(usb_hid_battery_peripheral, zmk_peripheral_battery_state_changed);
#endif /* IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING) */

/* Initialization */
static int usb_hid_battery_init(void) {
    int ret;
    
    hid_dev = device_get_binding("HID_1");
    if (hid_dev == NULL) {
        LOG_ERR("Failed to get HID device binding for battery");
        return -ENODEV;
    }
    
    usb_hid_register_device(hid_dev,
                            battery_hid_report_desc,
                            sizeof(battery_hid_report_desc),
                            &hid_ops);
    
    ret = usb_hid_init(hid_dev);
    if (ret != 0) {
        LOG_ERR("Failed to initialize battery HID device: %d", ret);
        return ret;
    }
    
    usb_hid_battery_ready = true;
    
    LOG_INF("USB HID Battery reporting initialized for split keyboard");
    
    return 0;
}

SYS_INIT(usb_hid_battery_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
