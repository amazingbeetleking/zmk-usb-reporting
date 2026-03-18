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
#include <zmk/events/split_peripheral_status_changed.h>
#endif

LOG_MODULE_REGISTER(usb_hid_battery, CONFIG_ZMK_LOG_LEVEL);

/* HID Report IDs - use high IDs to avoid conflicts with ZMK's existing reports */
#define BATTERY_REPORT_ID_CENTRAL     0x05
#define BATTERY_REPORT_ID_PERIPHERAL  0x06

/* HID Report Descriptor for Battery System
 * Uses the Generic Desktop Page (0x01) with System Control and Battery Strength
 * This is the format most likely to be recognized by operating systems
 */
static const uint8_t battery_hid_report_desc[] = {
    /* Central/Dongle Battery Report */
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x06,       /* Usage (Keyboard) - Associate with keyboard device */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, BATTERY_REPORT_ID_CENTRAL,  /* Report ID */
    
    /* Battery Strength */
    0x05, 0x06,       /* Usage Page (Generic Device Controls) */
    0x09, 0x20,       /* Usage (Battery Strength) */
    0x15, 0x00,       /* Logical Minimum (0) */
    0x26, 0x64, 0x00, /* Logical Maximum (100) */
    0x75, 0x08,       /* Report Size (8 bits) */
    0x95, 0x01,       /* Report Count (1) */
    0x81, 0x02,       /* Input (Data, Variable, Absolute) */
    0xC0,             /* End Collection */

#if IS_ENABLED(CONFIG_ZMK_USB_HID_BATTERY_REPORTING_SPLIT)
    /* Peripheral Battery Report (for split keyboards) */
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x06,       /* Usage (Keyboard) */
    0xA1, 0x01,       /* Collection (Application) */
    0x85, BATTERY_REPORT_ID_PERIPHERAL,  /* Report ID */
    
    /* Battery Strength */
    0x05, 0x06,       /* Usage Page (Generic Device Controls) */
    0x09, 0x20,       /* Usage (Battery Strength) */
    0x15, 0x00,       /* Logical Minimum (0) */
    0x26, 0x64, 0x00, /* Logical Maximum (100) */
    0x75, 0x08,       /* Report Size (8 bits) */
    0x95, 0x01,       /* Report Count (1) */
    0x81, 0x02,       /* Input (Data, Variable, Absolute) */
    0xC0,             /* End Collection */
#endif
};

/* Battery report structures */
struct battery_report {
    uint8_t report_id;
    uint8_t level;
} __packed;

static struct battery_report central_battery_report = {
    .report_id = BATTERY_REPORT_ID_CENTRAL,
    .level = 0
};

#if IS_ENABLED(CONFIG_ZMK_USB_HID_BATTERY_REPORTING_SPLIT)
static struct battery_report peripheral_battery_report = {
    .report_id = BATTERY_REPORT_ID_PERIPHERAL,
    .level = 0
};
#endif

static const struct device *hid_dev;
static bool usb_hid_battery_ready = false;

/* Forward declarations */
static void send_battery_report(struct battery_report *report);

/* HID callbacks */
static void hid_int_ready_cb(const struct device *dev) {
    ARG_UNUSED(dev);
    /* Interrupt endpoint is ready */
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
        case BATTERY_REPORT_ID_CENTRAL:
            central_battery_report.level = zmk_battery_state_of_charge();
            *data = (uint8_t *)&central_battery_report;
            *len = sizeof(central_battery_report);
            return 0;
            
#if IS_ENABLED(CONFIG_ZMK_USB_HID_BATTERY_REPORTING_SPLIT)
        case BATTERY_REPORT_ID_PERIPHERAL: {
            uint8_t level = 0;
            zmk_split_central_get_peripheral_battery_level(0, &level);
            peripheral_battery_report.level = level;
            *data = (uint8_t *)&peripheral_battery_report;
            *len = sizeof(peripheral_battery_report);
            return 0;
        }
#endif
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

/* ZMK Battery event listener */
static int battery_state_listener(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    
    LOG_DBG("Battery state changed: %d%%", ev->state_of_charge);
    
    central_battery_report.level = ev->state_of_charge;
    send_battery_report(&central_battery_report);
    
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(usb_hid_battery, battery_state_listener);
ZMK_SUBSCRIPTION(usb_hid_battery, zmk_battery_state_changed);

#if IS_ENABLED(CONFIG_ZMK_USB_HID_BATTERY_REPORTING_SPLIT)
/* Peripheral battery event listener */
static int peripheral_battery_listener(const zmk_event_t *eh) {
    const struct zmk_peripheral_battery_state_changed *ev = 
        as_zmk_peripheral_battery_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    
    LOG_DBG("Peripheral %d battery changed: %d%%", ev->source, ev->state_of_charge);
    
    /* For now, only report first peripheral (source 0) */
    if (ev->source == 0) {
        peripheral_battery_report.level = ev->state_of_charge;
        send_battery_report(&peripheral_battery_report);
    }
    
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(usb_hid_battery_peripheral, peripheral_battery_listener);
ZMK_SUBSCRIPTION(usb_hid_battery_peripheral, zmk_peripheral_battery_state_changed);
#endif

/* Initialization */
static int usb_hid_battery_init(void) {
    int ret;
    
    hid_dev = device_get_binding("HID_1");
    if (hid_dev == NULL) {
        LOG_ERR("Failed to get HID device binding");
        return -ENODEV;
    }
    
    usb_hid_register_device(hid_dev,
                            battery_hid_report_desc,
                            sizeof(battery_hid_report_desc),
                            &hid_ops);
    
    ret = usb_hid_init(hid_dev);
    if (ret != 0) {
        LOG_ERR("Failed to initialize HID device: %d", ret);
        return ret;
    }
    
    usb_hid_battery_ready = true;
    
    LOG_INF("USB HID Battery reporting initialized");
    
    return 0;
}

SYS_INIT(usb_hid_battery_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
