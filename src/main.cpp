//
// Created by awalol on 2026/3/4.
//

#include <cstdio>
#include "bsp/board_api.h"
#include "bt.h"
#include "button_functions.h"
#include "utils.h"
#include "resample.h"
#include "audio.h"
#include "wake.h"
#ifdef ENABLE_WAKE_HID
#include "ps_shortcut.h"
#endif
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "state_mgr.h"
#ifdef DIAG_LOG
#include "hci_dump.h"
#include "hci_dump_embedded_stdout.h"
#endif
#if ENABLE_SERIAL
#include "pico/stdio_usb.h"
#endif
#include "config.h"
#include "cmd.h"
#include "dse.h"
#if ENABLE_BATT_LED
#include "battery_led.h"
#endif

// Pico SDK speciifically for waiting on conditions
#include "pico/critical_section.h"

int reportSeqCounter = 0;
uint8_t packetCounter = 0;
bool spk_active = false;

uint8_t interrupt_in_data[63] = {
    0x7f, 0x7d, 0x7f, 0x7e, 0x00, 0x00, 0xa7,
    0x08, 0x00, 0x00, 0x00, 0x52, 0x43, 0x30, 0x41,
    0x01, 0x00, 0x0e, 0x00, 0xef, 0xff, 0x03, 0x03,
    0x7b, 0x1b, 0x18, 0xf0, 0xcc, 0x9c, 0x60, 0x00,
    0xfc, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x09, 0x09, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xa7, 0xad, 0x60, 0x00, 0x29, 0x18, 0x00,
    0x53, 0x9f, 0x28, 0x35, 0xa5, 0xa8, 0x0c, 0x8b
};

critical_section_t report_cs;
volatile bool report_dirty = false;

void __not_in_flash_func(interrupt_loop)() {
    if (!tud_hid_ready()) return;

    // TODO: Refactor for better code reuse
    if (get_config().polling_rate_mode != 2) {
        if (!tud_hid_report(0x01, interrupt_in_data, 63)) {
            printf("[USBHID] tud_hid_report error\n");
        }
        return;
    }

    bool should_send = false;
    // Local buffer to hold the report data while we prepare it to send. 
    uint8_t safe_report[63];


    critical_section_enter_blocking(&report_cs);
    if (report_dirty) {
        memcpy(safe_report, interrupt_in_data, 63);
        report_dirty = false;
        should_send = true;
    }
    critical_section_exit(&report_cs);

    // Only send to TinyUSB if we actually grabbed fresh data
    if (should_send) {
        if (!tud_hid_report(0x01, safe_report, 63)) {
            printf("[USBHID] tud_hid_report error\n");

            // If the report failed to queue, restore the dirty flag 
            // so we try again on the next loop iteration.
            critical_section_enter_blocking(&report_cs);
            report_dirty = true;
            critical_section_exit(&report_cs);
        }
    }
}

#ifdef DIAG_LOG
static uint32_t diag_input_count = 0;
extern uint32_t diag_bt_write_count[256];
// BT input gap summary per 10 s window: the 4 s stall shows as gaps > 20 ms
static uint32_t diag_gaps_long = 0, diag_gaps_mid = 0, diag_gap_max_us = 0;

// Serial console: one letter plus optional digits, e.g. "a0" (see diag_radio_command).
static void diag_console() {
    static char cmd = 0;
    static int arg = 0;
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c >= '0' && c <= '9' && cmd) {
            arg = arg * 10 + (c - '0');
        } else if (c == '\r' || c == '\n') {
            if (cmd) diag_radio_command(cmd, arg);
            cmd = 0;
            arg = 0;
        } else if (c > ' ') {
            cmd = (char) c;
            arg = 0;
        }
    }
}

// Once a second: input reports received, and packets sent to the controller by report ID.
static void diag_tick() {
    static uint64_t next_us = 0, next_summary_us = 0;
    const uint64_t now_us = time_us_64();
    diag_console();
    if (now_us >= next_summary_us) {
        if (next_summary_us) {
            printf("[DIAG] %lu ms 10s: gaps >20ms %lu, 10-20ms %lu, max %lu us\n", (unsigned long) (now_us / 1000),
                   (unsigned long) diag_gaps_long, (unsigned long) diag_gaps_mid, (unsigned long) diag_gap_max_us);
        }
        diag_gaps_long = diag_gaps_mid = diag_gap_max_us = 0;
        next_summary_us = now_us + 10 * 1000 * 1000;
    }
    if (now_us < next_us) return;
    next_us = now_us + 1000 * 1000;
    printf("[DIAG] %lu ms 1s: in %lu, out 0x31 %lu, 0x32 %lu, 0x36 %lu\n", (unsigned long) (now_us / 1000),
           (unsigned long) diag_input_count, (unsigned long) diag_bt_write_count[0x31],
           (unsigned long) diag_bt_write_count[0x32], (unsigned long) diag_bt_write_count[0x36]);
    diag_input_count = 0;
    diag_bt_write_count[0x31] = diag_bt_write_count[0x32] = diag_bt_write_count[0x36] = 0;
}
#endif

void __not_in_flash_func(on_bt_data)(CHANNEL_TYPE channel, uint8_t *data, uint16_t len) {
    // printf("[Main] BT data callback: channel=%u len=%u\n", channel, len);
    if (channel == INTERRUPT && len > 2 && data[1] == 0x31) {
        // Mic audio: controller signals mic payload via bit1 of data[2];
        // the opus-encoded mic frame starts at data+4.
        if ((data[2] >> 1) & 1) {
            if (len >= 4) {
                mic_add_queue(data + 4, len - 4);
            }
            return;
        }
#ifdef DIAG_LOG
        {
            static uint64_t last_us = 0;
            const uint64_t now_us = time_us_64();
            diag_input_count++;
            if (last_us && now_us - last_us > 8000) {
                const uint32_t gap = (uint32_t) (now_us - last_us);
                printf("[DIAG] %lu ms BT input gap %lu us\n", (unsigned long) (now_us / 1000), (unsigned long) gap);
                if (gap > 20000) diag_gaps_long++;
                else if (gap > 10000) diag_gaps_mid++;
                if (gap > diag_gap_max_us) diag_gap_max_us = gap;
            }
            last_us = now_us;
        }
#endif
        if ((data[56] & 1) != (interrupt_in_data[53] & 1)) {
            set_headset(data[56] & 1);
        }

        // Wake-on-PS must observe every BT input report regardless of polling
        // mode: the wake feature has its own state to maintain (button-byte
        // diff for edge detection) and short-circuiting it on non-2 polling
        // modes silently breaks wake while the host is suspended.
        wake_on_bt_input(data + 3, len - 3);
        #ifdef ENABLE_WAKE_HID
        ps_shortcut_tick(data + 3, len - 3);
        #endif

        if (get_config().polling_rate_mode != 2) {
            memcpy(interrupt_in_data, data + 3, 63);
#if ENABLE_BATT_LED
            battery_led_note_report();
#endif
            return;
        }

        // We add the critical section here to avoid any race conditions when writing to the interrupt_in_data buffer,
        // which is shared between the main loop and this callback.
        // The critical section ensures that only one thread can access the buffer at a time,
        // preventing data corruption and ensuring thread safety.
        // We also set the report_dirty flag to true to indicate that new data is available
        //  and needs to be sent in the next interrupt report.
        critical_section_enter_blocking(&report_cs);
        memcpy(interrupt_in_data, data + 3, 63);
        report_dirty = true;
        critical_section_exit(&report_cs);
#if ENABLE_BATT_LED
        battery_led_note_report();
#endif
    }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
#ifdef ENABLE_WAKE_HID
    if (itf == 1) {
        if (reqlen >= 8) {
            memset(buffer, 0, 8);
            return 8;
        }
        return 0;
    }
#endif
    (void) itf;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    if (is_pico_cmd(report_id)) {
        return pico_cmd_get(report_id, buffer, reqlen);
    }

    // DSE profiles: while the unlock + prefetch is still in progress, return 0
    // (NAK) for profile reads so the PS app retries rather than caching an
    // empty snapshot. Still kick off the background BT fetch.
    if (dse_is_profile_report(report_id) && !dse_profiles_ready()) {
        get_feature_data(report_id, reqlen);
        return 0;
    }

    std::vector<uint8_t> feature_data = get_feature_data(report_id, reqlen);
    if (!feature_data.empty()) {
        memcpy(buffer, feature_data.data() + 1, feature_data.size() - 1);
    }

    return feature_data.empty() ? 0 : feature_data.size() - 1;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void) rhport;
    uint8_t const itf = tu_u16_low(p_request->wIndex); // wInterface
    uint8_t const alt = tu_u16_low(p_request->wValue); // bAlternateSetting

    if (itf == 1) {
        printf("[AUDIO] Set interface Speaker to alternate setting %d\n", alt);
        spk_active = alt;
    }
    if (itf == 2) { // ITF_NUM_AUDIO_STREAMING_IN (microphone)
        printf("[AUDIO] Set interface Microphone to alternate setting %d\n", alt);
        set_mic_active(alt);
    }

    return true;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {
#ifdef ENABLE_WAKE_HID
    if (itf == 1) {
        // Drop keyboard SET_REPORT (host LED state).
        return;
    }
#endif
    (void) itf;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) bufsize;

    if (is_pico_cmd(report_id)) {
#if ENABLE_VERBOSE
        printf("[HID] Receive 0xf6 setting config, funcid:0x%02X\n", buffer[0]);
#endif
        pico_cmd_set(report_id, buffer, bufsize);
        return;
    }

    // INTERRUPT OUT
    if (report_id == 0) {
        switch (buffer[0]) {
            case 0x02: {
                state_update(buffer + 1, bufsize - 1);
                bool send_now = ((buffer[1] >> 1) & 1) || // UseRumbleNotHaptics
                                ((buffer[39] >> 3) & 1); // UseRumbleNotHaptics2
                if (!send_now && spk_active) {
                    break;
                }
                uint8_t outputData[78]{};
                outputData[0] = 0x31;
                outputData[1] = reportSeqCounter << 4;
                if (++reportSeqCounter == 256) {
                    reportSeqCounter = 0;
                }
                outputData[2] = 0x10;
                // memcpy(outputData + 3, buffer + 1, bufsize - 1);
                state_set(outputData + 3, sizeof(SetStateData));
                bt_write(outputData, sizeof(outputData));
                break;
            }
        }
    }
    if (report_id == 0x80 ||
        // DSE: Write Profile Block
        report_id == 0x60 ||
        report_id == 0x62 ||
        report_id == 0x61) {
        set_feature_data(report_id, const_cast<uint8_t *>(buffer), bufsize);
    }
}

int main() {
#if SYS_CLOCK_KHZ != 150000
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(1000);
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);
#endif

    board_init();
    // Before tusb_init(): the configuration descriptor's bInterval comes from polling_rate_mode,
    // and serial builds enumerate as soon as tud_task() runs.
    config_load();
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
#if !ENABLE_SERIAL
    sleep_ms(150);
    tud_disconnect();
#endif
    board_init_after_tusb();
#if ENABLE_SERIAL
    stdio_usb_init();
#endif

#ifdef DIAG_LOG
    // Hold boot until the serial logger attaches (max 20 s) so TLV init and pairing logs aren't lost
    for (uint32_t t0 = to_ms_since_boot(get_absolute_time());
         !stdio_usb_connected() && to_ms_since_boot(get_absolute_time()) - t0 < 20000;) {
        tud_task();
    }
    printf("[DIAG] %lu ms serial attached, starting BT\n", (unsigned long) (time_us_64() / 1000));
    hci_dump_init(hci_dump_embedded_stdout_get_instance());
    hci_dump_enable_packet_log(false);
#endif

    // Start core1 before cyw43_arch_init(): that sets up BTstack's TLV link key store,
    // and with PICO_FLASH_ASSUME_CORE1_SAFE=0 every flash_safe_execute() fails with
    // NOT_PERMITTED until core1 has registered as a lockout victim. The store's bank
    // header then never got written, so each boot rewrote keys at offset 8 on top of
    // the old ones and deletes found nothing. core1_entry reads the config (loaded above).
    audio_init();
#if !DISABLE_SPEAKER_PROC
    while (!multicore_lockout_victim_is_initialized(1)) tight_loop_contents();
#endif

    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return 1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);

#if ENABLE_BATT_LED
    battery_led_init();
#endif

#if !ENABLE_SERIAL
    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
        // 当崩溃重启以后，闪三下灯
        for (int i = 0; i < 6; i++) {
            if (i % 2 == 0) {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
            } else {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
            }
            sleep_ms(500);
        }
    } else {
        printf("Clean boot\n");
    }
#endif

    // Initialize the critical section for the report buffer
    critical_section_init(&report_cs);
    wake_init();

    bt_init();
    bt_register_data_callback(on_bt_data);

    state_init();

#if !ENABLE_SERIAL
    watchdog_enable(1000, true);
#endif

    while (1) {
#ifdef DIAG_LOG
        // Per-step timestamps; a loop iteration over 15 ms is printed with its breakdown
        uint32_t ts[9];
        ts[0] = time_us_32();
#define DIAG_STEP(i) ts[i] = time_us_32()
#else
#define DIAG_STEP(i)
#endif
#if !ENABLE_SERIAL
        watchdog_update();
#endif
        cyw43_arch_poll();
        DIAG_STEP(1);
        tud_task();
        DIAG_STEP(2);
        wake_task();
        audio_loop();
        DIAG_STEP(3);
        interrupt_loop();
        DIAG_STEP(4);
#if ENABLE_BATT_LED
        battery_led_tick();
#endif
        button_check();
        DIAG_STEP(5);
        bt_inquiring_led();
        DIAG_STEP(6);
        dse_task();
        DIAG_STEP(7);
#ifdef DIAG_LOG
        diag_tick();
        ts[8] = time_us_32();
        if (ts[8] - ts[0] > 15000) {
            printf("[DIAG] %lu ms loop stall %lu us: cyw43 %lu, tud %lu, audio %lu, hid %lu, button %lu, led %lu, dse %lu\n",
                   (unsigned long) (time_us_64() / 1000), (unsigned long) (ts[8] - ts[0]),
                   (unsigned long) (ts[1] - ts[0]), (unsigned long) (ts[2] - ts[1]), (unsigned long) (ts[3] - ts[2]),
                   (unsigned long) (ts[4] - ts[3]), (unsigned long) (ts[5] - ts[4]), (unsigned long) (ts[6] - ts[5]),
                   (unsigned long) (ts[7] - ts[6]));
        }
#endif
#undef DIAG_STEP
    }
}
