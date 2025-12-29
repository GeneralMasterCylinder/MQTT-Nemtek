#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "config_wizard.h"

const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
SystemConfig sys_cfg;
#define MAX_SCAN_RESULTS 20
typedef struct {
    char ssid[33];
    int rssi;
} ScanResult;

static ScanResult scan_results[MAX_SCAN_RESULTS];
static int scan_count = 0;
bool scan_complete = false;

static int scan_result_cb(void *env, const cyw43_ev_scan_result_t *result) {
    if (scan_count >= MAX_SCAN_RESULTS) return 0; 
    
    
    strncpy(scan_results[scan_count].ssid, (const char*)result->ssid, 32);
    scan_results[scan_count].ssid[32] = '\0'; 
    scan_results[scan_count].rssi = result->rssi;
    
    
    scan_count++;
    return 0; 
}

void read_input(char* buffer, int max_len) {
    int i = 0;
    memset(buffer, 0, max_len);
    while(true) {
        int c = getchar();
        if (c == '\r' || c == '\n') { printf("\n"); break; }
        if (c == 0x08 || c == 0x7F) {
            if (i > 0) { i--; printf("\b \b"); }
        } else if (i < max_len - 1 && c >= 32 && c <= 126) {
            buffer[i++] = (char)c; printf("%c", c);
        }
    }
    buffer[i] = '\0';
}

void load_config() {
    memcpy(&sys_cfg, flash_target_contents, sizeof(SystemConfig));

    if (sys_cfg.magic != CONFIG_MAGIC) {
        printf("Config: No valid config. Loading Defaults.\n");
        
        strncpy(sys_cfg.wifi_ssid, DEFAULT_WIFI_SSID, 32);
        strncpy(sys_cfg.wifi_pass, DEFAULT_WIFI_PASS, 63);
        strncpy(sys_cfg.mqtt_server, DEFAULT_MQTT_IP, 63);
        sys_cfg.mqtt_port = DEFAULT_MQTT_PORT;
        strncpy(sys_cfg.mqtt_user, DEFAULT_MQTT_USER, 32);
        strncpy(sys_cfg.mqtt_pass, DEFAULT_MQTT_PASS, 32);
        for (int i=0; i<4; i++)
        {
            sys_cfg.master_pin[i] = DEFAULT_MASTER_PIN[i]-48;     
        }
        
        
        sys_cfg.static_ip = DEFAULT_USE_STATIC;
        strncpy(sys_cfg.ip, DEFAULT_STATIC_IP, 15);
        strncpy(sys_cfg.mask, DEFAULT_STATIC_MASK, 15);
        strncpy(sys_cfg.gateway, DEFAULT_STATIC_GW, 15);
        
        // Calibration Defaults
        sys_cfg.r1_val = DEFAULT_R1_VAL;
        sys_cfg.r2_val = DEFAULT_R2_VAL;
    }
}

void save_config() {
    sys_cfg.magic = CONFIG_MAGIC; 
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, 4096);
    flash_range_program(FLASH_TARGET_OFFSET, (const uint8_t *)&sys_cfg, 4096);
    restore_interrupts(ints);
    printf("Config: Saved to Flash.\n");
}

void perform_wifi_scan() {
    printf("Initializing Wi-Fi for scan...\n");
    if (cyw43_arch_init()) {
        printf("Failed to init Wi-Fi hardware.\n");
        return;
    }
    cyw43_arch_enable_sta_mode();

    printf("Scanning... (Wait ~5s)\n");
    scan_count = 0;

    cyw43_wifi_scan_options_t scan_options = {0};
    int err = cyw43_wifi_scan(&cyw43_state, &scan_options, NULL, scan_result_cb);
    
    if (err == 0) {
        uint32_t start = to_ms_since_boot(get_absolute_time());
        while (cyw43_wifi_scan_active(&cyw43_state)) {
            sleep_ms(100);
            if (to_ms_since_boot(get_absolute_time()) - start > 10000) break;
        }
    } else {
        printf("Scan failed to start (%d)\n", err);
    }
}

void run_config_wizard() {
    char buf[64];
    printf("\n\n=== NEMTEK PICO CONFIGURATION ===\n");

    
    printf("\n[1/6] Wi-Fi SSID\n");
    printf("1. Scan\n2. Keep Current (%s)\n3. Manual\nSelect: ", sys_cfg.wifi_ssid);
    read_input(buf, 10);
    if (buf[0] == '1') {
        perform_wifi_scan();
        printf("\nFound %d networks:\n", scan_count);
        for(int i=0; i<scan_count; i++) printf("%d. %s\n", i+1, scan_results[i].ssid);
        printf("Select #: ");
        read_input(buf, 5);
        int choice = atoi(buf);
        if (choice > 0 && choice <= scan_count) strncpy(sys_cfg.wifi_ssid, scan_results[choice-1].ssid, 32);
    } else if (buf[0] == '3') {
        printf("SSID: "); read_input(sys_cfg.wifi_ssid, 32);
    }

    printf("\n[2/6] Wi-Fi Pass: ");
    read_input(buf, 63);
    if (strlen(buf) > 0) strncpy(sys_cfg.wifi_pass, buf, 63);

    
    printf("\n[3/6] MQTT IP (%s): ", sys_cfg.mqtt_server);
    read_input(buf, 63);
    if (strlen(buf) > 0) strncpy(sys_cfg.mqtt_server, buf, 63);

    printf("MQTT User (%s): ", sys_cfg.mqtt_user);
    read_input(buf, 32);
    if (strlen(buf) > 0) strncpy(sys_cfg.mqtt_user, buf, 32);

    printf("MQTT Pass: ");
    read_input(buf, 32);
    if (strlen(buf) > 0) strncpy(sys_cfg.mqtt_pass, buf, 32);
    char pin[5];
    for (int i=0; i<4; i++)
    {
        pin[i]=sys_cfg.master_pin[i]+48;
    }
    pin[4] ='\0';
    printf("\n[4/6] Master PIN (%s): ", pin);
    read_input(buf, 8);
    if (strlen(buf) > 3) 
    {
        for (int i=0; i<4; i++)
        {
            sys_cfg.master_pin[i] = buf[i]-48;     
        }

    }

    
    printf("\n[5/6] Use Static IP? (y/n): ");
    read_input(buf, 5);
    if (buf[0] == 'y' || buf[0] == 'Y') {
        sys_cfg.static_ip = true;
        printf("IP: "); read_input(sys_cfg.ip, 15);
        printf("Mask: "); read_input(sys_cfg.mask, 15);
        printf("GW: "); read_input(sys_cfg.gateway, 15);
    } else {
        sys_cfg.static_ip = false;
    }

    
    printf("\n[6/6] Voltage Calibration\n");
    printf("R1 Value (Top Resistor) Ohms (Current: %.1f): ", sys_cfg.r1_val);
    read_input(buf, 15);
    if (strlen(buf) > 0) sys_cfg.r1_val = atof(buf);

    printf("R2 Value (Bottom Resistor) Ohms (Current: %.1f): ", sys_cfg.r2_val);
    read_input(buf, 15);
    if (strlen(buf) > 0) sys_cfg.r2_val = atof(buf);

    save_config();
    printf("\nSaved. REBOOTING...\n");
    sleep_ms(1000);
    watchdog_reboot(0, 0, 0);
    while(1);
}

void run_config_check() {
    load_config(); 
    printf("\nBooting Nemtek Interface...\n");
    printf("Press 'c' in 3s to Configure...\n");
    for(int i=3; i>0; i--) {
        printf("%d... ", i);
        int c = getchar_timeout_us(1000000);
        if (c == 'c' || c == 'C') run_config_wizard();
    }
    printf("\nStarting App.\n");
}