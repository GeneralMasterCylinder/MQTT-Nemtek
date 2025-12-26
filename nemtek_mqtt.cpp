#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/netif.h"
#include "hardware/adc.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include <vector>

#include "config_wizard.h"

static float VOLTAGE_DIVIDER_RATIO = 4.9;


const float ADC_CONVERSION = 3.3f / 4096.0f;

// --- FULL STATE STRUCT (21 Flags + Raw Logger) ---
typedef struct {
    // --- DECODED FLAGS (For Binary Sensors) ---
    bool hv_good;           // B3 0x08
    bool hv_check;          // B3 0x20
    bool hv_bad;            // B3 0x00 (Derived)

    bool fence_alarm_volts; // B4 0x05
    bool fence_low_power;   // B4 0x10

    bool alarm_active;      // B5 0x02
    bool tamper;            // B5 0x20

    bool gate_alarm;        // B6 0x04
    bool batt_hist;         // B6 0x10
    bool batt_good;         // B6 0x40

    bool fence_off;         // B7 0x08
    bool fence_volts_low;   // B7 0x02

    bool alarm_bypass;      // B8 0x01
    bool silent_alarm;      // B8 0x10
    bool gate_immediate;    // B8 0x40

    bool gate_bypass;       // B9 0x02
    bool batt_empty;        // B9 0x08
    bool batt_low;          // B9 0x20

    bool buzzer_active;     // B10 0x02
    bool mains_present;     // Derived

    uint8_t raw[8]; 
} NemtekState;

queue_t state_queue;
volatile uint32_t core1_heartbeat = 0;
static mqtt_client_t* mqtt_client;
bool mqtt_connected = false;
bool mqtt_connecting = false;
static uint8_t rx_raw[32]; 
static int rx_idx = 0;
queue_t key_queue;            
const uint JUMPER_PIN = 22;   
uint8_t my_keypad_addr = 0x48; 
bool keypad_tx_enabled = false; 
NemtekState last_known_state = {0};
const char ret = 0x0B;
const char star = 0x0A;
#ifndef NDEBUG
    char boot_reason[64] = "Clean Boot";
#endif

void perform_address_discovery() {
    printf("Core 1: Starting Keypad Discovery (5s)...\n");
    
    bool addr_40_taken = false;
    bool addr_48_taken = false;
    
    uint64_t start_time = time_us_64();
    uint8_t buf[32]; 
    int idx = 0;
    uint8_t last_polled_addr = 0;
    uint64_t last_packet_time = 0;
    uint64_t gap = 0;
   
    while (time_us_64() - start_time < 5000000) {
        core1_heartbeat++;
        if (uart_is_readable(UART_ID)) {
            uint8_t ch = uart_getc(UART_ID);
            
            if (idx < 31) buf[idx++] = ch;

            if (idx >= 11 && buf[0] == 0xFF) {
                last_polled_addr = buf[10] & (~0x2);  // Buzzer bit may cause trouble.
                last_packet_time = time_us_64();
                idx = 0; 
                continue; 
            }
            if (buf[0] == 0xFF) {
               
            }
            gap = time_us_64() - last_packet_time;
            if (ch == 0xFE && (gap < 100000)) {
                #ifndef NDEBUG
                    printf("Gap (keypad)= %i uS\n",(int)gap);
                #endif
                if (last_polled_addr == 0x40) addr_40_taken = true;
                if (last_polled_addr == 0x48) addr_48_taken = true;
                
                last_polled_addr = 0; 
            }
            
            if (idx > 15) idx = 0;
        }
        if (uart_get_hw(UART_ID)->rsr & 0x0F) {
                uart_get_hw(UART_ID)->rsr = 0; 
            }
        sleep_us(100);

    }

    
    if (addr_40_taken && addr_48_taken) {
        printf("Both Keypad Addresses (1 & 2) are occupied!\n");
        printf("Virtual Keypad remains disabled to prevent collisions.\n");
        
    } 
    else if (addr_40_taken) {
        printf("Keypad 1 (0x40) detected. Switching to Keypad 2 (0x48).\n");
        my_keypad_addr = 0x48;
        keypad_tx_enabled = true;
    } 
    else if (addr_48_taken) {
        printf("Keypad 2 (0x48) detected. Switching to Keypad 1 (0x40).\n");
        my_keypad_addr = 0x40;
        keypad_tx_enabled = true;
    } 
    else {
        if (!gpio_get(JUMPER_PIN)) my_keypad_addr = 0x40; 
        else my_keypad_addr = 0x48; // Default
        printf("No existing keypads detected. Defaulting to jumper State. (%02X)\n", my_keypad_addr);
        keypad_tx_enabled = true;
        
    }
}

// --- CORE 1: DECODER ---
void core1_entry() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_EVEN);
    uart_set_fifo_enabled(UART_ID, true);
    
    gpio_set_inover(RX_PIN, GPIO_OVERRIDE_INVERT);
    gpio_set_outover(TX_PIN, GPIO_OVERRIDE_INVERT);
    gpio_pull_up(RX_PIN);
    
    perform_address_discovery();
    uint64_t last_byte_time = 0;
    NemtekState last_pushed_state = {0};
    bool periodic_publish_flag = true;
    int periodic_publish = 0;
    int rx_idx =0;
    
    printf("[CORE 1] UART Handling.\n");
    while (uart_is_readable(UART_ID)) uart_getc(UART_ID);
    while (true) {
        core1_heartbeat++;
        #ifndef NDEBUG
            watchdog_hw->scratch[4] = 1;
        #endif
        if (uart_is_readable(UART_ID)) {
            #ifndef NDEBUG
            watchdog_hw->scratch[4] = 2;
            #endif
            uint8_t ch = uart_getc(UART_ID);
            
            if ((time_us_64() - last_byte_time > 50000) && (rx_idx !=0)) rx_idx =0;
            last_byte_time = time_us_64();
            if (rx_idx < 32) {
                rx_raw[rx_idx++] = ch;
            } else {
                rx_idx = 0; 
                continue;
            }
            if (rx_idx > 0 && rx_raw[0] != 0xFF) {
                rx_idx = 0;
                continue;
            }

            if (rx_idx > 1 && rx_raw[1] != 0x09) {
                rx_idx = 0;
                continue;
            }
            
            if (rx_idx >= 11) {
                #ifndef NDEBUG
                    watchdog_hw->scratch[4] = 3;
                #endif
                NemtekState s = {0};
                uint8_t poll_byte = rx_raw[10] & (~0x02);
                sleep_us(5000);

                if (keypad_tx_enabled && poll_byte == my_keypad_addr) {
                    
                    
                    uint8_t response[3] = { 0xFE, 0x06, 0x7F }; 
                    if (queue_get_level(&key_queue) > 0) {
                         uint8_t key;
                        if (queue_try_remove(&key_queue, &key)) response[2] = key;
                    }
                    uart_write_blocking(UART_ID, response, 3);
                }
                
                for(int i=0; i<8; i++) s.raw[i] = rx_raw[3+i];

                // Byte 3
                s.hv_good           = (rx_raw[3] & 0x08);
                s.hv_check          = (rx_raw[3] & 0x20);
                s.hv_bad            = !(s.hv_good || s.hv_check);

                // Byte 4
                s.fence_alarm_volts = (rx_raw[4] & 0x05);
                s.fence_low_power   = (rx_raw[4] & 0x10);

                // Byte 5
                s.alarm_active      = (rx_raw[5] & 0x02);
                s.tamper            = (rx_raw[5] & 0x20);

                // Byte 6
                s.gate_alarm        = (rx_raw[6] & 0x04);
                s.batt_hist         = (rx_raw[6] & 0x10);
                s.batt_good         = (rx_raw[6] & 0x40);

                // Byte 7
                s.fence_volts_low   = (rx_raw[7] & 0x02);
                s.fence_off         = (rx_raw[7] & 0x08);

                // Byte 8
                s.alarm_bypass      = (rx_raw[8] & 0x01);
                s.silent_alarm      = (rx_raw[8] & 0x10);
                s.gate_immediate    = (rx_raw[8] & 0x40);

                // Byte 9
                s.gate_bypass       = (rx_raw[9] & 0x02);
                s.batt_empty        = (rx_raw[9] & 0x08);
                s.batt_low          = (rx_raw[9] & 0x20);

                // Byte 10 
                s.buzzer_active     = (rx_raw[10] & 0x02);

                s.mains_present     = !(s.batt_good || s.batt_low || s.batt_empty);
                
                periodic_publish++;
                if (periodic_publish>periodic_publish_interval)
                {
                    periodic_publish=0;
                    periodic_publish_flag=true;
                }
                if (queue_get_level(&state_queue) > 3)
                {
                    periodic_publish_flag =false; 
                    
                }
                bool state_changed = false;

                s.raw[7] = s.raw[7] & (~0x08); //Simply mask keypad address.
                if (memcmp(s.raw, last_pushed_state.raw, 8) != 0) {
                    state_changed = true;
                }

                if (periodic_publish_flag || state_changed) {
                    if (queue_try_add(&state_queue, &s)) {
                        last_pushed_state = s;
                        periodic_publish_flag = false;
                        periodic_publish = 0;
                        printf("Change Or Heartbeat Publish\n");
                    }
                }
                #ifndef NDEBUG
                    watchdog_hw->scratch[4] = 3;
                #endif
                rx_idx =0;
            }
        } else {
            if (uart_get_hw(UART_ID)->rsr & 0x0F) {
                uart_get_hw(UART_ID)->rsr = 0; 
            }
            #ifndef NDEBUG
                watchdog_hw->scratch[4] = 4;
            #endif
            sleep_us(50);
        }
    }

}
void init_keypad_logic() {
    queue_init(&key_queue, sizeof(uint8_t), 64);
    gpio_init(JUMPER_PIN);
    gpio_set_dir(JUMPER_PIN, GPIO_IN);
    gpio_pull_up(JUMPER_PIN); 

    
}

float get_core_temperature() {
    adc_select_input(4);
    sleep_ms(20);
    uint16_t raw = adc_read();
    const float conversion_factor = 3.3f / (1 << 12);
    float voltage = raw * conversion_factor;
    float temperature = 27.0f - (voltage - 0.706f) / 0.001721f;
    return temperature;
}

float get_supply_voltage() {
    adc_select_input(0); 
    sleep_ms(20);
    uint16_t raw = adc_read();
    return (float)raw * ADC_CONVERSION * VOLTAGE_DIVIDER_RATIO;
}

void init_adc_sensors() {
    adc_init();
    adc_gpio_init(26);
    adc_set_temp_sensor_enabled(true);
    gpio_disable_pulls(26); 
}

static char current_topic[128];


static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    strncpy(current_topic, topic, sizeof(current_topic)-1);
    
    current_topic[sizeof(current_topic)-1] = '\0';
    printf("In mqtt_incoming)publish_cb Topic=%s\n", current_topic);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    char payload[64];
    if (len >= sizeof(payload)) len = sizeof(payload) - 1; 
    memcpy(payload, data, len);
    payload[len] = '\0'; 

    printf("MQTT RX: Topic='%s' Payload='%s'\n", current_topic, payload);

    if (strstr(current_topic, "/cmd/arm")) {
        
        bool req_arm = (strcmp(payload, "ARM_AWAY") == 0) || (strcmp(payload, "ARM_HOME") == 0);
        bool req_disarm = (strcmp(payload, "DISARM") == 0);

        const char* pin = sys_cfg.master_pin; 
        
        if (req_arm && last_known_state.fence_off) { 
            printf("CMD: Arming System...\n");
            for(int i=0; i<4; i++) queue_add_blocking(&key_queue, &pin[i]);
            uint8_t enter = 0x0B; queue_add_blocking(&key_queue, &enter);
        } 
        else if (req_disarm && !last_known_state.fence_off) {
            printf("CMD: Disarming System...\n");
            for(int i=0; i<4; i++) queue_add_blocking(&key_queue, &pin[i]);
            uint8_t enter = 0x0B; queue_add_blocking(&key_queue, &enter);
        }
        else {
            printf("CMD: Ignored (State already matches request)\n");
        }
    }
    
    else if (strstr(current_topic, "/cmd/keys")) {
        
        for(int i=0; i<len; i++) {
            if (payload[i]>= '0' && payload[i]<='9') queue_add_blocking(&key_queue, &payload[i]-'0');
            if (payload[i] == '*') queue_add_blocking(&key_queue, &star);
            if (payload[i] == '#') queue_add_blocking(&key_queue, &ret);
        }
    }
}

void mqtt_pub_safe(const char* topic, const char* payload) {
    mqtt_publish(mqtt_client, topic, payload, strlen(payload), 1, 1, NULL, NULL);
}

void pub_one_config(const char* key, const char* name, const char* cls, const char* on_v, const char* off_v) {
    char p[512];
    const char* dev = "\"dev\":{\"ids\":[\"nemtek_pico\"],\"name\":\"Nemtek Energizer\",\"mf\":\"Nemtek\",\"sw\":\"v3.1\"}";
    snprintf(p, sizeof(p), "{\"name\":\"%s\",\"stat_t\":\"nemtek/status\",\"val_tpl\":\"{{value_json.%s}}\",\"pl_on\":\"%s\",\"pl_off\":\"%s\",\"dev_cla\":\"%s\",\"uniq_id\":\"nt_%s\",%s}",
             name, key, on_v, off_v, cls, key, dev);
    char topic[128];
    snprintf(topic, sizeof(topic), "homeassistant/binary_sensor/nt_%s/config", key);
    mqtt_pub_safe(topic, p);
    cyw43_arch_poll();
    sleep_ms(100); 
}

void publish_discovery() {
    if (!mqtt_connected) return;
    printf("Publishing Config (Sequential)...\n");

    pub_one_config("alarm_active",    "Fence Alarm",       "problem",       "ON", "OFF");
    pub_one_config("fence_volts_low", "Fence Voltage Low", "problem",       "ON", "OFF");
    pub_one_config("tamper",          "Tamper",            "tamper",        "ON", "OFF");
    pub_one_config("gate_alarm",      "Gate Alarm",        "garage_door",   "ON", "OFF");
    pub_one_config("batt_empty",      "Battery Empty",     "battery",       "ON", "OFF");
    pub_one_config("batt_low",        "Battery Low",       "battery",       "ON", "OFF");
    pub_one_config("batt_hist",       "Battery History",   "battery",       "ON", "OFF");

    pub_one_config("mains",           "Mains Power",       "power",         "ON", "OFF");
    pub_one_config("batt_good",       "Battery Good",      "battery",       "ON", "OFF");
    pub_one_config("low_power",       "Low Power Mode",    "running",       "ON", "OFF");
    pub_one_config("silent",          "Silent Mode",       "running",       "ON", "OFF");
    pub_one_config("alarm_bypass",    "Alarm Bypassed",    "running",        "ON", "OFF");
    pub_one_config("gate_bypass",     "Gate Bypassed",     "running",        "ON", "OFF");
    pub_one_config("gate_immed",      "Gate Immediate",    "running",       "ON", "OFF");
    pub_one_config("buzzer",          "Keypad Buzzer",     "sound",         "ON", "OFF");
    pub_one_config("comms_good",      "Comms Good",        "connectivity",  "ON", "OFF");

    pub_one_config("hv_good",         "HV Good",           "running",       "ON", "OFF");
    pub_one_config("hv_check",        "HV Check",          "problem",       "ON", "OFF");
    pub_one_config("hv_bad",          "HV Bad",            "problem",       "ON", "OFF");
    pub_one_config("fence_off",       "Fence Off",         "running",       "ON", "OFF");

    char p[1024];
    const char* dev = "\"dev\":{\"ids\":[\"nemtek_pico\"],\"name\":\"Nemtek Energizer\",\"mf\":\"Nemtek\",\"sw\":\"3.1\"}";
    snprintf(p, sizeof(p), 
        "{"
        "\"name\":\"Nemtek Control\","
        "\"stat_t\":\"nemtek/status\","
        "\"val_tpl\":\"{%% if value_json.alarm_active == 'ON' %%}triggered{%% elif value_json.fence_off == 'ON' %%}disarmed{%% else %%}armed_away{%% endif %%}\","
        "\"cmd_t\":\"nemtek/control/cmd/arm\","
        "\"code_arm_required\":false," 
        "\"uniq_id\":\"nt_alarm_panel\","
        "%s" 
        "}", dev);

    mqtt_pub_safe("homeassistant/alarm_control_panel/nt_alarm/config", p);
    cyw43_arch_poll();
    sleep_ms(50);
    

    sleep_ms(50);
    cyw43_arch_poll();
    
    snprintf(p, sizeof(p), "{\"name\":\"Raw Packet\",\"stat_t\":\"nemtek/status\",\"val_tpl\":\"{{value_json.raw}}\",\"icon\":\"mdi:code-braces\",\"uniq_id\":\"nt_raw\",%s}", dev);
    mqtt_pub_safe("homeassistant/sensor/nt_raw/config", p);
    sleep_ms(50);
    cyw43_arch_poll();

    char p_batt[512];
    snprintf(p_batt, sizeof(p_batt), 
        "{"
        "\"name\":\"Supply Voltage\","
        "\"stat_t\":\"nemtek/status\","
        "\"val_tpl\":\"{{value_json.batt_v}}\"," 
        "\"unit_of_meas\":\"V\","
        "\"dev_cla\":\"voltage\","
        "\"stat_cla\":\"measurement\"," 
        "\"uniq_id\":\"nt_supply_v\","
        "%s"
        "}", dev);
        
    mqtt_pub_safe("homeassistant/sensor/nt_supply_v/config", p_batt);
    cyw43_arch_poll();
    sleep_ms(50);

    char p_temp[512];
    snprintf(p_temp, sizeof(p_temp), 
        "{"
        "\"name\":\"Device Temperature\","
        "\"stat_t\":\"nemtek/status\","
        "\"val_tpl\":\"{{value_json.temp}}\","
        "\"unit_of_meas\":\"°C\","
        "\"dev_cla\":gpio_init\"temperature\","
        "\"uniq_id\":\"nt_temp\","
        "%s" 
        "}", dev);
        
    mqtt_pub_safe("homeassistant/sensor/nt_temp/config", p_temp);
    cyw43_arch_poll();
    sleep_ms(50);
    char p_boot[512];
    snprintf(p_boot, sizeof(p_boot), 
        "{"
        "\"name\":\"Last Boot Reason\","
        "\"stat_t\":\"nemtek/debug/boot_reason\","
        "\"uniq_id\":\"nt_boot_reason\","
        "\"icon\":\"mdi:information-outline\","
        "\"ent_cat\":\"diagnostic\"," 
        "%s"
        "}", dev);

    mqtt_pub_safe("homeassistant/sensor/nt_boot/config", p_boot);
    cyw43_arch_poll();
     sleep_ms(50);
}

// ***TODO*** Review how to set the status to "unavailable"
void publish_availability_offline() {
    if (!mqtt_connected) return;
    
    float temp_c = get_core_temperature();
    float bat_voltage = get_supply_voltage();
    printf("Comms Timeout - Publishing Offline State\n");
    char json[1024];
    snprintf(json, sizeof(json),
        "{"
        "\"temp\":%.1f," 
        "\"batt_v\":%.2f,"
        "\"armed\":\"unknown\"," 
        "\"hv_good\":\"unknown\",\"hv_check\":\"unknown\",\"hv_bad\":\"unknown\","
        "\"fence_alarm_volts\":\"unknown\",\"low_power\":\"unknown\","
        "\"alarm_active\":\"unknown\",\"tamper\":\"unknown\","
        "\"gate_alarm\":\"unknown\",\"batt_hist\":\"unknown\",\"batt_good\":\"unknown\","
        "\"fence_off\":\"unknown\",\"fence_volts_low\":\"unknown\","
        "\"alarm_bypass\":\"unknown\",\"silent\":\"unknown\",\"gate_immed\":\"unknown\","
        "\"gate_bypass\":\"unknown\",\"batt_empty\":\"unknown\",\"batt_low\":\"unknown\","
        "\"buzzer\":\"unknown\",\"mains\":\"unknown\",\"comms_good\":\"OFF\""
        "}",
        temp_c, bat_voltage
        
    );

    mqtt_pub_safe("nemtek/status", json);
    
    
}
void publish_state(NemtekState s) {
    if (!mqtt_connected) return;
    char json[1024];
    float temp_c = get_core_temperature();
    float bat_voltage = get_supply_voltage();
    snprintf(json, sizeof(json),
        "{"
        "\"raw\":\"%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\"," 
        "\"temp\":%.1f,"  
        "\"batt_v\":%.2f,"
        "\"armed\":\"%s\"," 
        "\"hv_good\":\"%s\",\"hv_check\":\"%s\",\"hv_bad\":\"%s\","
        "\"fence_alarm_volts\":\"%s\",\"low_power\":\"%s\","
        "\"alarm_active\":\"%s\",\"tamper\":\"%s\","
        "\"gate_alarm\":\"%s\",\"batt_hist\":\"%s\",\"batt_good\":\"%s\","
        "\"fence_off\":\"%s\",\"fence_volts_low\":\"%s\","
        "\"alarm_bypass\":\"%s\",\"silent\":\"%s\",\"gate_immed\":\"%s\","
        "\"gate_bypass\":\"%s\",\"batt_empty\":\"%s\",\"batt_low\":\"%s\","
        "\"buzzer\":\"%s\",\"mains\":\"%s\",\"comms_good\":\"ON\""
        "}",
        
        s.raw[0], s.raw[1], s.raw[2], s.raw[3], s.raw[4], s.raw[5], s.raw[6], s.raw[7],
        temp_c, bat_voltage,
        (!s.fence_off) ? "ON" : "OFF",
        s.hv_good?"ON":"OFF", s.hv_check?"ON":"OFF", s.hv_bad?"ON":"OFF",
        s.fence_alarm_volts?"ON":"OFF", s.fence_low_power?"ON":"OFF",
        s.alarm_active?"ON":"OFF", s.tamper?"ON":"OFF",
        s.gate_alarm?"ON":"OFF", s.batt_hist?"ON":"OFF", s.batt_good?"ON":"OFF",
        s.fence_off?"ON":"OFF", s.fence_volts_low?"ON":"OFF",
        s.alarm_bypass?"ON":"OFF", s.silent_alarm?"ON":"OFF", s.gate_immediate?"ON":"OFF",
        s.gate_bypass?"ON":"OFF", s.batt_empty?"ON":"OFF", s.batt_low?"ON":"OFF",
        s.buzzer_active?"ON":"OFF", s.mains_present?"ON":"OFF"
    );

    mqtt_pub_safe("nemtek/status", json);
    
    
}

static void mqtt_sub_request_cb(void *arg, err_t result) {
    printf("MQTT: Subscription result: %d\n", result);
}
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    mqtt_connecting = false;
    mqtt_connected = (status == MQTT_CONNECT_ACCEPTED);
    if(mqtt_connected) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        err_t err = mqtt_sub_unsub(client, "nemtek/control/#", 0, mqtt_sub_request_cb, NULL, 1);
        if(err != ERR_OK) {
            printf("MQTT: Subscription Error: %d\n", err);
        }
    }
}

void connect_mqtt() {
    if (mqtt_connecting) return;
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = "pico_nemtek";
    ci.client_user = sys_cfg.mqtt_user;
    ci.client_pass = sys_cfg.mqtt_pass;
    ci.keep_alive = 60;
    ci.will_topic = "nemtek/status";
    ci.will_msg = "{\"comms_good\":\"OFF\",\"armed\":\"unknown\",\"batt_v\":\"unknown\"}"; // Minimal JSON
    ci.will_qos = 1;
    ci.will_retain = 0;
    ip_addr_t broker_ip;
    ip4addr_aton(sys_cfg.mqtt_server, &broker_ip);
    
    mqtt_connecting = true;
    mqtt_client_connect(mqtt_client, &broker_ip, 1883, mqtt_connection_cb, NULL, &ci);
}

void setup_static_ip() {
    struct netif *n = &cyw43_state.netif[CYW43_ITF_STA];
    ip_addr_t ip, mask, gw;
    ip4addr_aton(sys_cfg.ip, &ip);
    ip4addr_aton(sys_cfg.mask, &mask);
    ip4addr_aton(sys_cfg.gateway, &gw);
    netif_set_addr(n, &ip, &mask, &gw);
    netif_set_up(n);
}

int main() {
    init_keypad_logic();
    #ifndef NDEBUG
    if (watchdog_caused_reboot()) snprintf(boot_reason, sizeof(boot_reason), "Core1: %d | Core0: %d\n", 
           watchdog_hw->scratch[4], 
           watchdog_hw->scratch[5]);
    #endif
    stdio_init_all();
    sleep_ms(2000);
    run_config_check();
    if (sys_cfg.r2_val >0)
        VOLTAGE_DIVIDER_RATIO =((sys_cfg.r1_val +sys_cfg.r2_val)/ sys_cfg.r2_val);
     
    printf("Nemtek Druid MQTT Client\n");
    init_adc_sensors() ;

    gpio_init(RELAY_PIN); gpio_set_dir(RELAY_PIN, GPIO_OUT); gpio_put(RELAY_PIN, 0);

    queue_init(&state_queue, sizeof(NemtekState), 100);
    multicore_launch_core1(core1_entry);

    if (cyw43_arch_init()) { printf("HW Fail\n"); return 1; }
    cyw43_arch_enable_sta_mode();
    struct netif *n = &cyw43_state.netif[CYW43_ITF_STA];

    if (sys_cfg.static_ip) {
        dhcp_stop(n);
        dhcp_release(n); 

        ip4_addr_t ip, mask, gw;
        ip4addr_aton(sys_cfg.ip, &ip);
        ip4addr_aton(sys_cfg.mask, &mask);
        ip4addr_aton(sys_cfg.gateway, &gw);

        netif_set_addr(n, &ip, &mask, &gw);
        
        netif_set_up(n);
        
        printf("Network: Static IP Configured: %s\n", sys_cfg.ip);
    } else {
        dhcp_start(n);
        printf("Network: using DHCP\n");
    }

    mqtt_client = mqtt_client_new();
    #ifndef NDEBUG
    watchdog_hw->scratch[5] = 1;
    #endif
    uint64_t last_valid_packet_time = 0;
    uint32_t last_hb = 0;
    bool prev_connected = false;
    NemtekState current_state = {0};
    printf("Starting MQTT Loop\n");
    watchdog_enable(8000,1);
    while (true) {
        #ifndef NDEBUG
        watchdog_hw->scratch[5] = 2;
        #endif
        if(core1_heartbeat != last_hb)  last_hb = core1_heartbeat; watchdog_update(); 
       
        if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) {
            printf("(Re)Connecting WiFi\n");
            cyw43_arch_wifi_connect_async(sys_cfg.wifi_ssid, sys_cfg.wifi_pass, CYW43_AUTH_WPA2_AES_PSK);
            for (int i=0;i<7;i++)
            {
                sleep_ms(1000);
                if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) 
                {
                    if(core1_heartbeat != last_hb)  last_hb = core1_heartbeat; watchdog_update(); 
                 
                }
                else
                {
                    break;
                }
            } 
            continue;           

        } 
        if (!mqtt_connected && !mqtt_connecting) {
            printf("(Re)Connecting MQTT\n");
            connect_mqtt();
            sleep_ms(1000);
            continue;
        }
       
        if (mqtt_connected) {
            if (!prev_connected) {
                printf("MQTT Success.\n");
                publish_discovery();
                sleep_ms(1500);
                mqtt_set_inpub_callback(mqtt_client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);
                if(core1_heartbeat != last_hb)  last_hb = core1_heartbeat; watchdog_update(); 
                
            }
            
        }

        if (queue_try_remove(&state_queue, &current_state)) {
            last_known_state = current_state;
            gpio_put(RELAY_PIN, current_state.alarm_active || current_state.gate_alarm);
            if (mqtt_connected) {
                publish_state(current_state);
                if(core1_heartbeat != last_hb)  last_hb = core1_heartbeat; watchdog_update(); 
                last_valid_packet_time = time_us_64();
                #ifndef NDEBUG
                mqtt_pub_safe("nemtek/debug/boot_reason", boot_reason);
                #endif
                watchdog_hw->scratch[5] = 3;
            }
        }
        if (time_us_64() - last_valid_packet_time > COMMS_FAIL_TIMEOUT)
        {
            publish_availability_offline();
            #ifndef NDEBUG
            watchdog_hw->scratch[5] = 4;
            #endif
        }

        
        
        prev_connected = mqtt_connected;
        if(core1_heartbeat != last_hb)  last_hb = core1_heartbeat; watchdog_update(); 
        cyw43_arch_poll();
        #ifndef NDEBUG
        watchdog_hw->scratch[5] = 5;
        #endif
        sleep_ms(50);
    }
}