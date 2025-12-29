#ifndef CONFIG_WIZARD_H
#define CONFIG_WIZARD_H
#define VERSION "1.0.0-beta"
#include <stdint.h>
#include <stdbool.h>
#if __has_include("config.h")
    #include "config.h"
#else
    #include "config_example.h"
#endif

#define FLASH_TARGET_OFFSET (2 * 1024 * 1024 - 4096) 
#define CONFIG_MAGIC        0xDEADBEEF
                            

typedef struct {
    char wifi_ssid[33];
    char wifi_pass[64];
    char mqtt_server[64];
    int  mqtt_port;
    char mqtt_user[33];
    char mqtt_pass[33];
    char master_pin[4];
    
    
    bool static_ip;
    char ip[16];
    char mask[16];
    char gateway[16];
    float r1_val;
    float r2_val;
    
    uint32_t magic; 
} SystemConfig;


extern SystemConfig sys_cfg;

void load_config();
void save_config();
void run_config_check(); 

#endif 