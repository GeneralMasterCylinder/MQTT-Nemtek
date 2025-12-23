//Customize this file.

#define DEFAULT_WIFI_SSID       "My Wifi"
#define DEFAULT_WIFI_PASS       "My Wifi Password"
#define DEFAULT_MQTT_IP  "192.168.1.1"  //change to your Home assistant with mosquito or othe MQTT broker.  
#define DEFAULT_MQTT_USER       "My MQTT User"      
#define DEFAULT_MQTT_PASS       "My MQTT Password"  
#define MQTT_CLIENT_ID  "pico_nemtek"
#define periodic_publish_interval 100 //Adjust how often you want an update if no change.
#define DEFAULT_MQTT_PORT   1883
#define DEFAULT_USE_STATIC  true
#define DEFAULT_STATIC_IP       "192.168.1.1" //Change to a free static IP
#define DEFAULT_STATIC_MASK     "255.255.255.0" //Match to your network subnet mask
#define STATICDEFAULT_STATIC_GW       "192.168.1.1"  //Match to your router/gateway.
#define UART_ID         uart0
#define BAUD_RATE          2400
#define TX_PIN             0
#define RX_PIN             1
#define RELAY_PIN          15
#define WDT_TIMEOUT_MS     20000000
#define COMMS_FAIL_TIMEOUT 60000000
#define DEFAULT_R1_VAL 38480.0f
#define DEFAULT_R2_VAL 9827.0f
#define DEFAULT_MASTER_PIN  "1234"
