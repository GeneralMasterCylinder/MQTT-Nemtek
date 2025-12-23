//Customize this file.

#define WIFI_SSID       "My Wifi"
#define WIFI_PASS       "My Wifi Password"
#define MQTT_BROKER_IP  "192.168.1.1"  //change to your Home assistant with mosquito or othe MQTT broker.  
#define MQTT_USER       "My MQTT User"      
#define MQTT_PASS       "My MQTT Password"  
#define MQTT_CLIENT_ID  "pico_nemtek"
#define periodic_publish_interval 100 //Adjust how often you want an update if no change.
#define STATIC_IP       "192.168.1.1" //Change to a free static IP
#define STATIC_MASK     "255.255.255.0" //Match to your network subnet mask
#define STATIC_GW       "192.168.1.1"  //Match to your router/gateway.
#define UART_ID         uart0
#define BAUD_RATE          2400
#define TX_PIN             0
#define RX_PIN             1
#define RELAY_PIN          15
#define WDT_TIMEOUT_MS     20000000
#define COMMS_FAIL_TIMEOUT 60000000
#define R1_VAL 38480.0f     //Match to your exact voltage divider resistor values (see readme)
#define R2_VAL 9827.0f
