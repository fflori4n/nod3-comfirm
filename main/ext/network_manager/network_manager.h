#pragma once

#define NETWORK_MANAGER_TEST_STR "hello network manager 1.0"

#include <algorithm>
#include <mutex>
#include <cstring>

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include <cstring>
#include <string>

namespace NETWORK
{
    class Wlan{

        public:
            enum class tWlanState
            {
                wlanState_notInitialised,
                wlanState_initCompleted,
                wlanState_rdyToConnect,
                wlanState_connecting,
                wlanState_waitingForIP,
                wlanState_connected,
                wlanState_connectedAndStable,
                wlanState_disconnected,
                wlanState_error
            };

            enum class wifi_power_save_e{

                psave_disabled = WIFI_PS_NONE,
                psave_default = WIFI_PS_MIN_MODEM,
                psave_maximum = WIFI_PS_MAX_MODEM
            };

            typedef struct
            {
                unsigned frame_ctrl : 16;
                unsigned duration_id : 16;
                uint8_t addr1[6]; /* receiver address */
                uint8_t addr2[6]; /* sender address */
                uint8_t addr3[6]; /* filtering address */
                unsigned sequence_ctrl : 16;
                uint8_t addr4[6]; /* optional */
            } wifi_ieee80211_mac_hdr_t;

            typedef struct
            {
                wifi_ieee80211_mac_hdr_t hdr;
                uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
            } wifi_ieee80211_packet_t;

            static const uint16_t wlan_scan_period_sec{60};

            static constexpr char* network_sensor_template = "\"w_rssi\":%.2f,\"w_dest\":%.2f,\"w_con_tm\":%.2f,\"w_recon\":%d,\"w_condev\":%d,\"w_alldev\":%d,\"w_chdev\":%d,\"w_apssid\":\"%s\"";

        private:
            char macAddressCStr[12+1]{};
            static tWlanState _wlanIfaceState;
            static wifi_init_config_t _wifiInitCfg;
            static wifi_config_t _wifiConfig;

            esp_netif_t* p_netif;
        public:

            static constexpr uint64_t espressif_macs[] = {
                0x048308,
0x083A8D,0x083AF2,0x08A6F7,0x08B61F,0x08D1F9,0x08F9E0,0x0C8B95,0x0CB815,0x0CDC7E,0x10003B,0x10061C,0x10521C,0x1091A8,0x1097BD,0x142B2F,0x188B0E,0x18FE34,0x1C6920,0x1C9DC2,0x2043A8,0x240AC4,0x244CAB,0x24587C,0x2462AB,0x246F28,0x24A160,0x24B2DE,0x24D7EB,0x24DCC3,0x24EC4A,0x28372F,0x2C3AE8,0x2CBCBB,0x2CF432,0x3030F9,0x308398,0x30AEA4,0x30C6F7,0x30C922,0x30EDA0,0x345F45,0x348518,0x34865D,0x349454,0x34987A,0x34AB95,0x34B472,0x34B7DA,0x34CDB0,0x38182B,0x3C6105,0x3C71BF,0x3C8427,0x3C8A1F,0x3CE90E,0x4022D8,0x404CCA,0x409151,0x40F520,0x441793,0x4827E2,0x4831B7,0x483FDA,0x485519,0x48CA43,0x48E729,0x4C11AE,0x4C7525,0x4CEBD6,0x500291,0x543204,0x5443B2,0x545AA6,0x588C81,0x58BF25,0x58CF79,0x5C013B,0x5CCF7F,0x600194,0x6055F9,0x64B708,0x64E833,0x686725,0x68B6B3,0x68C63A,0x6CB456,0x70039F,0x70041D,0x70B8F6,0x744DBD,0x782184,0x78421C,0x78E36D,0x78EE4C,0x7C2C67,0x7C7398,0x7C87CE,0x7C9EBD,0x7CDFA1,0x80646F,0x806599,0x807D3A,0x840D8E,0x84CCA8,0x84F3EB,0x84F703,0x84FCE6,0x8813BF,0x8C4B14,0x8C4F00,0x8CAAB5,0x8CBFEA,0x8CCE4E,0x901506,0x90380C,0x9097D5,0x943CC6,0x9454C5,0x94A990,0x94B555,0x94B97E,0x94E686,0x983DAE,0x98CDAC,0x98F4AB,0x9C9C1F,0x9C9E6E,0xA020A6,0xA0764E,0xA085E3,0xA0A3B3,0xA0B765,0xA0DD6C,0xA47B9D,0xA4CF12,0xA4E57C,0xA8032A,0xA842E3,0xA848FA,0xAC0BFB,0xAC1518,0xAC67B2,0xACD074,0xB08184,0xB0A732,0xB0B21C,0xB43A45,0xB48A0A,0xB4E62D,0xB8D61A,0xB8F009,0xBCDDC2,0xBCFF4D,0xC049EF,0xC04E30,0xC05D89,0xC44F33,0xC45BBE,0xC4D8D5,0xC4DD57,0xC4DEE2,0xC82B96,0xC82E18,0xC8C9A3,0xC8F09E,0xCC50E3,0xCC7B5C,0xCC8DA2,0xCCBA97,0xCCDBA7,0xD0EF76,0xD48AFC,0xD48C49,0xD4D4DA,0xD4F98D,0xD8132A,0xD83BDA,0xD8A01D,0xD8BC38,0xD8BFC0,0xD8F15B,0xDC0675,0xDC1ED5,0xDC4F22,0xDC5475,0xDCDA0C,0xE05A1B,0xE09806,0xE0E2E6,
0xE465B8,0xE4B063,0xE4B323,0xE80690,0xE831CD,0xE868E7,0xE86BEA,0xE89F6D,0xE8DB84,0xEC6260,0xEC64C9,0xEC94CB,0xECC9FF,
0xECDA3B,0xECFABC,0xF008D1,0xF024F9,0xF09E9E,0xF0F5BD,0xF412FA,0xF4650B,0xF4CFA2,0xF8B3B7,0xFCB467,0xFCE8C0,0xFCF5C4,
           };
           typedef struct wlan_manager_stats_t{

                uint32_t connection_ok_sec;
                int rssi;
                uint16_t num_of_reconnect;


                uint8_t num_of_ap;
                uint8_t num_of_sta;
                uint8_t num_of_ap_current_channel;
                uint8_t num_of_sta_current_channel;
                uint8_t num_of_sta_connected_to_ap;
                uint8_t num_of_espressif_devices;

                uint8_t channel;

            } wlan_manager_stats_t;

            typedef struct wlan_scan_devices_t{

                uint64_t device_mac;
                uint64_t associated_mac;
                uint8_t channel;
                bool isAP;
                float rssi;
               /*char* ssid; *//* This is a bad idea */
                std::string ssid;
                uint16_t scan_cycle_since_detect;

                void print_dev_collected(void){
                    if(rssi > -98){
                        ESP_LOGI("\x1b[34m+STA","%llx CH:%d, RSSI:%.0f", device_mac, channel, rssi);
                    }
                    else{
                        ESP_LOGI("\x1b[34m+STA","%llx CH:%d, RSSI:%.0f (indirectly)", device_mac, channel, rssi);
                    }
                }

                char* get_mac_highlight(void){


                    /* TODO: !!! bad practice, memory is not secured when using */

                    char* highlight = "\x1b[0m";
                    if(device_mac == esp_my_mac){
                        highlight = "\x1b[1;34m";
                    }
                    else if(0 != std::strncmp(known_device_by_mac(device_mac),"",1)){
                        highlight= "\x1b[1;36m";
                    }
                    else {

                        for(uint16_t i = 0; i < std::size(espressif_macs); i++){
                            if((device_mac >> 24) == espressif_macs[i]){
                                highlight = "\x1b[1;35m";
                                break;
                            }
                        }                      
                    }

                    return highlight;

                }

                void print_ap(void){

                    ESP_LOGI("\x1b[34m", "---v--  %s%12llx%s - %12llx  CH:%02d RSSI:%.0f ~%2.2fm seen: %dsec ssid: %-33s", 
                    get_mac_highlight(),
                    device_mac,
                    "\x1b[0m",
                    associated_mac,
                    channel,
                    rssi,
                    estimate_ap_dist_from_rssi(rssi),
                    (scan_cycle_since_detect * wlan_scan_period_sec),
                    ssid.c_str()
                    );
                }

                void print_assoc_sta(void){

                    ESP_LOGI("\x1b[34m", "   l--> %s%12llx%s - %12llx  CH:%02d RSSI:%.0f ~%2.2fm seen: %dsec %s", 
                    get_mac_highlight(),
                    device_mac,
                    "\x1b[0m",
                    associated_mac,
                    channel,
                    rssi,
                    estimate_ap_dist_from_rssi(rssi),
                    (scan_cycle_since_detect * wlan_scan_period_sec),
                    known_device_by_mac(device_mac)
                    );
                }

                void print_sta(void){

                    ESP_LOGI("\x1b[34m", "     ~> %s%12llx%s - %12llx  CH:%02d RSSI:%.0f ~%2.2fm seen: %dsec %s", 
                    get_mac_highlight(),
                    device_mac,
                    "\x1b[0m",
                    associated_mac,
                    channel,
                    rssi,
                    estimate_ap_dist_from_rssi(rssi),
                    (scan_cycle_since_detect * wlan_scan_period_sec),
                    known_device_by_mac(device_mac)
                    );
                }

                void print_no_con_sta(void){

                    ESP_LOGI("\x1b[34m", "      x %s%12llx%s - %12llx  CH:%02d RSSI:%.0f ~%.2fm seen: %dsec %s", 
                    get_mac_highlight(),
                    device_mac,
                    "\x1b[0m",
                    associated_mac,
                    channel,
                    rssi,
                    estimate_ap_dist_from_rssi(rssi),
                    (scan_cycle_since_detect * wlan_scan_period_sec),
                    known_device_by_mac(device_mac)
                    );
                }
                

            }wlan_scan_devices_t;

            typedef struct wlan_access_point_id_t{

                const char* ssid;
                const char* passwd;
                uint64_t device_mac;
                uint8_t channel;
                int8_t low_rssi_tresh;
                uint8_t priority;
                uint8_t last_seen;

                float connect_score;

                void print(void){
                    ESP_LOGI(log_label,"wlan_access_point_id_t:: %s, pass: %s, %12llx, CH%d, RSSI dropout: %d, prio: %d, last seen: %d", ssid, passwd, device_mac, channel, low_rssi_tresh, priority, last_seen);
                }
            }wlan_access_point_id_t;

            /* arraylist to hold APs that are preset for connection if present */
            static uint8_t associated_ap_index;
            static constexpr bool enable_switch_to_higher_prio{false}; /* this will allow the ESP to leave a good connection and switch AP if a higher prio AP is present, TODO: could be used for OTA update, with a predefined name or prio*/
            static std::array<wlan_access_point_id_t,2>  access_point_list;

            static constexpr uint8_t maximum_number_of_APs{20};
            static constexpr uint8_t maimum_number_of_STAs{80};
            static std::array<wlan_scan_devices_t, (maximum_number_of_APs + maimum_number_of_STAs)> wlan_device_list;

            static uint64_t esp_my_mac;
            

            RTC_FAST_ATTR static wlan_manager_stats_t statistics;

            uint16_t scan_period_counter = 0;

            static wifi_power_save_e wifi_power_save_mode;
            constexpr static char* log_label{"\x1b[34mNETWORK"};
            constexpr static char* log_label_scan{"\x1b[34mSCAN"};
            static uint64_t macAddress;
            uint32_t wifi_connected_sec;
            uint32_t wifi_trying_to_connect;
            

            static std::mutex wifi_driver_mutex;
            /* Maybe it makes sense to use a separate mutex for callbacks because they are async compred to main task. */
            static std::mutex wifi_driver_callback_mutex;
            

        private:
            static esp_err_t _init(void);
            static void _wifiEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data);
        public:
            Wlan(void);
            ~Wlan(void)                     = default;
            Wlan(const Wlan&)               = default;
            Wlan(Wlan&&)                    = default;
            Wlan& operator=(const Wlan&)    = default;
            Wlan& operator=(Wlan&&)         = default;

            esp_err_t init(void);
            /*void fast_scan(void)*/
            esp_err_t begin(void);
            esp_err_t begin(char* ssid, char* passwd);

            
            template<typename cstr_type> esp_err_t sta_connect(cstr_type* &ssid, cstr_type* &passwd);
            esp_err_t disconnect_power_off(void);

            static esp_err_t is_connected(void) { return (tWlanState::wlanState_connected == _wlanIfaceState ) ? (ESP_OK) : (ESP_FAIL);};   /* note: only reading*/
            
            esp_err_t loadMACAddress(void);
            esp_err_t fastScan(void);

            static void wlan_promiscuous_handler(void *buff, wifi_promiscuous_pkt_type_t type);
            static esp_err_t collect_wifi_device(wlan_scan_devices_t pckt_source_device);
            static esp_err_t add_device_to_dev_list(wlan_scan_devices_t new_device);
            static void print_collected_device(void);
            static uint16_t count_sta_on_channel(uint8_t channel);      
            static uint16_t count_devices_connected_to_AP(uint64_t ap_mac); /* including AP itself */
            static uint16_t count_all_sta(void);
            static uint16_t count_ap_on_channel(uint8_t channel);
            static uint16_t count_all_ap(void);
            static float estimate_ap_dist_from_rssi(int8_t rssi);
            static void calculate_nearby_dev_statistics(wlan_manager_stats_t &statistics, wlan_access_point_id_t &associated_ap);
            esp_err_t set_wifi_connection_target(int8_t ap_index);

            int16_t get_fraction(float value){
                return (abs(((int16_t)(value * 100)) % 100));
            }

    
            esp_err_t get_service_data(char *text_buffer, int16_t text_buffer_size)
            {

                esp_wifi_sta_get_rssi(&statistics.rssi);

                float ap_dist_estimated = estimate_ap_dist_from_rssi(statistics.rssi);
                
                int16_t res = snprintf(text_buffer, text_buffer_size, network_sensor_template,
                                       (float)statistics.rssi,                                  /* w_rssi */
                                       ap_dist_estimated,  
                                       (float)(statistics.connection_ok_sec* 1.0/3600.0),     /* w_con_tm */
                                       (int16_t)(statistics.num_of_reconnect),                                                                                                /* w_recon */
                                       (int16_t)(statistics.num_of_sta_connected_to_ap),                                                            /* w_condev */
                                       (int16_t)(statistics.num_of_ap + statistics.num_of_sta),                                                     /* w_alldev */
                                       (int16_t)(statistics.num_of_ap_current_channel + statistics.num_of_sta_current_channel),                     /* w_chdev */
                                       Wlan::access_point_list[Wlan::associated_ap_index].ssid
                                       );
                if ((res < 0) || (res >= text_buffer_size))
                {
                    return ESP_FAIL;
                }
                return ESP_OK;
            }

            void print_stats(){

            }

            tWlanState get_wlan_state(void) { return _wlanIfaceState; };

            inline bool network_ready(void){ return (tWlanState::wlanState_connected == _wlanIfaceState) ? true : false; };

            constexpr char* getMACAddressCStr(void){ return macAddressCStr;};

            static const char* ip_event_to_string(ip_event_t key)
            {
                constexpr std::array<std::pair<ip_event_t, const char *>, 8> event_to_str_map = {{
                    {IP_EVENT_STA_GOT_IP, "STA_GOT_IP"},
                    {IP_EVENT_STA_LOST_IP, "\x1b[31mSTA_LOST_IP"},
                    {IP_EVENT_GOT_IP6, "GOT_IP6"},
                    {IP_EVENT_ETH_GOT_IP, "ETH_GOT_IP"},
                    {IP_EVENT_ETH_LOST_IP, "ETH_LOST_IP"},
                    {IP_EVENT_PPP_GOT_IP, "PPP_GOT_IP"},
                    {IP_EVENT_PPP_LOST_IP, "PPP_LOST_IP"},
                    {IP_EVENT_TX_RX, "RX_TX"}
                }};

                for (auto &pair : event_to_str_map)
                {
                    if (pair.first == key)
                    {
                        return pair.second;
                    }
                }

                return "UNDEFINED";
            }

            static const char* wifi_event_to_string(wifi_event_t key)
            {
                constexpr std::array<std::pair<wifi_event_t, const char *>, 45> event_to_str_map = {{

                    {WIFI_EVENT_WIFI_READY, "WIFI_READY"},
                    {WIFI_EVENT_SCAN_DONE, "SCAN_FINISHED"},
                    {WIFI_EVENT_STA_START, "STA_START"},
                    {WIFI_EVENT_STA_STOP, "STA_STOP"},
                    {WIFI_EVENT_STA_CONNECTED, "STA_CONNECTED"},
                    {WIFI_EVENT_STA_DISCONNECTED, "\x1b[31mSTA_DISCONNECTED"},
                    {WIFI_EVENT_STA_AUTHMODE_CHANGE, "STA_AUTHMODE_CHANGE"},

                    {WIFI_EVENT_STA_WPS_ER_SUCCESS, "WPS_ER_SUCCESS"},
                    {WIFI_EVENT_STA_WPS_ER_FAILED, "WPS_ER_FAILED"},
                    {WIFI_EVENT_STA_WPS_ER_TIMEOUT, "WPS_ER_TIMEOUT"},
                    {WIFI_EVENT_STA_WPS_ER_PIN, "WPS_ER_PIN"},
                    {WIFI_EVENT_STA_WPS_ER_PBC_OVERLAP, "WPS_ER_PBC_OVERLAP"},

                    {WIFI_EVENT_AP_START, "AP_START"},
                    {WIFI_EVENT_AP_STOP, "AP_STOP"},
                    {WIFI_EVENT_AP_STACONNECTED, "AP_STADEV_CONNECT"},
                    {WIFI_EVENT_AP_STADISCONNECTED, "AP_STADEV_DISCONNECT"},
                    {WIFI_EVENT_AP_PROBEREQRECVED, "AP_PROBE_REQ"},

                    {WIFI_EVENT_FTM_REPORT, "FTM_REPORT_RDY"},

                    {WIFI_EVENT_STA_BSS_RSSI_LOW, "RSSI_LOW_ALERT"},
                    {WIFI_EVENT_ACTION_TX_STATUS, "ACTION_TX_STATUS"},
                    {WIFI_EVENT_ROC_DONE, "ROC_DONE"},

                    {WIFI_EVENT_STA_BEACON_TIMEOUT, "\x1b[31mSTA_BEACON_TIMEOUT"},

                    {WIFI_EVENT_CONNECTIONLESS_MODULE_WAKE_INTERVAL_START, "CONNECTIONLESS_MODULE_WAKE_INTERVAL_START"},

                    {WIFI_EVENT_AP_WPS_RG_SUCCESS, "AP_WPS_RG_SUCCESS"},
                    {WIFI_EVENT_AP_WPS_RG_FAILED, "AP_WPS_RG_FAILED"},
                    {WIFI_EVENT_AP_WPS_RG_TIMEOUT, "AP_WPS_RG_TIMEOUT"},
                    {WIFI_EVENT_AP_WPS_RG_PIN, "AP_WPS_RG_PIN"},
                    {WIFI_EVENT_AP_WPS_RG_PBC_OVERLAP, "AP_WPS_RG_PBC_OVERLAP"},

                    {WIFI_EVENT_ITWT_SETUP, "ITWT_SETUP"},
                    {WIFI_EVENT_ITWT_TEARDOWN, "ITWT_TEARDOWN"},
                    {WIFI_EVENT_ITWT_PROBE, "ITWT_PROBE"},
                    {WIFI_EVENT_ITWT_SUSPEND, "ITWT_SUSPEND"},
                    {WIFI_EVENT_TWT_WAKEUP, "TWT_WAKEUP"},
                    {WIFI_EVENT_BTWT_SETUP, "BTWT_SETUP"},
                    {WIFI_EVENT_BTWT_TEARDOWN, "BTWT_TEARDOWN"},

                    {WIFI_EVENT_NAN_STARTED, "NAN_STARTED"},
                    {WIFI_EVENT_NAN_STOPPED, "NAN_STOPPED"},
                    {WIFI_EVENT_NAN_SVC_MATCH, "NAN_SVC_MATCH"},
                    {WIFI_EVENT_NAN_REPLIED, "NAN_REPLIED"},
                    {WIFI_EVENT_NAN_RECEIVE, "NAN_RECEIVE"},
                    {WIFI_EVENT_NDP_INDICATION, "NDP_INDICATION"},
                    {WIFI_EVENT_NDP_CONFIRM, "NDP_CONFIRM"},
                    {WIFI_EVENT_NDP_TERMINATED, "NDP_TERMINATED"},

                    {WIFI_EVENT_HOME_CHANNEL_CHANGE, "HOME_CHANNEL_CHANGE"},
                    {WIFI_EVENT_STA_NEIGHBOR_REP, "STA_NEIGHBOR_REP"},

                }};

                for (auto &pair : event_to_str_map)
                {
                    if (pair.first == key)
                    {
                        return pair.second;
                    }
                }

                return "UNDEFINED";
            }

            static const char* wlan_auth_mode(wifi_auth_mode_t key)
            {
                // constexpr std::array<std::pair<wifi_auth_mode_t, const char *>, 16> event_to_str_map = {{
                //     /*{WIFI_AUTH_OPEN, "OPEN"},
                //     {WIFI_AUTH_WEP, "WEP"},
                //     {WIFI_AUTH_WPA_PSK, "WPA_PSK"},
                //     {WIFI_AUTH_WPA2_PSK, "WPA2_PSK"},
                //     {WIFI_AUTH_WPA_WPA2_PSK, "WPA_WPA2_PSK"},
                //     {WIFI_AUTH_ENTERPRISE, "ENTERPRISE"},
                //     {WIFI_AUTH_WPA2_ENTERPRISE, "ENTERPRISE"},
                //     {WIFI_AUTH_WPA3_PSK, "WPA3_PSK"},
                //     {WIFI_AUTH_WPA2_WPA3_PSK, "WPA2_WPA3_PSK"},
                //     {WIFI_AUTH_WAPI_PSK, "WAPI_PSK"},
                //     {WIFI_AUTH_OWE, "OWE"},
                //     {WIFI_AUTH_WPA3_ENT_192, "WPA3_ENT_192"},
                //     {WIFI_AUTH_WPA3_EXT_PSK, "WPA3_EXT_PSK"},
                //     {WIFI_AUTH_WPA3_EXT_PSK_MIXED_MODE, "WPA3_EXT_PSK_MIXED_MODE"},
                //     {WIFI_AUTH_DPP, "DPP"},
                //     {WIFI_AUTH_MAX, "MAX"}*/
                // }};

                // for (auto &pair : event_to_str_map)
                // {
                //     if (pair.first == key)
                //     {
                //         return pair.second;
                //     }
                // }

                return "UNDEFINED";
            }

            static const char* known_device_by_mac(uint64_t mac)
            {
                constexpr std::array<std::pair<uint64_t, const char *>, 5> mac_to_name = {{
                    {0xf0f5bdfba81c, "ESP_FRIDGE"},
                    {0x94e23ce4dae5, "FFL_LAPTOP"},
                    {0xa4423ba74ad1, "DEV_UNKNOWN"},
                    {0x24587ccce958, "ESP_C3_SMARTED"},
                    {0x002324c2382f, "GPRO"},
                }};

                for (auto &pair : mac_to_name)
                {
                    if (pair.first == mac)
                    {
                        return pair.second;
                    }
                }

                return "";
            }
    };
}


