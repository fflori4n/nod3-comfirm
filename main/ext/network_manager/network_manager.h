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

            static constexpr char* network_sensor_template = "{\"w_rssi\":%d.%d,\"w_con_tm\":%d.%d,\"w_recon\":%d,\"w_condev\":%d,\"w_alldev\":%d,\"w_chdev\":%d,\"w_apssid\":\"%s\"}";

           // constexpr static const char* wlanSSID{"TS-uG65"};/*{"adsl sandor"}*//*;*/
            //constexpr static const char* wlanPassword{"4XfuPgEx"}/*{"floriflori"}*//**/;
            constexpr static const char* wlanSSID{"adsl sandor"};
            constexpr static const char* wlanPassword{"floriflori"};

        private:
            char macAddressCStr[12+1]{};
            static tWlanState _wlanIfaceState;
            static wifi_init_config_t _wifiInitCfg;
            static wifi_config_t _wifiConfig;

            esp_netif_t* p_netif;
        public:

            typedef struct wlan_manager_stats_t{

                uint32_t connection_ok_sec;
                int rssi;
                uint16_t num_of_reconnect;


                uint8_t num_of_ap;
                uint8_t num_of_ap_current_channel;
                uint8_t num_of_dev_current_channel;
                uint8_t num_of_dev;
                uint8_t num_of_dev_connected_to_ap;

                uint8_t channel;

            } wlan_manager_stats_t;

            typedef struct wlan_scan_devices_t{

                uint64_t device_mac;
                uint64_t associated_mac;
                uint8_t channel;
                bool isAP;
                float rssi;
                char* ssid; /* This is a bad idea */
                uint16_t scan_cycle_since_detect;

            }wlan_scan_devices_t;

            typedef struct wlan_access_point_id_t{

                char* ssid;
                char* passwd;
                uint64_t device_mac;
                uint8_t channel;
                uint8_t low_rssi_tresh;

            }wlan_access_point_id_t;

            static constexpr uint8_t wlan_active_ap_index{0};
            static std::array<wlan_access_point_id_t,1>  access_point_list;

            static constexpr uint8_t maximum_number_of_APs{20};
            static constexpr uint8_t maimum_number_of_STAs{80};
            static std::array<wlan_scan_devices_t, (maximum_number_of_APs + maimum_number_of_STAs)> wlan_device_list;

            wlan_manager_stats_t statistics;

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
            static void print_collected_device(void);
            static uint16_t count_sta_on_channel(uint8_t channel);      
            static uint16_t count_devices_connected_to_AP(uint64_t ap_mac); /* including AP itself */
            static uint16_t count_all_sta(void);
            static uint16_t count_ap_on_channel(uint8_t channel);
            static uint16_t count_all_ap(void);

            int16_t get_fraction(float value){
                return (abs(((int16_t)(value * 100)) % 100));
            }

    
            esp_err_t get_service_data(char *text_buffer, int16_t text_buffer_size)
            {

                esp_wifi_sta_get_rssi(&statistics.rssi);
                
                int16_t res = snprintf(text_buffer, text_buffer_size, network_sensor_template,
                                       (int16_t)(statistics.rssi), get_fraction(statistics.rssi),                                   /* w_rssi */
                                       (int16_t)(statistics.connection_ok_sec* 1.0/60.0), get_fraction(statistics.connection_ok_sec* 1.0/60.0),     /* w_con_tm */
                                       (int16_t)(statistics.num_of_reconnect),                                                                                                /* w_recon */
                                       (int16_t)(statistics.num_of_dev_connected_to_ap),                                                            /* w_condev */
                                       (int16_t)(statistics.num_of_ap + statistics.num_of_dev),                                                     /* w_alldev */
                                       (int16_t)(statistics.num_of_ap_current_channel + statistics.num_of_dev_current_channel),                     /* w_chdev */
                                       wlanSSID
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
    };
}


