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
#include <mutex>

namespace NETWORK
{
    class Wlan{

        public:

            constexpr static char* network_tag{COLOR_BLUE "NETWORK" COLOR_WHITE};
            constexpr static char* net_scan_tag{COLOR_BLUE "NET_SCAN" COLOR_WHITE};
            constexpr static char* network_async_tag{COLOR_CYAN "NETWORK_ASINC" COLOR_WHITE};

            
            static inline std::mutex wlan_state_mutex{};
            static inline std::mutex wifi_driver_mutex{};
            static inline std::mutex wifi_driver_callback_mutex{};  /* Maybe it makes sense to use a separate mutex for callbacks because they are async compred to main task. */

            enum class wlan_state_t
            {
                NOT_INITIALIZED,
                INTERFACE_DOWN,
                NOT_CONNECTED,
                STA_CONNECT,
                WAITING_EVENT_STA_CONNECT,
                WAITING_EVENT_STA_GOT_IP,
                STA_CONNECTION_ESTABLISHED,
                STA_CONNECTED_RDY,
                AP_ACTIVE_RDY,
                STA_DISCONNECTED,
                STA_REQUEST_DISCONNECT,
                POWERING_DOWN,
                ER_UNDEFINED_ERROR,
                ER_UNABLE_TO_LOCK,
                /* END */
                numof_wlan_states
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

                float estimated_distance_from_ap;
                float connection_ok_hours;

                uint8_t channel;

            } wlan_manager_stats_t;

            static constexpr uint16_t wlan_scan_period_sec{60};
            static constexpr uint8_t numof_wlan_access_point_names{3};

            static constexpr char* network_sensor_template = "\"w_rssi\":%.2f,\"w_dest\":%.2f,\"w_con_tm\":%.2f,\"w_recon\":%d,\"w_condev\":%d,\"w_alldev\":%d,\"w_chdev\":%d,\"w_apssid\":\"%s\"";
            static constexpr time_t report_cycle_time_sec{2 * 60};
            RTC_FAST_ATTR static inline time_t last_report_unix;

        private:
            char macAddressCStr[12+1]{};

            static wlan_state_t wlan_state;
            
            

            static wifi_init_config_t _wifiInitCfg;
            static wifi_config_t _wifiConfig;

            esp_netif_t* p_netif;
        public:

            /*static wlan_state_t previous_wlan_state;*/
            inline static uint32_t  switched_state_tm = 0;

            static constexpr uint64_t espressif_macs[] = {0x048308,0x083A8D,0x083AF2,0x08A6F7,0x08B61F,0x08D1F9,0x08F9E0,0x0C8B95,0x0CB815,0x0CDC7E,0x10003B,0x10061C,0x10521C,0x1091A8,0x1097BD,0x142B2F,0x188B0E,0x18FE34,0x1C6920,0x1C9DC2,0x2043A8,0x240AC4,0x244CAB,0x24587C,0x2462AB,0x246F28,0x24A160,0x24B2DE,0x24D7EB,0x24DCC3,0x24EC4A,0x28372F,0x2C3AE8,0x2CBCBB,0x2CF432,0x3030F9,0x308398,0x30AEA4,0x30C6F7,0x30C922,0x30EDA0,0x345F45,0x348518,0x34865D,0x349454,0x34987A,0x34AB95,0x34B472,0x34B7DA,0x34CDB0,0x38182B,0x3C6105,0x3C71BF,0x3C8427,0x3C8A1F,0x3CE90E,0x4022D8,0x404CCA,0x409151,0x40F520,0x441793,0x4827E2,0x4831B7,0x483FDA,0x485519,0x48CA43,0x48E729,0x4C11AE,0x4C7525,0x4CEBD6,0x500291,0x543204,0x5443B2,0x545AA6,0x588C81,0x58BF25,0x58CF79,0x5C013B,0x5CCF7F,0x600194,0x6055F9,0x64B708,0x64E833,0x686725,0x68B6B3,0x68C63A,0x6CB456,0x70039F,0x70041D,0x70B8F6,0x744DBD,0x782184,0x78421C,0x78E36D,0x78EE4C,0x7C2C67,0x7C7398,0x7C87CE,0x7C9EBD,0x7CDFA1,0x80646F,0x806599,0x807D3A,0x840D8E,0x84CCA8,0x84F3EB,0x84F703,0x84FCE6,0x8813BF,0x8C4B14,0x8C4F00,0x8CAAB5,0x8CBFEA,0x8CCE4E,0x901506,0x90380C,0x9097D5,0x943CC6,0x9454C5,0x94A990,0x94B555,0x94B97E,0x94E686,0x983DAE,0x98CDAC,0x98F4AB,0x9C9C1F,0x9C9E6E,0xA020A6,0xA0764E,0xA085E3,0xA0A3B3,0xA0B765,0xA0DD6C,0xA47B9D,0xA4CF12,0xA4E57C,0xA8032A,0xA842E3,0xA848FA,0xAC0BFB,0xAC1518,0xAC67B2,0xACD074,0xB08184,0xB0A732,0xB0B21C,0xB43A45,0xB48A0A,0xB4E62D,0xB8D61A,0xB8F009,0xBCDDC2,0xBCFF4D,0xC049EF,0xC04E30,0xC05D89,0xC44F33,0xC45BBE,0xC4D8D5,0xC4DD57,0xC4DEE2,0xC82B96,0xC82E18,0xC8C9A3,0xC8F09E,0xCC50E3,0xCC7B5C,0xCC8DA2,0xCCBA97,0xCCDBA7,0xD0EF76,0xD48AFC,0xD48C49,0xD4D4DA,0xD4F98D,0xD8132A,0xD83BDA,0xD8A01D,0xD8BC38,0xD8BFC0,0xD8F15B,0xDC0675,0xDC1ED5,0xDC4F22,0xDC5475,0xDCDA0C,0xE05A1B,0xE09806,0xE0E2E6,0xE465B8,0xE4B063,0xE4B323,0xE80690,0xE831CD,0xE868E7,0xE86BEA,0xE89F6D,0xE8DB84,0xEC6260,0xEC64C9,0xEC94CB,0xECC9FF,0xECDA3B,0xECFABC,0xF008D1,0xF024F9,0xF09E9E,0xF0F5BD,0xF412FA,0xF4650B,0xF4CFA2,0xF8B3B7,0xFCB467,0xFCE8C0,0xFCF5C4,};
           
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

            class Wlan_access_point{

                public:

                char ssid[32];
                char passwd[64];
                uint64_t device_mac;
                uint8_t channel;
                int8_t low_rssi_tresh;
                uint8_t priority;
                uint8_t last_seen;

                /*float connect_score;*/
                Wlan_access_point() = default;
                Wlan_access_point(char* ssid, char* passwd, const uint64_t device_mac, const uint8_t channel, const int8_t low_rssi, const uint8_t priority, uint8_t last_seen) : device_mac(device_mac),channel(channel),low_rssi_tresh(low_rssi),priority(priority),last_seen(last_seen)
                {
                    /* @TODO: some kind of warning when str truncated? */
                    strncpy(this->ssid, ssid, sizeof(this->ssid)/sizeof(char));
                    strncpy(this->passwd, passwd, sizeof(this->passwd)/sizeof(char));
                }

                Wlan_access_point(const Wlan_access_point& other){
                    this->device_mac = other.device_mac;
                    this->channel = other.channel;
                    this->low_rssi_tresh = other.low_rssi_tresh;
                    this->priority = other.priority;
                    this->last_seen = other.last_seen;

                    strncpy(this->ssid, other.ssid, sizeof(this->ssid)/sizeof(char));
                    strncpy(this->passwd, other.passwd, sizeof(this->passwd)/sizeof(char));
                }

                void print(void){
                    ESP_LOGI(log_label,"WLAN_AP: %s, pass: %s, %12llx, CH%d, RSSI dropout: %d, prio: %d, last seen: %d", ssid, passwd, device_mac, channel, low_rssi_tresh, priority, last_seen);
                }

                char* get_ssid() const { return const_cast<char*>(ssid); }
            };

            typedef struct wlan_scan_devices_t{

                std::string ssid;
                uint64_t device_mac;
                uint64_t associated_mac;
                uint16_t scan_cycle_since_detect;
                uint8_t channel;
                float rssi;
                bool isAP;
                
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

            

            /* arraylist to hold APs that are preset for connection if present */
            static uint8_t associated_ap_index;
            

            static constexpr uint8_t maximum_number_of_APs{20};
            static constexpr uint8_t maimum_number_of_STAs{80};
            static constexpr bool enable_switch_to_higher_prio{false}; /* this will allow the ESP to leave a good connection and switch AP if a higher prio AP is present, TODO: could be used for OTA update, with a predefined name or prio*/
            
            static RTC_FAST_ATTR std::array<wlan_scan_devices_t, (maximum_number_of_APs + maimum_number_of_STAs)> wlan_device_list;
            static std::array<Wlan_access_point, numof_wlan_access_point_names>  access_point_list;
            static uint64_t esp_my_mac;
            

            static RTC_FAST_ATTR wlan_manager_stats_t statistics;

            uint16_t scan_period_counter = 0;
            constexpr static char* log_label{"\x1b[34mNETWORK"};
            constexpr static char* log_label_scan{"\x1b[34mSCAN"};

            static uint64_t macAddress;
            uint32_t wifi_connected_sec;
            uint32_t wifi_trying_to_connect;
            static bool wifi_ext_task_inhibit_flag;
            inline static char* ssid_name = "";

            

            uint16_t num_of_active_ap_scanned = maximum_number_of_APs;
            inline RTC_FAST_ATTR static wifi_ap_record_t ap_info[maximum_number_of_APs];
            inline RTC_FAST_ATTR static uint16_t ap_count = 0;
            

        private:
            static void network_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data);
        public:

            esp_err_t init(void);
            esp_err_t start_ap(void);
            esp_err_t fastScan(void);

            static float estimate_ap_dist_from_rssi(int8_t rssi);

            static void wlan_promiscuous_handler(void *buff, wifi_promiscuous_pkt_type_t type);
            static esp_err_t collect_wifi_device(wlan_scan_devices_t pckt_source_device);
            static esp_err_t add_device_to_dev_list(wlan_scan_devices_t new_device);
            static void print_collected_device(void);
            static uint16_t count_sta_on_channel(uint8_t channel);      
            static uint16_t count_devices_connected_to_AP(uint64_t ap_mac); /* including AP itself */
            static uint16_t count_all_sta(void);
            static uint16_t count_ap_on_channel(uint8_t channel);
            static uint16_t count_all_ap(void);
            static void calculate_nearby_dev_statistics(wlan_manager_stats_t &statistics, Wlan::Wlan_access_point &associated_ap);
            
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

            wlan_state_t get_wlan_state(void) {
                std::unique_lock<std::mutex> lock(wlan_state_mutex, std::try_to_lock);

                if (true == lock.owns_lock())
                {
                    return wlan_state;
                }
                else 
                {
                    return wlan_state_t::ER_UNABLE_TO_LOCK;
                }
            }

            esp_err_t set_wlan_state(const wlan_state_t new_wlan_state){
                std::unique_lock<std::mutex>lock(wlan_state_mutex, std::try_to_lock);

                if(true == lock.owns_lock())
                {
                    if(wlan_state != new_wlan_state)
                    {
                        ESP_LOGI(NETWORK::Wlan::network_tag, "wlan state switched to %s from %s after %lld secs.", Wlan::wlan_state_to_cstr(new_wlan_state), Wlan::wlan_state_to_cstr(wlan_state), ((esp_timer_get_sec() + 1) - switched_state_tm));
                        wlan_state = new_wlan_state;
                        switched_state_tm = esp_timer_get_sec();
                    }
                }
                else
                {
                    return ESP_FAIL;
                }

                return ESP_OK;
            }

            static const char* wlan_state_to_cstr(wlan_state_t state)
            {
                switch (state) {
                    case wlan_state_t::NOT_INITIALIZED:               return "NOT_INITIALIZED";
                    case wlan_state_t::INTERFACE_DOWN:                return "INTERFACE_DOWN";
                    case wlan_state_t::NOT_CONNECTED:                 return "NOT_CONNECTED";
                    case wlan_state_t::STA_CONNECT:                   return "STA_CONNECT";
                    case wlan_state_t::WAITING_EVENT_STA_CONNECT:     return "WAITING_EVENT_STA_CONNECT";
                    case wlan_state_t::WAITING_EVENT_STA_GOT_IP:      return "WAITING_EVENT_STA_GOT_IP";
                    case wlan_state_t::STA_CONNECTION_ESTABLISHED:    return "STA_CONNECTION_ESTABLISHED";
                    case wlan_state_t::STA_CONNECTED_RDY:             return "STA_CONNECTED_RDY";
                    case wlan_state_t::AP_ACTIVE_RDY:                 return "AP_ACTIVE_RDY";
                    case wlan_state_t::STA_DISCONNECTED:              return "STA_DISCONNECTED";
                    case wlan_state_t::STA_REQUEST_DISCONNECT:        return "STA_REQUEST_DISCONNECT";
                    case wlan_state_t::POWERING_DOWN:                 return "POWERING_DOWN";
                    case wlan_state_t::ER_UNDEFINED_ERROR:            return "ER_UNDEFINED_ERROR";
                    case wlan_state_t::ER_UNABLE_TO_LOCK:             return "ER_UNABLE_TO_LOCK";
                    default:                                          return "UNKNOWN";
                }
            }

            inline bool is_sta_connected(void)
            {
                wlan_state_t current_state = get_wlan_state();

                /* NOTE: wlan_state_t::STA_CONNECTION_ESTABLISHED will return false, it should switch to wlan_state_t::STA_CONNECTED_RDY in a short time, unless our device gets disconnected immediatelly*/
                if(wlan_state_t::STA_CONNECTED_RDY == current_state)
                {
                    return true;
                }

                return false;
            };

            static const char* ip_event_to_string(ip_event_t event)
            {
                switch (event) {
                    case IP_EVENT_STA_GOT_IP:    return "STA_GOT_IP";
                    case IP_EVENT_STA_LOST_IP:   return "STA_LOST_IP";
                    case IP_EVENT_GOT_IP6:       return "GOT_IP6";
                    case IP_EVENT_ETH_GOT_IP:    return "ETH_GOT_IP";
                    case IP_EVENT_ETH_LOST_IP:   return "ETH_LOST_IP";
                    case IP_EVENT_PPP_GOT_IP:    return "PPP_GOT_IP";
                    case IP_EVENT_PPP_LOST_IP:   return "PPP_LOST_IP";
                    case IP_EVENT_TX_RX:         return "RX_TX";
                    default:                     return "UNDEFINED";
                }
            }

            static const char* wifi_event_to_string(wifi_event_t event)
            {
                switch (event) {
                    case WIFI_EVENT_WIFI_READY:             return "WIFI_READY";
                    case WIFI_EVENT_SCAN_DONE:              return "SCAN_FINISHED";
                    case WIFI_EVENT_STA_START:              return "STA_START";
                    case WIFI_EVENT_STA_STOP:               return "STA_STOP";
                    case WIFI_EVENT_STA_CONNECTED:          return "STA_CONNECTED";
                    case WIFI_EVENT_STA_DISCONNECTED:       return "\x1b[31mSTA_DISCONNECTED";
                    case WIFI_EVENT_STA_AUTHMODE_CHANGE:    return "STA_AUTHMODE_CHANGE";
                    case WIFI_EVENT_STA_WPS_ER_SUCCESS:     return "WPS_ER_SUCCESS";
                    case WIFI_EVENT_STA_WPS_ER_FAILED:      return "WPS_ER_FAILED";
                    case WIFI_EVENT_STA_WPS_ER_TIMEOUT:     return "WPS_ER_TIMEOUT";
                    case WIFI_EVENT_STA_WPS_ER_PIN:         return "WPS_ER_PIN";
                    case WIFI_EVENT_STA_WPS_ER_PBC_OVERLAP: return "WPS_ER_PBC_OVERLAP";
                    case WIFI_EVENT_AP_START:               return "AP_START";
                    case WIFI_EVENT_AP_STOP:                return "AP_STOP";
                    case WIFI_EVENT_AP_STACONNECTED:        return "AP_STADEV_CONNECT";
                    case WIFI_EVENT_AP_STADISCONNECTED:     return "AP_STADEV_DISCONNECT";
                    case WIFI_EVENT_AP_PROBEREQRECVED:      return "AP_PROBE_REQ";
                    case WIFI_EVENT_FTM_REPORT:             return "FTM_REPORT_RDY";
                    case WIFI_EVENT_STA_BSS_RSSI_LOW:       return "RSSI_LOW_ALERT";
                    case WIFI_EVENT_ACTION_TX_STATUS:       return "ACTION_TX_STATUS";
                    case WIFI_EVENT_ROC_DONE:               return "ROC_DONE";
                    case WIFI_EVENT_STA_BEACON_TIMEOUT:     return "\x1b[31mSTA_BEACON_TIMEOUT";
                    case WIFI_EVENT_CONNECTIONLESS_MODULE_WAKE_INTERVAL_START: return "CONNECTIONLESS_MODULE_WAKE_INTERVAL_START";
                    case WIFI_EVENT_AP_WPS_RG_SUCCESS:      return "AP_WPS_RG_SUCCESS";
                    case WIFI_EVENT_AP_WPS_RG_FAILED:       return "AP_WPS_RG_FAILED";
                    case WIFI_EVENT_AP_WPS_RG_TIMEOUT:      return "AP_WPS_RG_TIMEOUT";
                    case WIFI_EVENT_AP_WPS_RG_PIN:          return "AP_WPS_RG_PIN";
                    case WIFI_EVENT_AP_WPS_RG_PBC_OVERLAP:  return "AP_WPS_RG_PBC_OVERLAP";
                    case WIFI_EVENT_ITWT_SETUP:             return "ITWT_SETUP";
                    case WIFI_EVENT_ITWT_TEARDOWN:          return "ITWT_TEARDOWN";
                    case WIFI_EVENT_ITWT_PROBE:             return "ITWT_PROBE";
                    case WIFI_EVENT_ITWT_SUSPEND:           return "ITWT_SUSPEND";
                    case WIFI_EVENT_TWT_WAKEUP:             return "TWT_WAKEUP";
                    case WIFI_EVENT_BTWT_SETUP:             return "BTWT_SETUP";
                    case WIFI_EVENT_BTWT_TEARDOWN:          return "BTWT_TEARDOWN";
                    case WIFI_EVENT_NAN_STARTED:            return "NAN_STARTED";
                    case WIFI_EVENT_NAN_STOPPED:            return "NAN_STOPPED";
                    case WIFI_EVENT_NAN_SVC_MATCH:          return "NAN_SVC_MATCH";
                    case WIFI_EVENT_NAN_REPLIED:            return "NAN_REPLIED";
                    case WIFI_EVENT_NAN_RECEIVE:            return "NAN_RECEIVE";
                    case WIFI_EVENT_NDP_INDICATION:         return "NDP_INDICATION";
                    case WIFI_EVENT_NDP_CONFIRM:            return "NDP_CONFIRM";
                    case WIFI_EVENT_NDP_TERMINATED:         return "NDP_TERMINATED";
                    case WIFI_EVENT_HOME_CHANNEL_CHANGE:    return "HOME_CHANNEL_CHANGE";
                    case WIFI_EVENT_STA_NEIGHBOR_REP:       return "STA_NEIGHBOR_REP";
                    default:                                return "UNDEFINED";
                }
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


