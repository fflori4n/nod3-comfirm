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

            constexpr static const char* wlanSSID/*{"GGGGG"}*//*{"adsl sandor"}*/{"TS-uG65"};
            constexpr static const char* wlanPassword/*{"123456789"}*//*{"floriflori"}*/{"4XfuPgEx"};

        private:
            char macAddressCStr[12+1]{};
            static tWlanState _wlanIfaceState;
            static wifi_init_config_t _wifiInitCfg;
            static wifi_config_t _wifiConfig;
        public:
            static wifi_power_save_e wifi_power_save_mode;
            constexpr static char* logLabel{"NETWORK"};
            static uint64_t macAddress;
            

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
            
            esp_err_t loadMACAddress(void);
            esp_err_t fastScan(void);

            constexpr char* getMACAddressCStr(void){ return macAddressCStr;};
    };
}


