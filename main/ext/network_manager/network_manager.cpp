#include "network_manager.h"

namespace NETWORK
{
    /* static member init */
    Wlan::tWlanState Wlan::_wlanIfaceState{Wlan::tWlanState::wlanState_notInitialised};
    std::mutex Wlan::initLock_mutex{};
    wifi_init_config_t Wlan::_wifiInitCfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t Wlan::_wifiConfig{};

Wlan::Wlan(){
    ESP_LOGI(logLabel,"WLAN constructor called.");
};

/* FROM: https://github.com/espressif/esp-idf/blob/master/examples/wifi/getting_started/station/main/station_example_main.c */
/* FROM: https://www.youtube.com/watch?v=3dM6LiAriEg */ 
esp_err_t Wlan::init(void){

    std::lock_guard<std::mutex> guard(initLock_mutex);
    esp_err_t retStatus = ESP_OK;

    if( tWlanState::wlanState_notInitialised == _wlanIfaceState){

        retStatus = esp_netif_init();
        /*esp_event_loop_create_default()*/

        if(ESP_OK == retStatus){
            const esp_netif_t* const p_netif = esp_netif_create_default_wifi_sta();

            if(NULL == p_netif){
                retStatus = ESP_FAIL;
            }
        }

        if(ESP_OK == retStatus){
            _wifiInitCfg = WIFI_INIT_CONFIG_DEFAULT(); /*@TODO: set at compile to be default.*/
            retStatus = esp_wifi_init(&_wifiInitCfg);
        }

        if(ESP_OK == retStatus){

            if(strlen(wlanSSID) > sizeof(_wifiConfig.sta.ssid)){
                ESP_LOGW(logLabel, "SSID too long, will be trancated to buffer size!");
            }
            if(strlen(wlanPassword) > sizeof(_wifiConfig.sta.password)){
                ESP_LOGW(logLabel, "SSID too long, will be trancated to buffer size!");
            }

            memcpy(_wifiConfig.sta.ssid, wlanSSID, std::min(strlen(wlanSSID), sizeof(_wifiConfig.sta.ssid)));
            memcpy(_wifiConfig.sta.password, wlanPassword, std::min(strlen(wlanPassword),sizeof(_wifiConfig.sta.password)));

            /*_wifiConfig.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK,
            _wifiConfig.sta.pmf_cfg.capable     = true;
            _wifiConfig.sta.pmf_cfg.required    = false;*/ /* Works fine without this crap in example */

             /* register wifi event handler functions */
            esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, _wifiEventHandler, NULL);
            esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, _wifiEventHandler, NULL);
            
        }

        if(ESP_OK == retStatus){
             
            esp_wifi_set_config(WIFI_IF_STA, &_wifiConfig);

        }

        _wlanIfaceState = tWlanState::wlanState_initCompleted;
    }

    return retStatus;
}

esp_err_t Wlan::begin(void){

    esp_err_t retStatus = ESP_OK;

    if(tWlanState::wlanState_notInitialised == _wlanIfaceState){
        retStatus = this->init();
    }

    if(ESP_OK == retStatus){
        
        _wlanIfaceState = tWlanState::wlanState_rdyToConnect;
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();
        retStatus = esp_wifi_connect();
    }
    
    return retStatus;
}

esp_err_t Wlan::begin(char* ssid, char* passwd){


    /*TODO: set wifi config*/
    return this->begin();
}


void Wlan::_wifiEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data){

    if(event_id == WIFI_EVENT_STA_START)
    {
        _wlanIfaceState = tWlanState::wlanState_connecting;
        ESP_LOGI("WLAN_EVENT","Connecting : STA start");
    }
    else if (event_id == WIFI_EVENT_STA_CONNECTED)
    {
        _wlanIfaceState = tWlanState::wlanState_waitingForIP;
        ESP_LOGI("WLAN_EVENT","Connecting : waiting for IP");
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
        _wlanIfaceState = tWlanState::wlanState_connected;
        ESP_LOGI("WLAN_EVENT","Is connected : got IP");
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        _wlanIfaceState = tWlanState::wlanState_disconnected;
        ESP_LOGI("WLAN_EVENT","Disconnected");
    }
    else{
        /*_wlanIfaceState = tWlanState::wlanState_error;*/
        ESP_LOGI("WLAN_EVENT","Undefined event");
    }
}

/* from: https://www.youtube.com/watch?v=qdbbZYdT9Dg*/
esp_err_t Wlan::loadMACAddress(void)
{
    esp_err_t retStatus = ESP_FAIL;

    if('\0' == macAddressCStr[0]){
        uint8_t mac_byte_buffer[6]{};
    
        retStatus = esp_efuse_mac_get_default(mac_byte_buffer);
        if (ESP_OK == retStatus)
        {
        snprintf(macAddressCStr, sizeof(macAddressCStr), 
        "%02x%02x%02x%02x%02x%02x",
            mac_byte_buffer[0],
            mac_byte_buffer[1],
            mac_byte_buffer[2],
            mac_byte_buffer[3],
            mac_byte_buffer[4],
            mac_byte_buffer[5]
          );
        }

        ESP_LOGI(logLabel,"MAC address loaded: %s",macAddressCStr);
    }
    else{
        ESP_LOGI(logLabel,"mac already loaded: %s",macAddressCStr);
    }

    return retStatus;
}

#define DEFAULT_SCAN_LIST_SIZE 20

static const char *TAG = "scan";

/* Initialize Wi-Fi as sta and set scan method */
esp_err_t Wlan::fastScan(void){

    esp_err_t retStatus = ESP_OK;

    if(tWlanState::wlanState_notInitialised == _wlanIfaceState){
        retStatus = this->init();
    }

    /* NOTE: W (609) wifi:sta_scan: STA is connecting, scan are not allowed!*/
    if(tWlanState::wlanState_rdyToConnect == _wlanIfaceState ||
        tWlanState::wlanState_connecting == _wlanIfaceState ||
        tWlanState::wlanState_waitingForIP == _wlanIfaceState){
        retStatus = ESP_FAIL;

        ESP_LOGI("", "STA is connecting, scan are not allowed!");
    }

    if(ESP_OK == retStatus){

        uint16_t number = DEFAULT_SCAN_LIST_SIZE;
        wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
        uint16_t ap_count = 0;
        memset(ap_info, 0, sizeof(ap_info));

        esp_wifi_scan_start(NULL, true);

        ESP_LOGI(TAG, "Max AP number ap_info can hold = %u", number);
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
        ESP_LOGI(TAG, "Total APs scanned = %u, actual AP number ap_info holds = %u", ap_count, number);
        for (int i = 0; i < number; i++) {
            ESP_LOGI("", "SSID %s\t\t\t RSSI %d CH: \t\t%d", ap_info[i].ssid, ap_info[i].rssi, ap_info[i].primary);
        } 
    }

    if(ESP_OK != retStatus){ESP_LOGE("", "Scan returned /w fail status.");}
    return retStatus;
}

void task_manageWlanConnection(void *parameters){

        Wlan wifiIF;
        wifiIF.loadMACAddress();
        wifiIF.begin();

        for(;;){
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            wifiIF.fastScan();
            /*printf("MAC: %s\n", wifiIF.getMACAddressCStr());*/
        }  
}

} /* NETWORK */