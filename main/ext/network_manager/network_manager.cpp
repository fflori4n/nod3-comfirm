#include "network_manager.h"

namespace NETWORK
{
    /* static member init */
    Wlan::tWlanState Wlan::_wlanIfaceState{Wlan::tWlanState::wlanState_notInitialised};
    std::mutex Wlan::wifi_driver_mutex{};
    std::mutex Wlan::wifi_driver_callback_mutex{};
    wifi_init_config_t Wlan::_wifiInitCfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t Wlan::_wifiConfig{};
    Wlan::wifi_power_save_e Wlan::wifi_power_save_mode{Wlan::wifi_power_save_e::psave_maximum};

Wlan::Wlan(){
    ESP_LOGI(logLabel,"WLAN constructor called.");
};

/* FROM: https://github.com/espressif/esp-idf/blob/master/examples/wifi/getting_started/station/main/station_example_main.c */
/* FROM: https://www.youtube.com/watch?v=3dM6LiAriEg */ 
esp_err_t Wlan::init(void){

    std::lock_guard<std::mutex> guard(wifi_driver_mutex);
    esp_err_t retStatus = ESP_OK;

    if( tWlanState::wlanState_notInitialised == _wlanIfaceState){

        retStatus = esp_netif_init();
        /*esp_event_loop_create_default()*/

        /* register wifi event handler functions */
        if (ESP_OK == retStatus)
        {
            retStatus = esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&_wifiEventHandler,nullptr,nullptr);
        }
        if (ESP_OK == retStatus)
        {
            retStatus = esp_event_handler_instance_register(IP_EVENT,ESP_EVENT_ANY_ID,&_wifiEventHandler,nullptr,nullptr);
        }

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
        /* set WIFI power saving mode - more saving slower response */
        if(ESP_OK == retStatus){

            const wifi_ps_type_t power_mode{static_cast<wifi_ps_type_t>(wifi_power_save_mode)};
            retStatus = esp_wifi_set_ps(power_mode);
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

        }

        if(ESP_OK == retStatus)
        {
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

esp_err_t Wlan::disconnect_power_off(void){

    esp_err_t retStatus = ESP_OK;

    if(tWlanState::wlanState_notInitialised != _wlanIfaceState){

        std::lock_guard<std::mutex> guard(wifi_driver_mutex);

        retStatus = esp_wifi_disconnect();  /* Graceful disconnect from AP, or NOT connceted/ No init return if not connected*/
        _wlanIfaceState = tWlanState::wlanState_disconnected;

        if(ESP_FAIL != retStatus)
        {
            esp_wifi_stop();    /* stop STA mode, dont care about return*/
            _wlanIfaceState = tWlanState::wlanState_initCompleted;
            //esp_wifi_deinit();  /* clear init and stop wifi task - next time, init is needed again.*/
            retStatus = ESP_OK;
        }
        else
        {
            /* @TODO: go to error?? or keep last state?*/
        }
    }
    
    return retStatus;
}

esp_err_t Wlan::begin(char* ssid, char* passwd){


    /*TODO: set wifi config*/
    return this->begin();
}

/* TODO: not very memory efficient but convinient to avoid char* vs const char* issues */
/* this also does not work when one argument is const char, the other is not. so consider casting?? other option to avoid casting random crap??*/
template<typename cstr_type>
esp_err_t Wlan::sta_connect(cstr_type* &ssid, cstr_type* &passwd) {

  ESP_LOGI("WLAN","ssid: %s, passwd: %s", ssid, passwd);

  return ESP_FAIL;
}


void Wlan::_wifiEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data){

    /* no multiple events at the same time.*/
    std::lock_guard<std::mutex> guard(wifi_driver_callback_mutex);
    /* NOTE: _wlanIfaceState is protected by this different mutex, so also need this.*/
    std::lock_guard<std::mutex> lock(wifi_driver_mutex);

    if(IP_EVENT == event_base){

        const ip_event_t event_type{static_cast<ip_event_t>(event_id)};

        switch(event_type)
        {
        case IP_EVENT_STA_GOT_IP:
        {
            _wlanIfaceState = tWlanState::wlanState_connected;
            ESP_LOGI("WLAN_EVENT","Is connected : got IP");
            break;
        }

        case IP_EVENT_STA_LOST_IP:
        {
            _wlanIfaceState = tWlanState::wlanState_waitingForIP;
            ESP_LOGI("WLAN_EVENT","IP lost : waiting for IP");
            break;
        }

        default:
            ESP_LOGW("WLAN_EVENT","unexpected IP EVENT type: %d",event_type);
            break;
        }

    }else if(WIFI_EVENT == event_base){
        const wifi_event_t event_type{static_cast<wifi_event_t>(event_id)};

        switch(event_type)
        {
        case WIFI_EVENT_STA_START:
        {
            _wlanIfaceState = tWlanState::wlanState_connecting;
            ESP_LOGI("WLAN_EVENT","Connecting : STA start");
            break;
        }

        case WIFI_EVENT_STA_CONNECTED:
        {
            _wlanIfaceState = tWlanState::wlanState_waitingForIP;
            ESP_LOGI("WLAN_EVENT","Connecting : waiting for IP");
            break;
        }

        case WIFI_EVENT_HOME_CHANNEL_CHANGE:
        {
            ESP_LOGI("WLAN_EVENT","Home channel change");
            break;
        }

        case WIFI_EVENT_SCAN_DONE:
        {
            ESP_LOGI("WLAN_EVENT","Scan done!");
            break;
        }

        case WIFI_EVENT_STA_DISCONNECTED:
        {
            ESP_LOGI("WLAN_EVENT","STA disconnected");
            break;
        }

        case WIFI_EVENT_STA_STOP:
        {
            ESP_LOGI("WLAN_EVENT","STA stopped");
            break;
        }

        default:
            ESP_LOGW("WLAN_EVENT","unexpected WIFI EVENT type: %d",event_type);
            break;
        }
    }
    else{
        ESP_LOGI("WLAN_EVENT","unexpected callback type: %s",event_base);
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
        
        constexpr char* auth_mode_log_str[] = {
                "OPEN",             /**< authenticate mode : open */
                "WEP",              /**< authenticate mode : WEP */
                "WPA_PSK",          /**< authenticate mode : WPA_PSK */
                "WPA2_PSK",         /**< authenticate mode : WPA2_PSK */
                "WPA_WPA2_PSK",     /**< authenticate mode : WPA_WPA2_PSK */
                "ENTERPRISE",       /**< authenticate mode : WiFi EAP security */
                "WPA2_ENTERPRISE",  /**< authenticate mode : WiFi EAP security */
                "WPA3_PSK",         /**< authenticate mode : WPA3_PSK */
                "WPA2_WPA3_PSK",    /**< authenticate mode : WPA2_WPA3_PSK */
                "WAPI_PSK",         /**< authenticate mode : WAPI_PSK */
                "OWE",              /**< authenticate mode : OWE */
                "WPA3_ENT_192",     /**< authenticate mode : WPA3_ENT_SUITE_B_192_BIT */
                "WPA3_EXT_PSK",     /**< this authentication mode will yield same result as WIFI_AUTH_WPA3_PSK and not recommended to be used. It will be deprecated in future, please use WIFI_AUTH_WPA3_PSK instead. */
                "WPA3_EXT_PSK_MIXED_MODE", /**< this authentication mode will yield same result as WIFI_AUTH_WPA3_PSK and not recommended to be used. It will be deprecated in future, please use WIFI_AUTH_WPA3_PSK instead.*/
                "DPP"              /**< authenticate mode : DPP */
        };

        for (int i = 0; i < number; i++) {

            char mac_cstr[12+5+1]{'\0'};
            snprintf(mac_cstr, sizeof(mac_cstr), "%02x:%02x:%02x:%02x:%02x:%02x",
                ap_info[i].bssid[0],
                ap_info[i].bssid[1],
                ap_info[i].bssid[2],
                ap_info[i].bssid[3],
                ap_info[i].bssid[4],
                ap_info[i].bssid[5]
            );
            
            ESP_LOGI("", "%s -- %-33s %-23s RSSI %d CH: %02d FTM:%d/%d",mac_cstr, ap_info[i].ssid, auth_mode_log_str[(ap_info[i].authmode)], ap_info[i].rssi, ap_info[i].primary, ap_info[i].ftm_initiator, ap_info[i].ftm_responder);
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
            
            for(uint16_t i=0; i< (30000/5000); i++){
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                /*wifiIF.fastScan();*/
            }

            wifi_ap_record_t ap;
            esp_wifi_sta_get_ap_info(&ap);
            printf("%d\n", ap.rssi);
            printf("FTM responder %d\n", ap.ftm_responder);
            printf("FTM initiator %d\n", ap.ftm_initiator);

           /*wifiIF.disconnect_power_off();
            vTaskDelay(20000 / portTICK_PERIOD_MS);
            wifiIF.begin();*/

            const char* ssid{"ssid"};
            const char* pass{"pass"};
            wifiIF.sta_connect(ssid, pass);
        }  
}

} /* NETWORK */