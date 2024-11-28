#include "network_manager.h"

/* static member init */
NETWORK::Wlan wlan_interface;

namespace NETWORK
{
    
    
    Wlan::tWlanState Wlan::_wlanIfaceState{Wlan::tWlanState::wlanState_notInitialised};
    std::mutex Wlan::wifi_driver_mutex{};
    std::mutex Wlan::wifi_driver_callback_mutex{};
    std::array<Wlan::wlan_scan_devices_t, (Wlan::maximum_number_of_APs + Wlan::maimum_number_of_STAs) > Wlan::wlan_device_list;
    std::array<Wlan::wlan_access_point_id_t,1>  access_point_list = {
        {"TS-uG65", "4XfuPgEx", 0, 0, 90}
    };
    wifi_init_config_t Wlan::_wifiInitCfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t Wlan::_wifiConfig{};
    Wlan::wifi_power_save_e Wlan::wifi_power_save_mode{Wlan::wifi_power_save_e::psave_maximum};

Wlan::Wlan(){
    ESP_LOGI(log_label,"WLAN constructor called.");
};

/* FROM: https://github.com/espressif/esp-idf/blob/master/examples/wifi/getting_started/station/main/station_example_main.c */
/* FROM: https://www.youtube.com/watch?v=3dM6LiAriEg */ 
esp_err_t Wlan::init(void){

   /*std::lock_guard<std::mutex> guard(wifi_driver_mutex);*/
    esp_err_t retStatus = ESP_OK;

    if( tWlanState::wlanState_notInitialised == _wlanIfaceState){

        /* NOTE: These two break and reboot the RTOS when called twice, so init moved to main::init */
        /*retStatus = esp_netif_init();*/
        /*esp_event_loop_create_default()*/

        /* register wifi event handler functions */
        ESP_LOGI(NETWORK::Wlan::log_label, "registering wifi handlers");
        if (ESP_OK == retStatus)
        {
            retStatus = esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&_wifiEventHandler,nullptr,nullptr);
        }
        if (ESP_OK == retStatus)
        {
            retStatus = esp_event_handler_instance_register(IP_EVENT,ESP_EVENT_ANY_ID,&_wifiEventHandler,nullptr,nullptr);
        }

        if(ESP_OK == retStatus){
            p_netif = esp_netif_create_default_wifi_sta();

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
                ESP_LOGW(log_label, "SSID too long, will be trancated to buffer size!");
            }
            if(strlen(wlanPassword) > sizeof(_wifiConfig.sta.password)){
                ESP_LOGW(log_label, "SSID too long, will be trancated to buffer size!");
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

esp_err_t Wlan::disconnect_power_off(void)
{

    constexpr uint16_t lock_attempt_timeout_ms{1000};
    constexpr uint16_t islocked_poll_ms{10};

    esp_err_t retStatus = ESP_OK;

    std::unique_lock<std::mutex> lock(wifi_driver_mutex, std::try_to_lock);
    for (uint16_t i = 0; ((!lock.owns_lock()) && (i < (lock_attempt_timeout_ms / islocked_poll_ms))); i++)
    {
        vTaskDelay(islocked_poll_ms / portTICK_PERIOD_MS);
    }

    if (lock.owns_lock())
    {

        if (tWlanState::wlanState_notInitialised != _wlanIfaceState)
        {
             ESP_LOGI(NETWORK::Wlan::log_label, "disconnecting");
            retStatus = esp_wifi_disconnect(); /* Graceful disconnect from AP, or NOT connceted/ No init return if not connected*/
            _wlanIfaceState = tWlanState::wlanState_disconnected;

            if ((ESP_ERR_WIFI_NOT_STARTED != retStatus) /*&& (ESP_ERR_WIFI_NOT_INIT != retStatus)*/)
            {
                ESP_LOGI(NETWORK::Wlan::log_label, "stopping wlan iface");
                esp_wifi_stop(); /* stop STA mode, dont care about return*/
                /*esp_wifi_deinit();*/
               /*esp_wifi_clear_default_wifi_driver_and_handlers(p_netif);*/ // <-add this!
                /*esp_netif_destroy(p_netif);*/
                _wlanIfaceState = tWlanState::wlanState_initCompleted;
    
                // esp_wifi_deinit();  /* clear init and stop wifi task - next time, init is needed again.*/
                
            }
            retStatus = ESP_OK;
            _wlanIfaceState = tWlanState::wlanState_initCompleted;
        }
    }
    else
    {

        // mutex wasn't locked. Handle it.
        ESP_LOGW(NETWORK::Wlan::log_label, "disconnect_power_off() - unable to lock mutex");
        return ESP_ERR_TIMEOUT;
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

void Wlan::_wifiEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    constexpr uint16_t lock_attempt_timeout_ms{100};
    constexpr uint16_t islocked_poll_ms{1};

    /* no multiple events at the same time.*/
    /* NOTE: _wlanIfaceState is protected by this different mutex, so also need this.*/

    std::unique_lock<std::mutex> wlan_event_callback_lock(wifi_driver_callback_mutex, std::try_to_lock);

    for (uint16_t i = 0; ((!wlan_event_callback_lock.owns_lock()) && (i < (lock_attempt_timeout_ms / islocked_poll_ms))); i++)
        {
            vTaskDelay(islocked_poll_ms / portTICK_PERIOD_MS);
        }

    if (wlan_event_callback_lock.owns_lock())
    {

        std::unique_lock<std::mutex> wlan_driver_lock(wifi_driver_mutex, std::try_to_lock);
        for (uint16_t i = 0; ((!wlan_driver_lock.owns_lock()) && (i < (lock_attempt_timeout_ms / islocked_poll_ms))); i++)
        {
            vTaskDelay(islocked_poll_ms / portTICK_PERIOD_MS);
        }

        if (wlan_driver_lock.owns_lock())
        {
            if (IP_EVENT == event_base)
            {
                const ip_event_t event_type{static_cast<ip_event_t>(event_id)};

                switch (event_type)
                {
                case IP_EVENT_STA_GOT_IP:
                {
                    _wlanIfaceState = tWlanState::wlanState_connected;
                    break;
                }

                case IP_EVENT_STA_LOST_IP:
                {
                    _wlanIfaceState = tWlanState::wlanState_waitingForIP;
                    break;
                }

                default:
                    /* do nothing */
                    break;
                }
            }
            else if (WIFI_EVENT == event_base)
            {
                const wifi_event_t event_type{static_cast<wifi_event_t>(event_id)};

                switch (event_type)
                {
                case WIFI_EVENT_STA_START:
                {
                    _wlanIfaceState = tWlanState::wlanState_connecting;
                    break;
                }

                case WIFI_EVENT_STA_CONNECTED:
                {
                    _wlanIfaceState = tWlanState::wlanState_waitingForIP;
                    break;
                }
                default:
                    /* do nothing */
                    break;
                }
            }
        }
        else
        {
            ESP_LOGE(NETWORK::Wlan::log_label, "wlan event cannot lock wifi driver");
            return;
        }

        /* Print events to dbg terminal - this will take longer but does not need lock so will not block */

        if(IP_EVENT == event_base){
            const ip_event_t event_type{static_cast<ip_event_t>(event_id)};

            ESP_LOGI(NETWORK::Wlan::log_label, "IP_EVENT::%s", ip_event_to_string(event_type));
        }
        else if(WIFI_EVENT == event_base){
            const wifi_event_t event_type{static_cast<wifi_event_t>(event_id)};

            ESP_LOGI(NETWORK::Wlan::log_label, "WIFI_EVENT::%s", wifi_event_to_string(event_type));
        }
        else{
            ESP_LOGW(NETWORK::Wlan::log_label, "unexpected event type: %s", event_base);
        }
    }
    else
    {

        ESP_LOGW(NETWORK::Wlan::log_label, "wlan event dropped. handler busy based on mutex");
        return;
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

        ESP_LOGI(log_label,"MAC address loaded: %s",macAddressCStr);
    }
    else{
        ESP_LOGI(log_label,"mac already loaded: %s",macAddressCStr);
    }

    return retStatus;
}

const char * wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type)
{
  switch(type) {
  case WIFI_PKT_MGMT: return "MGMT";
  case WIFI_PKT_DATA: return "DATA";
  default:  
  case WIFI_PKT_MISC: return "MISC";
  }
}

void Wlan::print_collected_device(){

    for(uint8_t print_channel_group = 0; print_channel_group < 13; print_channel_group++){

        ESP_LOGI("\x1b[34m", "CH: %2d ---- ---- ---- ---- ---- ----", print_channel_group);
        for(uint8_t i=0; i < maximum_number_of_APs; i++){

            if((true == wlan_device_list[i].isAP) && (print_channel_group == wlan_device_list[i].channel)){

                ESP_LOGI("\x1b[34m", "---v--  %12llx - %12llx RSSI %.0f CH: %02d last scan:%d ssid: %-33s", 
                wlan_device_list[i].device_mac,
                wlan_device_list[i].associated_mac,
                wlan_device_list[i].rssi, wlan_device_list[i].channel,
                wlan_device_list[i].scan_cycle_since_detect,
                wlan_device_list[i].ssid
                );

                /*ESP_LOGE(NETWORK::Wlan::log_label, "%s CH: %d mac: %llx, assoc: %llx rssi: %.2f detected: %d",wlan_device_list[i].ssid, wlan_device_list[i].channel, , wlan_device_list[i].associated_mac, wlan_device_list[i].rssi, wlan_device_list[i].scan_cycle_since_detect);*/

                for(uint8_t j =0; j < (maximum_number_of_APs + maimum_number_of_STAs); j++){
                    if((wlan_device_list[j].associated_mac == wlan_device_list[i].device_mac) && (false == wlan_device_list[j].isAP)){

                        /*ESP_LOGE("\x1b[34m", "l--> %s: mac: %llx, assoc: %llx rssi: %.2f detected: %d", (true == wlan_device_list[j].isAP) ? "AP " : "DEV", wlan_device_list[j].device_mac, wlan_device_list[j].associated_mac, wlan_device_list[j].rssi, wlan_device_list[i].scan_cycle_since_detect);*/

                        ESP_LOGI("\x1b[34m", "   l--> %12llx - %12llx RSSI %.0f CH: %02d last scan:%d", 
                        wlan_device_list[j].device_mac,
                        wlan_device_list[j].associated_mac,
                        wlan_device_list[j].rssi, 
                        wlan_device_list[j].channel,
                        wlan_device_list[j].scan_cycle_since_detect
                        );
                    }
                }
            }
        }

        for(uint8_t i=0; i < (maximum_number_of_APs + maimum_number_of_STAs); i++){
            if((false == wlan_device_list[i].isAP) && (0 != wlan_device_list[i].device_mac) && (print_channel_group == wlan_device_list[i].channel) && (wlan_device_list[i].device_mac != wlan_device_list[i].associated_mac)){
               /* ESP_LOGE("\x1b[34m", "   > %s: mac: %llx, assoc: %llx rssi: %.2f detected: %d", (true == wlan_device_list[i].isAP) ? "AP " : "DEV", wlan_device_list[i].device_mac, wlan_device_list[i].associated_mac, wlan_device_list[i].rssi, wlan_device_list[i].scan_cycle_since_detect);*/
                if(0 == wlan_device_list[i].associated_mac){
                    ESP_LOGI("\x1b[34m", "      x %12llx - %12llx RSSI %.0f CH: %02d last scan:%d", 
                        wlan_device_list[i].device_mac,
                        wlan_device_list[i].associated_mac,
                        wlan_device_list[i].rssi, 
                        wlan_device_list[i].channel,
                        wlan_device_list[i].scan_cycle_since_detect
                        );
                }
                else{
                    ESP_LOGI("\x1b[34m", "     ~> %12llx - %12llx RSSI %.0f CH: %02d last scan:%d", 
                        wlan_device_list[i].device_mac,
                        wlan_device_list[i].associated_mac,
                        wlan_device_list[i].rssi, 
                        wlan_device_list[i].channel,
                        wlan_device_list[i].scan_cycle_since_detect
                        );
                }
                
            }
        }
    }

}

uint16_t Wlan::count_sta_on_channel(uint8_t channel){   

    uint16_t num_of_devices = 0;

    for(uint8_t i =0; i < (maximum_number_of_APs + maimum_number_of_STAs); i++){
        if((wlan_device_list[i].device_mac != 0) && (wlan_device_list[i].channel == channel) && (false == wlan_device_list[i].isAP)){
            num_of_devices++;
        }
    }
    return num_of_devices;
}

uint16_t Wlan::count_devices_connected_to_AP(uint64_t ap_mac){
    uint16_t num_of_devices = 0;

    for(uint8_t i =0; i < (maximum_number_of_APs + maimum_number_of_STAs); i++){
        if((wlan_device_list[i].device_mac != 0) && (wlan_device_list[i].associated_mac == ap_mac)){
            num_of_devices++;
        }
    }
    return num_of_devices;
}

uint16_t  Wlan::count_all_sta(void){
    uint16_t num_of_devices = 0;

    for(uint8_t i =0; i < (maximum_number_of_APs + maimum_number_of_STAs); i++){
        if((wlan_device_list[i].device_mac != 0) && (false == wlan_device_list[i].isAP)){
            num_of_devices++;
        }
    }
    return num_of_devices;
}

uint16_t  Wlan::count_ap_on_channel(uint8_t channel){
    uint16_t num_of_devices = 0;

    for(uint8_t i =0; i < (maximum_number_of_APs + maimum_number_of_STAs); i++){
        if((wlan_device_list[i].device_mac != 0) && (wlan_device_list[i].channel == channel) && (true == wlan_device_list[i].isAP)){
            num_of_devices++;
        }
    }
    return num_of_devices;
}

uint16_t  Wlan::count_all_ap(void){
    uint16_t num_of_devices = 0;

    for(uint8_t i =0; i < (maximum_number_of_APs + maimum_number_of_STAs); i++){
        if((wlan_device_list[i].device_mac != 0) && (true == wlan_device_list[i].isAP)){
            
            // ESP_LOGI("\x1b[34m", "isap: %s ssid: %s %12llx - %12llx RSSI %.0f CH: %02d last scan:%d", 
            //             ((true == wlan_device_list[i].isAP) ? "AP" : "STA"),
            //             wlan_device_list[i].ssid,
            //             wlan_device_list[i].device_mac,
            //             wlan_device_list[i].associated_mac,
            //             wlan_device_list[i].rssi, 
            //             wlan_device_list[i].channel,
            //             wlan_device_list[i].scan_cycle_since_detect
            //             );
            num_of_devices++;
        }
    }
    return num_of_devices;
}

esp_err_t Wlan::collect_wifi_device(Wlan::wlan_scan_devices_t pckt_source_device){

        if((pckt_source_device.device_mac > 0) && (pckt_source_device.device_mac != pckt_source_device.associated_mac)){    /* if both macs are the same, this is a packet from AP, disregard.*/
            /* check if address is already in device list */
            auto found_obj = std::find_if(Wlan::wlan_device_list.begin(), Wlan::wlan_device_list.end(), [pckt_source_device](const wlan_scan_devices_t& obj) {
                return obj.device_mac == pckt_source_device.device_mac;
            });

            if(found_obj == Wlan::wlan_device_list.end()){

                /* insert new device into array, at the first place where mac is 0 meaning empty slot */
                auto it = std::find_if(Wlan::wlan_device_list.begin(), Wlan::wlan_device_list.end(), [](const wlan_scan_devices_t &obj)
                { return obj.device_mac == 0; });

                if(it != Wlan::wlan_device_list.end()){

                    ESP_LOGI(NETWORK::Wlan::log_label_scan, "+STA: %llx CH:%d", pckt_source_device.device_mac, pckt_source_device.channel);
                    it->device_mac = pckt_source_device.device_mac;
                    it->associated_mac = pckt_source_device.associated_mac;
                    it->channel = pckt_source_device.channel;
                    it->isAP = false;
                    it->rssi = pckt_source_device.rssi;
                    it->scan_cycle_since_detect = 0;
                    return ESP_OK;
                }
                else{

                    ESP_LOGI(NETWORK::Wlan::log_label, "no space left for device");
                    return ESP_FAIL;
                }

            }
            else{

                if(0 != pckt_source_device.device_mac){
                    found_obj->associated_mac = pckt_source_device.device_mac;
                }
                found_obj->scan_cycle_since_detect = 0;
            }
        }
        #if 1 /* ignore devices that supposedly exist based on that they get packets addressed to, maybe they are far, but too many devices as is */
        if(pckt_source_device.associated_mac > 0){

            bool is_access_point = false;
            if((pckt_source_device.device_mac == pckt_source_device.associated_mac)){
                is_access_point = true;
            }
            /* check if address is already in device list */
            auto found_obj = std::find_if(Wlan::wlan_device_list.begin(), Wlan::wlan_device_list.end(), [pckt_source_device](const wlan_scan_devices_t& obj) {
                return obj.device_mac == pckt_source_device.associated_mac;
            });

            if(found_obj == Wlan::wlan_device_list.end()){

                /* insert indirectly detected new device into array, at the first place where mac is 0 meaning empty slot */
                auto it = std::find_if(Wlan::wlan_device_list.begin(), Wlan::wlan_device_list.end(), [](const wlan_scan_devices_t &obj)
                { return obj.device_mac == 0; });

                
                if(it != Wlan::wlan_device_list.end()){

                    /* do not add hearsay access point because there are way too many of them - (false == is_access_point)*/
                    if((false == is_access_point)){
                        ESP_LOGI(NETWORK::Wlan::log_label_scan, "+%s: %llx CH:%d (indirectly)", (true == is_access_point) ? "AP" : "STA", pckt_source_device.device_mac, pckt_source_device.channel);
                        it->device_mac = pckt_source_device.associated_mac;
                        it->associated_mac = pckt_source_device.device_mac;
                        it->channel = pckt_source_device.channel;
                        it->isAP = is_access_point;
                        it->rssi = -99.0;
                        if(true == is_access_point){
                            it->ssid = "Unknown AP";
                        }
                        it->scan_cycle_since_detect = 0;
                        return ESP_OK;
                    }
                }
                else{

                    ESP_LOGI(NETWORK::Wlan::log_label, "no space left for device");
                    return ESP_FAIL;
                }

            }
            else{

               if(0 != pckt_source_device.device_mac){
                found_obj->associated_mac = pckt_source_device.device_mac;
               }
               found_obj->scan_cycle_since_detect = 0;

            }
        }
        #endif

        return ESP_OK;
    }

void Wlan::wlan_promiscuous_handler(void *buff, wifi_promiscuous_pkt_type_t type)
{

    /*if ((type != WIFI_PKT_DATA) && (type != WIFI_PKT_MGMT))
        return;*/

    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

    wlan_scan_devices_t pckt_source_device;

    pckt_source_device.device_mac = ((uint64_t)hdr->addr2[0] << 40) | ((uint64_t)hdr->addr2[1] << 32) | ((uint64_t)hdr->addr2[2] << 24) | ((uint64_t)hdr->addr2[3] << 16) | ((uint64_t)hdr->addr2[4] << 8) | hdr->addr2[5];
    if(pckt_source_device.device_mac >= 0xffffffffffff){
        pckt_source_device.device_mac = 0;
    }

    pckt_source_device.associated_mac = ((uint64_t)hdr->addr3[0] << 40) | ((uint64_t)hdr->addr3[1] << 32) | ((uint64_t)hdr->addr3[2] << 24) | ((uint64_t)hdr->addr3[3] << 16) | ((uint64_t)hdr->addr3[4] << 8) | hdr->addr3[5];
    if(pckt_source_device.associated_mac >= 0xffffffffffff){
        pckt_source_device.associated_mac = 0;
    }

    pckt_source_device.rssi = ppkt->rx_ctrl.rssi;
    pckt_source_device.channel = ppkt->rx_ctrl.channel;
    collect_wifi_device(pckt_source_device);
}

/* Initialize Wi-Fi as sta and set scan method */
esp_err_t Wlan::fastScan(void)
{
    constexpr char *auth_mode_log_str[] = {
        "OPEN",                    /**< authenticate mode : open */
        "WEP",                     /**< authenticate mode : WEP */
        "WPA_PSK",                 /**< authenticate mode : WPA_PSK */
        "WPA2_PSK",                /**< authenticate mode : WPA2_PSK */
        "WPA_WPA2_PSK",            /**< authenticate mode : WPA_WPA2_PSK */
        "ENTERPRISE",              /**< authenticate mode : WiFi EAP security */
        "WPA2_ENTERPRISE",         /**< authenticate mode : WiFi EAP security */
        "WPA3_PSK",                /**< authenticate mode : WPA3_PSK */
        "WPA2_WPA3_PSK",           /**< authenticate mode : WPA2_WPA3_PSK */
        "WAPI_PSK",                /**< authenticate mode : WAPI_PSK */
        "OWE",                     /**< authenticate mode : OWE */
        "WPA3_ENT_192",            /**< authenticate mode : WPA3_ENT_SUITE_B_192_BIT */
        "WPA3_EXT_PSK",            /**< this authentication mode will yield same result as WIFI_AUTH_WPA3_PSK and not recommended to be used. It will be deprecated in future, please use WIFI_AUTH_WPA3_PSK instead. */
        "WPA3_EXT_PSK_MIXED_MODE", /**< this authentication mode will yield same result as WIFI_AUTH_WPA3_PSK and not recommended to be used. It will be deprecated in future, please use WIFI_AUTH_WPA3_PSK instead.*/
        "DPP"                      /**< authenticate mode : DPP */
    };

    esp_err_t retStatus = ESP_OK;
    uint16_t num_of_active_ap_scanned = maximum_number_of_APs;
        wifi_ap_record_t ap_info[maximum_number_of_APs];
        uint16_t ap_count = 0;

    if (tWlanState::wlanState_notInitialised == _wlanIfaceState)
    {
        retStatus = this->init();
    }

    /* NOTE: W (609) wifi:sta_scan: STA is connecting, scan are not allowed!*/
    if (tWlanState::wlanState_rdyToConnect == _wlanIfaceState ||
        tWlanState::wlanState_connecting == _wlanIfaceState ||
        tWlanState::wlanState_waitingForIP == _wlanIfaceState)
    {
        retStatus = ESP_FAIL;

        ESP_LOGI(NETWORK::Wlan::log_label, "STA is connecting, scan are not allowed!");
    }

    if (ESP_OK == retStatus)
    {
        

        constexpr const uint16_t scan_time_per_channel_ms{1200};
        constexpr const uint16_t scan_time_home_dwell_ms{250};
        constexpr const uint16_t scan_time_extra_margin_ms{2000};

        constexpr const wifi_scan_config_t scan_config = { 
            .show_hidden = true, 
            .scan_type = WIFI_SCAN_TYPE_PASSIVE, 
            .scan_time = {.passive = scan_time_per_channel_ms}, 
            .home_chan_dwell_time = scan_time_home_dwell_ms
        };

        ESP_LOGI(NETWORK::Wlan::log_label_scan, "Starting scan: Passive AP scan + Promiscous STA nearby scan");
        ESP_LOGI(NETWORK::Wlan::log_label_scan, "Scan will take: %d ms", ((13 * (scan_time_per_channel_ms + scan_time_home_dwell_ms)) + scan_time_extra_margin_ms));

        esp_wifi_set_promiscuous_rx_cb(&wlan_promiscuous_handler);
        esp_wifi_set_promiscuous(true);

        /* create space for AP scans */
        memset(ap_info, 0, sizeof(ap_info));
        retStatus = esp_wifi_scan_start(&scan_config, true);
        if(ESP_OK == retStatus){    /* scan has strated succesfully */
            /* scan is ongoing wait for it to finish. Could use wifi_event::SCAN_FINISHED but do not trust it completely so keep it simple.*/
            vTaskDelay(((12 * (scan_time_per_channel_ms + scan_time_home_dwell_ms)) + scan_time_extra_margin_ms) / portTICK_PERIOD_MS);
        }
        /* scan finished, turn off promiscous mode. */
        esp_wifi_set_promiscuous(false);
    }
    if (ESP_OK == retStatus)
    {
        retStatus = esp_wifi_scan_get_ap_records(&num_of_active_ap_scanned, ap_info);
    }
    if (ESP_OK == retStatus)
    {
        /* new scan results are ready, before updating devices list, prune the list of old or weak devices */
        for (uint8_t i = 0; i < (maximum_number_of_APs + maimum_number_of_STAs); i++)
        {
            wlan_device_list[i].scan_cycle_since_detect++;

            /* prune the list - delete old scans */
            if (wlan_device_list[i].scan_cycle_since_detect >= 8)
            {

                wlan_device_list[i].associated_mac = 0;
                wlan_device_list[i].device_mac = 0;
                wlan_device_list[i].isAP = false;
                wlan_device_list[i].channel = 0;
                wlan_device_list[i].ssid = nullptr;
            }
        }

        for (int i = 0; (i < num_of_active_ap_scanned) && (i < maximum_number_of_APs); i++)
        {

            // char mac_cstr[12 + 5 + 1]{'\0'};
            // snprintf(mac_cstr, sizeof(mac_cstr), "%02x:%02x:%02x:%02x:%02x:%02x",
            //          ap_info[i].bssid[0],
            //          ap_info[i].bssid[1],
            //          ap_info[i].bssid[2],
            //          ap_info[i].bssid[3],
            //          ap_info[i].bssid[4],
            //          ap_info[i].bssid[5]);

            // ESP_LOGI("\x1b[34m", "%s -- %-33s %-23s RSSI %d CH: %02d FTM:%d/%d", mac_cstr, ap_info[i].ssid, auth_mode_log_str[(ap_info[i].authmode)], ap_info[i].rssi, ap_info[i].primary, ap_info[i].ftm_initiator, ap_info[i].ftm_responder);
        
            wlan_device_list[i].associated_mac = 0;
            wlan_device_list[i].device_mac = 0;
            wlan_device_list[i].device_mac = ((uint64_t)ap_info[i].bssid[0] << 40) | ((uint64_t)ap_info[i].bssid[1] << 32) | ((uint64_t)ap_info[i].bssid[2] << 24) | ((uint64_t)ap_info[i].bssid[3] << 16) | ((uint64_t)ap_info[i].bssid[4] << 8) | ap_info[i].bssid[5];
            wlan_device_list[i].associated_mac = 0;
            wlan_device_list[i].rssi = ap_info[i].rssi;
            wlan_device_list[i].isAP = true;
            wlan_device_list[i].channel = ap_info[i].primary;
            wlan_device_list[i].ssid = (char*)ap_info[i].ssid;  /* NOTE: uint8_t* to char* so no fuss */
            wlan_device_list[i].scan_cycle_since_detect = 0;
        }

        ESP_LOGI(NETWORK::Wlan::log_label_scan, "Scan has finished.");
        if(maximum_number_of_APs > num_of_active_ap_scanned){
            ESP_LOGI(NETWORK::Wlan::log_label_scan, "number of APs = %u", num_of_active_ap_scanned);
        }
        else{
            ESP_LOGW(NETWORK::Wlan::log_label_scan, "number of APs is greater than %u, no more slots in memory!", num_of_active_ap_scanned);
        }

        /* process the results of promiscous scan - to get the number of devices on the same channel, or neerby*/
        statistics.num_of_ap_current_channel = 2;
        statistics.num_of_dev = count_all_sta();
        statistics.num_of_dev_current_channel = count_sta_on_channel(statistics.num_of_ap_current_channel);
        statistics.num_of_dev_connected_to_ap = count_devices_connected_to_AP(0x3c5447fd788c);
        statistics.num_of_ap = num_of_active_ap_scanned;

        ESP_LOGI(NETWORK::Wlan::log_label_scan, "number of devices connected via this AP: %3d", statistics.num_of_dev_connected_to_ap);
        ESP_LOGI(NETWORK::Wlan::log_label_scan, "number of devices on CH%2d: STA: %3d, AP: %3d", statistics.num_of_ap_current_channel, statistics.num_of_dev_current_channel, statistics.num_of_ap_current_channel);
        ESP_LOGI(NETWORK::Wlan::log_label_scan, "number of devices near me: STA: %3d, AP: %3d", statistics.num_of_dev, statistics.num_of_ap);
        print_collected_device();
        
    }

    if (ESP_OK != retStatus)
    {
        ESP_LOGE(NETWORK::Wlan::log_label, "Scan returned /w fail status.");
    }
    return retStatus;
}

void task_manageWlanConnection(void *parameters){

        /*wlan_interface.loadMACAddress();*/

        constexpr uint16_t WIFI_CONNECTION_CHECK_DELAY_SEC{5};
        constexpr uint16_t WIFI_CONNECTION_ATTEMPT_TIMEOUT_SEC{20};
        constexpr uint16_t WIFI_PAUSE_BEFORE_RECONNECT_ATTEMPT_SEC{5};
        constexpr uint16_t WIFI_NETWORK_STATISTICS_SCAN_PERIOD_SEC{60};

        ESP_LOGI(NETWORK::Wlan::log_label, "wifi iface manage start");

        wlan_interface.statistics.connection_ok_sec = 0;
        wlan_interface.wifi_trying_to_connect = 0;
        wlan_interface.begin();

        for(;;){
            
            if(Wlan::tWlanState::wlanState_rdyToConnect > wlan_interface.get_wlan_state()){

               wlan_interface.statistics.connection_ok_sec = 0;
               wlan_interface.wifi_trying_to_connect = 0;
               wlan_interface.begin();
            } 
            else if(wlan_interface.get_wlan_state() >= Wlan::tWlanState::wlanState_connected){
                wlan_interface.statistics.connection_ok_sec += WIFI_CONNECTION_CHECK_DELAY_SEC;
                wlan_interface.wifi_trying_to_connect = 0;

                ESP_LOGI(NETWORK::Wlan::log_label, "connected for: %ld", wlan_interface.statistics.connection_ok_sec);
                
            }
            else{
                /* trying to connect, but still not connected */

               /* wlan_interface.statistics.average_connecting_time_sec = ((wlan_interface.statistics.average_connecting_time_sec + wlan_interface.wifi_trying_to_connect)/2.0);*/

                if(wlan_interface.wifi_trying_to_connect <= WIFI_CONNECTION_ATTEMPT_TIMEOUT_SEC){
                    wlan_interface.wifi_trying_to_connect += WIFI_CONNECTION_CHECK_DELAY_SEC;
                }
                else{
                    wlan_interface.statistics.connection_ok_sec = 0;
                    wlan_interface.wifi_trying_to_connect = 0;
                    /* TODO: crude delay, can think of something smarter */
                    
                    ESP_LOGW(NETWORK::Wlan::log_label, "was unable to connect for: %d sec, will retry in: %d secs", WIFI_CONNECTION_ATTEMPT_TIMEOUT_SEC, WIFI_PAUSE_BEFORE_RECONNECT_ATTEMPT_SEC);
                    vTaskDelay((1000) / portTICK_PERIOD_MS);
                    wlan_interface.disconnect_power_off();
                    wlan_interface.statistics.num_of_reconnect++;
                    vTaskDelay((WIFI_PAUSE_BEFORE_RECONNECT_ATTEMPT_SEC * 1000) / portTICK_PERIOD_MS);
                }
            } 

            if(wlan_interface.scan_period_counter > (WIFI_NETWORK_STATISTICS_SCAN_PERIOD_SEC/WIFI_CONNECTION_CHECK_DELAY_SEC)){
                if((wlan_interface.get_wlan_state() == Wlan::tWlanState::wlanState_initCompleted) || (wlan_interface.get_wlan_state() == Wlan::tWlanState::wlanState_connected)){
                    esp_err_t res = ESP_OK; /*wlan_interface.fastScan();*/

                    /* scan is a success. wait for next period */
                    if(ESP_OK == res){
                        wlan_interface.scan_period_counter = 0;
                        /* wlan_interface.disconnect_power_off();*/
                    }
                }
            }
            else{
                wlan_interface.scan_period_counter++;
            }

            

            vTaskDelay((WIFI_CONNECTION_CHECK_DELAY_SEC * 1000) / portTICK_PERIOD_MS);

            // /*wifiIF.disconnect_power_off();
            // vTaskDelay(20000 / portTICK_PERIOD_MS);*/
            // wifiIF.begin();
        }  
}

} /* NETWORK */