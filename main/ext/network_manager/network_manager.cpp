#include "network_manager.h"

class ADC_continous;    /* forward declared because need to pause ADC when wifi is sta_begin ing*/
/* static member init */
NETWORK::Wlan wlan_interface;

namespace NETWORK
{
    
    
    
    Wlan::wlan_state_t Wlan::_wlanIfaceState{Wlan::wlan_state_t::waiting_init};
    Wlan::wlan_manager_stats_t Wlan::statistics;
    std::mutex Wlan::wifi_driver_mutex{};
    std::mutex Wlan::wifi_driver_callback_mutex{};
    std::array<Wlan::wlan_scan_devices_t, (Wlan::maximum_number_of_APs + Wlan::maimum_number_of_STAs) > Wlan::wlan_device_list;
    
    bool Wlan::wifi_ext_task_inhibit_flag = true;
    uint8_t Wlan::associated_ap_index = 0;
    std::array<Wlan::wlan_access_point_id_t,3>  Wlan::access_point_list = {{
        { .ssid = "", .passwd = "", .device_mac = 0x00, .channel = 0x00, .low_rssi_tresh = -99, .priority = 0 },
        { .ssid = "adsl sandor", .passwd = "floriflori", .device_mac = 0x00, .channel = 0x00, .low_rssi_tresh = -99, .priority = 0 },
        { .ssid = "GGGGG", .passwd = "123456789", .device_mac = 0x00, .channel = 0x00, .low_rssi_tresh = -99, .priority = 0 }
    }};

    wifi_init_config_t Wlan::_wifiInitCfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t Wlan::_wifiConfig{};
    Wlan::wifi_power_save_e Wlan::wifi_power_save_mode{Wlan::wifi_power_save_e::psave_disabled/*psave_default*/};    /* power save min*/
    /*esp_wifi_set_ps(WIFI_PS_NONE);*/

    uint64_t Wlan::esp_my_mac = 0x00;

Wlan::Wlan(){
    ESP_LOGI(log_label,"WLAN constructor called.");
};

/* FROM: https://github.com/espressif/esp-idf/blob/master/examples/wifi/getting_started/station/main/station_example_main.c */
/* FROM: https://www.youtube.com/watch?v=3dM6LiAriEg */ 
esp_err_t Wlan::init(void){

   /*std::lock_guard<std::mutex> guard(wifi_driver_mutex);*/
    esp_err_t retStatus = ESP_OK;

    if( wlan_state_t::waiting_init == _wlanIfaceState){

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

            /* replace the ssid and password, if there is a known AP loaded from Nvs */

            if(Nvs_Manager::config_in_use == user_cfg_basic.rdy_for_update){

                snprintf((char*)(_wifiConfig.sta.ssid), 40u, "%s", (char*)user_cfg_basic.ssid);
                snprintf((char*)(_wifiConfig.sta.password), 40u, "%s", (char*)user_cfg_basic.passwd);

                ESP_LOGW(NETWORK::Wlan::log_label, "AP for connection set based on Nvs data: ssid:%s passwd:%s",_wifiConfig.sta.ssid, _wifiConfig.sta.password);
            }
            else{
                if(strlen( Wlan::access_point_list[associated_ap_index].ssid) > sizeof(_wifiConfig.sta.ssid)){
                ESP_LOGW(log_label, "SSID too long, will be trancated to buffer size!");
                }
                if(strlen(Wlan::access_point_list[associated_ap_index].passwd) > sizeof(_wifiConfig.sta.password)){
                ESP_LOGW(log_label, "SSID too long, will be trancated to buffer size!");
                }

                memcpy(_wifiConfig.sta.ssid, Wlan::access_point_list[associated_ap_index].ssid, std::min(strlen(Wlan::access_point_list[associated_ap_index].ssid), sizeof(_wifiConfig.sta.ssid)));
                memcpy(_wifiConfig.sta.password, Wlan::access_point_list[associated_ap_index].passwd, std::min(strlen(Wlan::access_point_list[associated_ap_index].passwd),sizeof(_wifiConfig.sta.password)));
            }
        }

        if(ESP_OK == retStatus)
        {
            esp_wifi_set_config(WIFI_IF_STA, &_wifiConfig);
        }
    }

    return retStatus;
}

esp_err_t Wlan::set_wifi_connection_target(int8_t ap_index){

    esp_err_t retStatus = ESP_FAIL;
    


    // if(ap_index == -1){     /* disconnect wifi and do not connect to any AP*/
        
    //   //  Wlan::_wifiConfig.sta.ssid = {'\0'};
    //   //  Wlan::_wifiConfig.sta.password = {'\0'};

    //     strncpy(Wlan::_wifiConfig.sta.ssid, (char*)"", 32);
    //     strncpy(Wlan::_wifiConfig.sta.password, (char*)"", 64);
        
    // }

    // esp_wifi_disconnect();
    // esp_wifi_set_config(WIFI_IF_STA, &_wifiConfig);
    // esp_wifi_connect();

    return retStatus;
}

esp_err_t Wlan::sta_connect(void){

    esp_err_t retStatus = ESP_OK;

    if(wlan_state_t::waiting_init == _wlanIfaceState){
        retStatus = this->init();
    }

    if(ESP_OK == retStatus){
        
        _wlanIfaceState = wlan_state_t::standby;
        esp_wifi_set_mode(WIFI_MODE_STA);

        esp_wifi_set_max_tx_power(20 * 4);

        retStatus = esp_wifi_set_max_tx_power(80);  // 20 dBm
        if (retStatus != ESP_OK) {
            printf("Failed to set TX power: %s\n", esp_err_to_name(retStatus));
        }
        
        esp_wifi_start();
        retStatus = esp_wifi_connect();
    }
    
    return retStatus;
}

esp_err_t Wlan::start_ap(){
    
    esp_err_t res = ESP_OK;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    esp_netif_create_default_wifi_ap();

    
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    res = esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&_wifiEventHandler,nullptr,nullptr);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "jklello",
            .password = "123456789",
            .ssid_len = strlen("jklello"),
            .channel = 6,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = 1,
            .pmf_cfg = {
                    .required = true,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    _wlanIfaceState = wlan_state_t::ap_active;

    ESP_LOGI("wifi_AP", "wifi_init_softap finished. SSID:%s password:%s channel:%d","jklello", "123456789", 6);

    return res;
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

        if (wlan_state_t::waiting_init != _wlanIfaceState)
        {
             ESP_LOGI(NETWORK::Wlan::log_label, "disconnecting");
            retStatus = esp_wifi_disconnect(); /* Graceful disconnect from AP, or NOT connceted/ No init return if not connected*/
            /*_wlanIfaceState = wlan_state_t::wlanState_disconnected;*/

            if(Wlan::statistics.num_of_reconnect >= 1){
                Wlan::statistics.num_of_reconnect--;
            }
            

            if ((ESP_ERR_WIFI_NOT_STARTED != retStatus) /*&& (ESP_ERR_WIFI_NOT_INIT != retStatus)*/)
            {
                ESP_LOGI(NETWORK::Wlan::log_label, "stopping wlan iface");
                esp_wifi_stop(); /* stop STA mode, dont care about return*/
                /*esp_wifi_deinit();*/
               /*esp_wifi_clear_default_wifi_driver_and_handlers(p_netif);*/ // <-add this!
                /*esp_netif_destroy(p_netif);*/
                _wlanIfaceState = wlan_state_t::inactive;
    
                // esp_wifi_deinit();  /* clear init and stop wifi task - next time, init is needed again.*/
                
            }
            retStatus = ESP_OK;
            _wlanIfaceState = wlan_state_t::inactive;
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
    return this->sta_connect();
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
                    _wlanIfaceState = wlan_state_t::sta_connection_ok;
                    break;
                }

                case IP_EVENT_STA_LOST_IP:
                {
                    _wlanIfaceState = wlan_state_t::sta_wait_for_ip;
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
                    _wlanIfaceState = wlan_state_t::sta_wait_for_connection;
                    break;
                }



                case WIFI_EVENT_STA_CONNECTED:
                {
                    _wlanIfaceState = wlan_state_t::sta_wait_for_ip;
                    Wlan::statistics.num_of_reconnect++;
                    break;
                }

                case WIFI_EVENT_STA_BEACON_TIMEOUT:
                case WIFI_EVENT_STA_DISCONNECTED:
                {
                    /* TODO: check.*/
                    _wlanIfaceState = wlan_state_t::standby;//tWlanState::wlanState_initCompleted;
                    wlan_interface.statistics.connection_ok_sec = 0;
                    break;
                }

                case WIFI_EVENT_AP_STACONNECTED:
                {
                    wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
                    ESP_LOGI(NETWORK::Wlan::log_label, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
                    break;
                }

                case WIFI_EVENT_AP_STADISCONNECTED:
                {
                    wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
                    ESP_LOGI(NETWORK::Wlan::log_label, "station "MACSTR" leave, AID=%d, reason=%d", MAC2STR(event->mac), event->aid, event->reason);
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

            /* on disconnect event:*/
            if(event_type == WIFI_EVENT_STA_DISCONNECTED){
                wifi_event_sta_disconnected_t* disconnect_event_data = static_cast<wifi_event_sta_disconnected_t*>(event_data);
                ESP_LOGI(NETWORK::Wlan::log_label, "EVENT_DATA::%d, %d", disconnect_event_data->rssi, disconnect_event_data->reason);
            }
            
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

    if(0 == Wlan::esp_my_mac){

        uint8_t mac_byte_buffer[6]{};
    
        retStatus = esp_efuse_mac_get_default(mac_byte_buffer);
        if (ESP_OK == retStatus)
        {
            Wlan::esp_my_mac = ((uint64_t)mac_byte_buffer[0] << 40) | ((uint64_t)mac_byte_buffer[1] << 32) | ((uint64_t)mac_byte_buffer[2] << 24) | ((uint64_t)mac_byte_buffer[3] << 16) | ((uint64_t)mac_byte_buffer[4] << 8) | mac_byte_buffer[5];
            ESP_LOGI(log_label,"ESP my device mac addr: %llx",Wlan::esp_my_mac);
        } 
    }
    else{
        ESP_LOGI(log_label,"mac already loaded: %llx",Wlan::esp_my_mac);
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
        for(uint8_t i=0; i < maximum_number_of_APs + maimum_number_of_STAs; i++){

            if((true == wlan_device_list[i].isAP) && (print_channel_group == wlan_device_list[i].channel)){

                wlan_device_list[i].print_ap();

                for(uint8_t j =0; j < (maximum_number_of_APs + maimum_number_of_STAs); j++){
                    if((wlan_device_list[j].associated_mac == wlan_device_list[i].device_mac) && (false == wlan_device_list[j].isAP)){
                        
                        wlan_device_list[j].print_assoc_sta();
                    }
                }
            }
        }

        for(uint8_t i=0; i < (maximum_number_of_APs + maimum_number_of_STAs); i++){
            if((false == wlan_device_list[i].isAP) && (0 != wlan_device_list[i].device_mac) && (print_channel_group == wlan_device_list[i].channel) && (wlan_device_list[i].device_mac != wlan_device_list[i].associated_mac)){
               /* ESP_LOGE("\x1b[34m", "   > %s: mac: %llx, assoc: %llx rssi: %.2f detected: %d", (true == wlan_device_list[i].isAP) ? "AP " : "DEV", wlan_device_list[i].device_mac, wlan_device_list[i].associated_mac, wlan_device_list[i].rssi, wlan_device_list[i].scan_cycle_since_detect);*/
                
                bool associated_w_ap = false;
                for(uint8_t j=0; j < maximum_number_of_APs; j++){
                    if(wlan_device_list[j].device_mac == wlan_device_list[i].associated_mac){
                        associated_w_ap = true;
                        break;
                    }
                }

                if(associated_w_ap == false){
                    if(0 == wlan_device_list[i].associated_mac){
                        wlan_device_list[i].print_no_con_sta();
                    }
                    else{
                        wlan_device_list[i].print_sta();
                    }
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

float Wlan::estimate_ap_dist_from_rssi(int8_t rssi){

    constexpr float ap_txpwr_at1m{-35.0};   /* measured tx power in dBm at 1 meter (just guessing not measured cause lazy) */
    constexpr float path_loss_factor{3.0};

    if((rssi > -99) && (rssi <= 0)){
        return pow(10, (ap_txpwr_at1m - rssi) / (10.0 * path_loss_factor));
    }
    return -1.0;
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

esp_err_t Wlan::add_device_to_dev_list(Wlan::wlan_scan_devices_t new_device){


    Wlan::wlan_scan_devices_t* first_empty_slot = nullptr;

    /* check if device is already present in list */
    auto found_obj = std::find_if(Wlan::wlan_device_list.begin(), Wlan::wlan_device_list.end(), [new_device, &first_empty_slot](wlan_scan_devices_t& obj) {

        if((nullptr == first_empty_slot) && (0 == obj.device_mac)){
            first_empty_slot = &obj;
        }
        
        return obj.device_mac == new_device.device_mac;
    });

    if(found_obj == Wlan::wlan_device_list.end()){  /* device not in device list, add it.*/

        if(first_empty_slot == nullptr){
            /* no space left in list, clear less important devs. */
            ESP_LOGI(NETWORK::Wlan::log_label, "no space left for device");
            return ESP_FAIL;
        }

        first_empty_slot->device_mac = new_device.device_mac;
        first_empty_slot->associated_mac = new_device.associated_mac;
        first_empty_slot->channel = new_device.channel;
        first_empty_slot->isAP = new_device.isAP;
        first_empty_slot->rssi = new_device.rssi;
        first_empty_slot->scan_cycle_since_detect = 0;
        first_empty_slot->ssid = std::string(new_device.ssid);  /* this does deep copy */
        /*first_empty_slot->print_dev_collected();*/

    }
    else{

        if(new_device.rssi != -99){
            found_obj->rssi = new_device.rssi;
        }
        found_obj->associated_mac = new_device.associated_mac;
        found_obj->channel = new_device.channel;
        found_obj->scan_cycle_since_detect = 0;
        found_obj->isAP = new_device.isAP;
        found_obj->ssid = std::string(new_device.ssid);  /* this does deep copy */
    }

    return ESP_OK;
}

esp_err_t Wlan::collect_wifi_device(Wlan::wlan_scan_devices_t pckt_source_device){

        if((pckt_source_device.device_mac > 0) && (pckt_source_device.device_mac != pckt_source_device.associated_mac)){    /* if both macs are the same, this is a packet from AP, disregard.*/

            /* Add devices that are mentioned as captured packet source */
            if(pckt_source_device.device_mac > 0){
                Wlan::add_device_to_dev_list(pckt_source_device);
            }

            /* Add devices that are mentioned as destination address in captured packet, set rssi to -99 as we did not recive packet from them so no idea where they are*/
            if(pckt_source_device.associated_mac > 0){

                uint64_t temp_dev_mac = pckt_source_device.device_mac;
                pckt_source_device.device_mac = pckt_source_device.associated_mac;
                pckt_source_device.associated_mac = temp_dev_mac;
                pckt_source_device.rssi = -99;
                pckt_source_device.isAP = false;
                pckt_source_device.ssid = "";
                Wlan::add_device_to_dev_list(pckt_source_device);
            }
        
        }

        return ESP_OK;
    }

void Wlan::wlan_promiscuous_handler(void *buff, wifi_promiscuous_pkt_type_t type)
{

    /*if ((type != WIFI_PKT_DATA) && (type != WIFI_PKT_MGMT))
        return;*/

    if(type != WIFI_PKT_DATA){
        return;
    }

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
    pckt_source_device.isAP = false;
    collect_wifi_device(pckt_source_device);
}

void Wlan::calculate_nearby_dev_statistics(wlan_manager_stats_t &statistics, wlan_access_point_id_t &associated_ap){
    
    statistics.num_of_ap = 0;
    statistics.num_of_sta = 0;
    statistics.num_of_ap_current_channel = 0;
    statistics.num_of_sta_current_channel = 0;
    statistics.num_of_sta_connected_to_ap = 0;
    statistics.num_of_espressif_devices = 0;

    std::for_each(Wlan::wlan_device_list.begin(),Wlan::wlan_device_list.end(), [&statistics, associated_ap](wlan_scan_devices_t& device) {

        statistics.channel = associated_ap.channel;

        if(0 != device.device_mac){

            if(true == device.isAP){
                statistics.num_of_ap++;

                if(statistics.channel == device.channel){
                    statistics.num_of_ap_current_channel++;
                }
            }
            else{
                statistics.num_of_sta++;

                if(statistics.channel == device.channel){
                    statistics.num_of_sta_current_channel++;

                    if(associated_ap.device_mac == device.associated_mac){
                        statistics.num_of_sta_connected_to_ap++;
                    }
                }
            }

            for(uint16_t i = 0; i < std::size(espressif_macs); i++){
                if((device.device_mac >> 24) == espressif_macs[i]){
                    statistics.num_of_espressif_devices++;
                    break;
                }
            }

          /*ESP_LOGI(NETWORK::Wlan::log_label, "dev: %s", known_device_by_mac(device.device_mac));*/
        } 
    });

    return;
}
/* Initialize Wi-Fi as sta and set scan method */
esp_err_t Wlan::fastScan(void)
{
    esp_err_t retStatus = ESP_OK;
    uint16_t num_of_active_ap_scanned = maximum_number_of_APs;
    wifi_ap_record_t ap_info[maximum_number_of_APs];
    uint16_t ap_count = 0;

    vTaskDelay(100 / portTICK_PERIOD_MS);       /* wait a little bit, to let other tasks pause*/

    if (wlan_state_t::waiting_init == _wlanIfaceState)
    {
        retStatus = this->init();
    }

    /* NOTE: W (609) wifi:sta_scan: STA is connecting, scan are not allowed!*/
    if (wlan_state_t::standby == _wlanIfaceState ||
        wlan_state_t::sta_wait_for_connection == _wlanIfaceState ||
        wlan_state_t::sta_wait_for_ip == _wlanIfaceState)
    {

        
        esp_wifi_disconnect();
       /*esp_wifi_set_config(...);
        esp_wifi_connect();*/
        /*esp_wifi_set_mode(WIFI_MODE_STA);*/
       // esp_wifi_start();
        /*retStatus = ESP_FAIL;*/

        ESP_LOGI(NETWORK::Wlan::log_label, "STA is connecting, scan are not allowed!");
        /*esp_wifi_disconnect();*/
        /*retStatus = esp_wifi_stop();
        _wlanIfaceState = tWlanState::wlanState_initCompleted;*/
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        /*esp_wifi_set_mode(WIFI_MODE_STA);*/
        /*esp_wifi_start();*/
        retStatus = ESP_OK;
    }

    if (ESP_OK == retStatus)
    {
        

        constexpr const uint16_t scan_time_per_channel_ms{500}; /* 500 */
        constexpr const uint16_t scan_time_home_dwell_ms{250};  /* 250 */
        constexpr const uint16_t scan_time_extra_margin_ms{2000};   /* 2000 */

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
        ESP_LOGI(NETWORK::Wlan::log_label_scan, "AP records returned: %d", retStatus);
    }
    if (ESP_OK == retStatus)
    {
        /* new scan results are ready, before updating devices list, prune the list of old or weak devices */
        for (uint16_t i = 0; i < (maximum_number_of_APs + maimum_number_of_STAs); i++)
        {
            wlan_device_list[i].scan_cycle_since_detect++;

            /* prune the list - delete old scans */
            if (wlan_device_list[i].scan_cycle_since_detect >= 8)
            {
                wlan_device_list[i].associated_mac = 0;
                wlan_device_list[i].device_mac = 0;
                wlan_device_list[i].isAP = false;
                wlan_device_list[i].channel = 0;
                wlan_device_list[i].ssid = "";
            }
        }

        for (auto& access_point : access_point_list) {
            access_point.print();
        }

       

        std::for_each(std::begin(ap_info), std::end(ap_info), [&](wifi_ap_record_t& ap_record)
        {

            wlan_scan_devices_t new_ap_device;
            new_ap_device.device_mac = 0x00 | ((uint64_t)ap_record.bssid[0] << 40) | ((uint64_t)ap_record.bssid[1] << 32) | ((uint64_t)ap_record.bssid[2] << 24) | ((uint64_t)ap_record.bssid[3] << 16) | ((uint64_t)ap_record.bssid[4] << 8) | ap_record.bssid[5];

            if( 0x00 != new_ap_device.device_mac){
                
                new_ap_device.associated_mac = 0U;
                new_ap_device.rssi = ap_record.rssi;
                new_ap_device.isAP = true;
                new_ap_device.channel = ap_record.primary;
                new_ap_device.ssid = (char*)ap_record.ssid;
                new_ap_device.scan_cycle_since_detect = 0U;

                for(auto& access_point : Wlan::access_point_list){

                    if(0 == strncmp((char*)ap_record.ssid, access_point.ssid, 30)){
                        access_point.device_mac = new_ap_device.device_mac;
                        access_point.channel = new_ap_device.channel;
                        access_point.last_seen = 0;

                        ESP_LOGI("","AP is currently present: %s, mac: %llx, CH:%d", ap_record.ssid, access_point.device_mac, access_point.channel);
                    }
                }

                /*new_ap_device.print_ap();*/
                Wlan::collect_wifi_device(new_ap_device);
            } 
        });

        // for (int i = 0; (i < 100/*maximum_number_of_APs*/); i++){
        //     wlan_device_list[i].print_ap();
        // }
        
        /* process the results of promiscous scan - to get the number of devices on the same channel, or neerby*/
        calculate_nearby_dev_statistics(Wlan::statistics, Wlan::access_point_list[Wlan::associated_ap_index]);

        ESP_LOGI(NETWORK::Wlan::log_label_scan, "Scan has finished.");
        if(maximum_number_of_APs > num_of_active_ap_scanned){
            ESP_LOGI(NETWORK::Wlan::log_label_scan, "number of APs = %u", num_of_active_ap_scanned);
        }
        else{
            ESP_LOGW(NETWORK::Wlan::log_label_scan, "number of APs is greater than %u, no more slots in memory!", num_of_active_ap_scanned);
        }
        
        ESP_LOGI(NETWORK::Wlan::log_label_scan, "number of devices connected via this AP: %3d", Wlan::statistics.num_of_sta_connected_to_ap);
        ESP_LOGI(NETWORK::Wlan::log_label_scan, "number of devices on CH%2d: STA: %3d, AP: %3d", statistics.channel, Wlan::statistics.num_of_sta_current_channel, statistics.num_of_ap_current_channel);
        ESP_LOGI(NETWORK::Wlan::log_label_scan, "number of devices near me: STA: %3d, AP: %3d", statistics.num_of_sta, Wlan::statistics.num_of_ap);
        ESP_LOGI(NETWORK::Wlan::log_label_scan, "number of espressif devices: %3d", Wlan::statistics.num_of_espressif_devices);
        print_collected_device();
        
    }

    if (ESP_OK != retStatus)
    {
        ESP_LOGE(NETWORK::Wlan::log_label, "Scan returned /w fail status.");
    }
    return retStatus;
}

void task_wlan_manager(void *parameters){

        // some problems: https://github.com/espressif/arduino-esp32/issues/6430

        constexpr uint16_t WIFI_CONNECTION_CHECK_DELAY_SEC{5};
        constexpr uint16_t WIFI_CONNECTION_ATTEMPT_TIMEOUT_SEC{20};
        constexpr uint16_t WIFI_PAUSE_BEFORE_RECONNECT_ATTEMPT_SEC{20};
        constexpr uint16_t WIFI_NETWORK_STATISTICS_SCAN_PERIOD_SEC{60};

        ESP_LOGI(NETWORK::Wlan::log_label, "wifi iface manage start");
        wlan_interface.loadMACAddress();

        /*wlan_interface.statistics.connection_ok_sec = 0;*/
        //wlan_interface.wifi_trying_to_connect = 0;
        //wlan_interface.sta_connect();
        

        for(;;){

            switch(((Wlan::wlan_state_t)wlan_interface.get_wlan_state()))
            {

                case Wlan::wlan_state_t::waiting_init:
                {
                    wlan_interface.wifi_trying_to_connect = 0;

                    if(ESP_OK == wlan_interface.init()){
                        
                        ESP_LOGI(NETWORK::Wlan::log_label, "WLAN interface init succesfull, config preloaded");
                        wlan_interface.set_wlan_state(Wlan::wlan_state_t::inactive);
                    }

                    break;
                }

                case Wlan::wlan_state_t::inactive:
                {
                    /* if user config is required, start the AP here. */
                    /* @TODO: */

                    if(Nvs_Manager::config_in_use != user_cfg_basic.rdy_for_update){
                        wlan_interface.start_ap();
                        user_cfg_http_server = start_webserver();
                    }
                    else
                    {
                        wlan_interface.wifi_trying_to_connect = 0;
                        wlan_interface.sta_connect();
                    }
                   
                    /* if STA connection is required, start sta connection process */

                    
                    /* When iface is started -> transition to wlan_state_t::sta_wait_for_connection;*/

                    break;
                }

                case Wlan::wlan_state_t::sta_wait_for_connection:
                {
                    if(wlan_interface.wifi_trying_to_connect <= WIFI_CONNECTION_ATTEMPT_TIMEOUT_SEC)
                    {
                        wlan_interface.wifi_trying_to_connect += WIFI_CONNECTION_CHECK_DELAY_SEC;
                    }
                    else
                    {
                        wlan_interface.statistics.connection_ok_sec = 0;
                        wlan_interface.wifi_trying_to_connect = 0;
                        /* TODO: crude delay, can think of something smarter */
                    
                        ESP_LOGW(NETWORK::Wlan::log_label, "was unable to connect for: %d sec, will retry in: %d secs", WIFI_CONNECTION_ATTEMPT_TIMEOUT_SEC, WIFI_PAUSE_BEFORE_RECONNECT_ATTEMPT_SEC);
                        vTaskDelay((1000) / portTICK_PERIOD_MS);
                        wlan_interface.disconnect_power_off();
                        wlan_interface.statistics.num_of_reconnect++;
                        vTaskDelay((WIFI_PAUSE_BEFORE_RECONNECT_ATTEMPT_SEC * 1000) / portTICK_PERIOD_MS);
                    }
                    
                    break;
                }

                case Wlan::wlan_state_t::sta_wait_for_ip:
                {
                    /* Waiting for IP, nothing to do */
                    break;
                }

                case Wlan::wlan_state_t::sta_connection_ok:
                case Wlan::wlan_state_t::sta_connection_stable:
                {
                    wlan_interface.statistics.connection_ok_sec += WIFI_CONNECTION_CHECK_DELAY_SEC;
                    wlan_interface.wifi_trying_to_connect = 0;
                    break;
                }

                case Wlan::wlan_state_t::ap_active:
                {
                    /* TODO: */
                    break;
                }

                case Wlan::wlan_state_t::power_down:
                {
                    /* this is triggered from main thread, to power off wifi */
                    wlan_interface.disconnect_power_off();
                    break;
                }

                default:
                {
                    /* something's not right, I can feel it. */
                    break;
                }
            }

#ifdef NETWORK_SCAN_EN
            /* if enabled, run promiscous network scan to see how crowded is the space */
            if(wlan_interface.scan_period_counter < (WIFI_NETWORK_STATISTICS_SCAN_PERIOD_SEC/WIFI_CONNECTION_CHECK_DELAY_SEC))
            {
                wlan_interface.scan_period_counter++;
            }
            else
            {
                switch(((Wlan::wlan_state_t)wlan_interface.get_wlan_state()))
                {
                    case Wlan::wlan_state_t::inactive:
                    case Wlan::wlan_state_t::sta_wait_for_ip:
                    case Wlan::wlan_state_t::sta_connection_ok:
                    case Wlan::wlan_state_t::sta_connection_stable:
                    /*case Wlan::wlan_state_t::ap_active:*/
                    {
                        if(ESP_OK == wlan_interface.fastScan()){
                            wlan_interface.scan_period_counter = 0;
                            /*wlan_interface.disconnect_power_off();*/
                        }

                        break;
                    }

                    default:
                    {
                        break;
                    }
                }
            }
#endif /* NETWORK_SCAN_EN */

            /*xTaskResumeAll();*/
            Wlan::wifi_ext_task_inhibit_flag = false;
            vTaskDelay((WIFI_CONNECTION_CHECK_DELAY_SEC * 1000) / portTICK_PERIOD_MS);
            Wlan::wifi_ext_task_inhibit_flag = true;
            // /*wifiIF.disconnect_power_off();
            // vTaskDelay(20000 / portTICK_PERIOD_MS);*/
            // wifiIF.begin();
        }  
}

} /* NETWORK */


