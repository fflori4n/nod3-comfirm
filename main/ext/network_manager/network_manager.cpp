#include "network_manager.h"
#include "network_credentials.h"

class ADC_continous;    /* forward declared because need to pause ADC when wifi is sta_begin ing*/
/* static member init */
NETWORK::Wlan wlan_interface;

namespace NETWORK
{
    Wlan::wlan_state_t Wlan::wlan_state{Wlan::wlan_state_t::NOT_INITIALIZED};
    Wlan::wlan_manager_stats_t Wlan::statistics;
    std::array<Wlan::wlan_scan_devices_t, (Wlan::maximum_number_of_APs + Wlan::maimum_number_of_STAs) > Wlan::wlan_device_list;
    
    bool Wlan::wifi_ext_task_inhibit_flag = true;
    uint8_t Wlan::associated_ap_index = 1;

    std::array<Wlan::Wlan_access_point, NETWORK::Wlan::numof_wlan_access_point_names>  Wlan::access_point_list = get_wlan_credentials();

    wifi_init_config_t Wlan::_wifiInitCfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t Wlan::_wifiConfig{};

/* FROM: https://github.com/espressif/esp-idf/blob/master/examples/wifi/getting_started/station/main/station_example_main.c */
/* FROM: https://www.youtube.com/watch?v=3dM6LiAriEg */ 
esp_err_t Wlan::init(void){

    esp_err_t retStatus = ESP_OK;

    if( wlan_state_t::NOT_INITIALIZED == wlan_state){

        /* NOTE: Dont forget to first do: esp_netif_init() and esp_event_loop_create_default() they are moved to main::init */

        std::unique_lock<std::mutex> wlan_iface_guard(wifi_driver_mutex, std::try_to_lock);
        retStatus = (true == wlan_iface_guard.owns_lock()) ? ESP_OK : ESP_FAIL;

        if (ESP_OK == retStatus)
        {
            /* register wifi event handler functions - these will create CBs for stepping through state machine */
            /* TODO: this has a mistake, if first returns ESP_ER, second ESP_OK exec will still continue. fix at some point...*/
            retStatus = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &network_event_handler, nullptr, nullptr);
            retStatus = esp_event_handler_instance_register(IP_EVENT,   ESP_EVENT_ANY_ID, &network_event_handler, nullptr, nullptr);
        }

        if(ESP_OK == retStatus)
        {
            p_netif = esp_netif_create_default_wifi_sta();
            retStatus = (nullptr == p_netif) ? (ESP_FAIL) : (ESP_OK);
        }

        if(ESP_OK == retStatus)
        {
            retStatus = esp_wifi_init(&_wifiInitCfg);
        }
        
        if(ESP_OK == retStatus)
        {
            /* set WIFI power saving mode - more saving slower response */
            retStatus = esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
        }

        if(ESP_OK == retStatus)
        {
            const char* sta_ssid = Wlan::access_point_list[associated_ap_index].ssid;
            const char* sta_passwd = Wlan::access_point_list[associated_ap_index].passwd;

            // if(Nvs_Manager::config_in_use == user_cfg_basic.rdy_for_update)
            // {
            //     sta_ssid = user_cfg_basic.ssid;
            //     sta_passwd = user_cfg_basic.passwd;
            // }

            /* check if ssid or pass will be truncated, generate a warning, but continue */
            if(strlen(sta_ssid) >= 32u){ ESP_LOGW(log_label, "SSID too long, will be trancated to buffer size!"); }
            if(strlen(sta_passwd) >= 64u){ ESP_LOGW(log_label, "pass too long, will be trancated to buffer size!"); }

            snprintf(reinterpret_cast<char*>(_wifiConfig.sta.ssid), sizeof(_wifiConfig.sta.ssid),"%s", sta_ssid);
            snprintf(reinterpret_cast<char*>(_wifiConfig.sta.password), sizeof(_wifiConfig.sta.password),"%s", sta_passwd);
            ESP_LOGW(NETWORK::Wlan::log_label, "Reading creds from NVS: %s, STA will try to connect to %s : %s",((Nvs_Manager::config_in_use == user_cfg_basic.rdy_for_update) ? "OK" : "FAIL"), sta_ssid, sta_passwd);
        
            _wifiConfig.sta.scan_method = WIFI_FAST_SCAN;
            _wifiConfig.sta.bssid_set = false;
            _wifiConfig.sta.channel = Wlan::access_point_list[associated_ap_index].channel;
            _wifiConfig.sta.listen_interval = 20;
            _wifiConfig.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
            _wifiConfig.sta.threshold.rssi = -75;
            _wifiConfig.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
            _wifiConfig.sta.pmf_cfg.capable = true;
            _wifiConfig.sta.pmf_cfg.required = false;
            _wifiConfig.sta.failure_retry_cnt = 1;
        }

        if(ESP_OK == retStatus)
        {
            esp_wifi_set_config(WIFI_IF_STA, &_wifiConfig);
        }
    }
    return retStatus;
}

esp_err_t Wlan::start_ap(){
    
    esp_err_t res = ESP_OK;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    esp_netif_create_default_wifi_ap();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    res = esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&network_event_handler,nullptr,nullptr);

    Wlan_access_point espap_provisioning_wlan = get_esp_ap_provisioning_credentials();

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "espC3_magpie",
            .password = "123456789",
            .ssid_len = strlen("espC3_magpie"),
            .channel = 3,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = 10,
            .pmf_cfg = {
                .required = true,
            },
        },
    };

    /* but actually change the AP credentials to the ones in network_credentials.h */
    snprintf((char*)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "%s", espap_provisioning_wlan.ssid);
    snprintf((char*)wifi_config.ap.password, sizeof(wifi_config.ap.password), "%s", espap_provisioning_wlan.passwd);
    wifi_config.ap.ssid_len = strlen((char*)wifi_config.ap.ssid);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    // esp_netif_ip_info_t ip;
    // IP4_ADDR(&ip.ip, 192,168,1,1);
    // IP4_ADDR(&ip.gw, 192,168,1,1);
    // IP4_ADDR(&ip.netmask, 255,255,255,0);

    // esp_netif_dhcps_stop(ap_netif);
    // esp_netif_set_ip_info(ap_netif, &ip);
    // esp_netif_dhcps_start(ap_netif);

    if (ap_netif &&
        esp_netif_get_ip_info(ap_netif, &ip_info) == ESP_OK)
    {
        ESP_LOGI("wifi_AP","AP IP: " IPSTR, IP2STR(&ip_info.ip));
    }

    wlan_state = wlan_state_t::AP_ACTIVE_RDY;

    ESP_LOGI("wifi_AP", "wifi_init_softap finished. SSID:%s password:%s channel:%d", wifi_config.ap.ssid, wifi_config.ap.password, wifi_config.ap.channel);

    return res;
}

void Wlan::network_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    constexpr uint16_t lock_attempt_timeout_ms{100};
    constexpr uint16_t islocked_poll_ms{1};

    /* no multiple events at the same time.*/
    /* NOTE: wlan_state is protected by this different mutex, so also need this.*/

    std::unique_lock<std::mutex> wlan_event_callback_lock(wifi_driver_callback_mutex,std::defer_lock);

    for(uint16_t i = 0; !wlan_event_callback_lock.try_lock() && i < (lock_attempt_timeout_ms / islocked_poll_ms); ++i)
    {
        vTaskDelay(pdMS_TO_TICKS(islocked_poll_ms));
    }

    if (wlan_event_callback_lock.owns_lock()){

            if (IP_EVENT == event_base)
            {
                const ip_event_t event_type{static_cast<ip_event_t>(event_id)};

                switch (event_type)
                {
                case IP_EVENT_STA_GOT_IP:
                {
                    wlan_interface.set_wlan_state(wlan_state_t::STA_CONNECTION_ESTABLISHED);
                    Ntp_time::start_sntp(); /* TODO: is this really the best place to put this??*/
                    break;
                }

                case IP_EVENT_STA_LOST_IP:
                {
                    wlan_interface.set_wlan_state(wlan_state_t::WAITING_EVENT_STA_GOT_IP);
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
                    wlan_interface.set_wlan_state(wlan_state_t::WAITING_EVENT_STA_CONNECT);
                    break;
                }

                case WIFI_EVENT_STA_CONNECTED:
                {
                    wlan_interface.set_wlan_state(wlan_state_t::WAITING_EVENT_STA_GOT_IP);
                    Wlan::statistics.num_of_reconnect++;
                    break;
                }

                case WIFI_EVENT_STA_BEACON_TIMEOUT:
                case WIFI_EVENT_STA_DISCONNECTED:
                {
                    wlan_interface.set_wlan_state(wlan_state_t::STA_DISCONNECTED);
                    wlan_interface.statistics.connection_ok_sec = 0;
                    break;
                }

                case WIFI_EVENT_AP_STACONNECTED:
                {
                    wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
                    ESP_LOGI(NETWORK::Wlan::network_async_tag, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
                    break;
                }

                case WIFI_EVENT_AP_STADISCONNECTED:
                {
                    wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
                    ESP_LOGI(NETWORK::Wlan::network_async_tag, "station "MACSTR" leave, AID=%d, reason=%d", MAC2STR(event->mac), event->aid, event->reason);
                    break;
                }

                default:
                    /* do nothing */
                    break;
                }
            }

        /* Print events to dbg terminal - this will take longer but does not need lock so will not block */
        auto debug_print_event = [](esp_event_base_t event_base, int32_t event_id, void *event_data){

            if (event_base == IP_EVENT)
            {
                ESP_LOGI(NETWORK::Wlan::network_async_tag, "IP_EVENT::%s", ip_event_to_string(static_cast<ip_event_t>(event_id)));
            }
            else if (event_base == WIFI_EVENT)
            {
                // Wi-Fi event handling
                ESP_LOGI(NETWORK::Wlan::network_async_tag, "WIFI_EVENT::%s", wifi_event_to_string(static_cast<wifi_event_t>(event_id)));
                            /* on disconnect event:*/
                if(static_cast<wifi_event_t>(event_id) == WIFI_EVENT_STA_DISCONNECTED){
                    wifi_event_sta_disconnected_t* disconnect_event_data = static_cast<wifi_event_sta_disconnected_t*>(event_data);
                    ESP_LOGI(NETWORK::Wlan::network_async_tag, "EVENT_DATA::%d, %d", disconnect_event_data->rssi, disconnect_event_data->reason);
                }
            }
            else
            {
                ESP_LOGW(NETWORK::Wlan::network_async_tag, "unexpected event type: %s", event_base);
            }

        };

        debug_print_event(event_base, event_id, event_data);
    }
    else
    {
        ESP_LOGW(NETWORK::Wlan::network_async_tag, "wlan event dropped. handler busy based on mutex");
        return;
    }
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

void Wlan::calculate_nearby_dev_statistics(wlan_manager_stats_t &statistics, Wlan::Wlan_access_point &associated_ap){
    
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
    

    vTaskDelay(100 / portTICK_PERIOD_MS);       /* wait a little bit, to let other tasks pause*/

    if (wlan_state_t::NOT_INITIALIZED == wlan_state)
    {
        retStatus = this->init();
    }

    /* NOTE: W (609) wifi:sta_scan: STA is connecting, scan are not allowed!*/
    if (wlan_state_t::NOT_CONNECTED == wlan_state ||
        wlan_state_t::WAITING_EVENT_STA_CONNECT == wlan_state ||
        wlan_state_t::WAITING_EVENT_STA_GOT_IP == wlan_state)
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
        wlan_state = tWlanState::wlanState_initCompleted;*/
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        /*esp_wifi_set_mode(WIFI_MODE_STA);*/
        /*esp_wifi_start();*/
        retStatus = ESP_OK;
    }

    if (ESP_OK == retStatus)
    {
        

        constexpr const uint16_t scan_time_per_channel_ms{100}; /* 500 */
        constexpr const uint16_t scan_time_home_dwell_ms{250};  /* 250 */
        constexpr const uint16_t scan_time_extra_margin_ms{250};   /* 2000 */

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

                // for(auto& access_point : Wlan::access_point_list){

                //     if(0 == strncmp((char*)ap_record.ssid, access_point.ssid, 30)){
                //         access_point.device_mac = new_ap_device.device_mac;
                //         access_point.channel = new_ap_device.channel;
                //         access_point.last_seen = 0;

                //         ESP_LOGI("","AP is currently present: %s, mac: %llx, CH:%d", ap_record.ssid, access_point.device_mac, access_point.channel);
                //     }
                // }
                if( 0 == strncmp(new_ap_device.ssid.c_str(),(const char*)_wifiConfig.sta.ssid, 30)){

                    Wlan_access_point& access_point = NETWORK::Wlan::access_point_list[0u];
                    access_point.device_mac = new_ap_device.device_mac;
                    access_point.channel = new_ap_device.channel;
                    access_point.last_seen = 0;

                    ESP_LOGI("","AP is currently present: %s, mac: %llx, CH:%d", ap_record.ssid, access_point.device_mac, access_point.channel);
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

        ESP_LOGI(NETWORK::Wlan::network_tag, "WIFI TASK STARTED");

        /* [INIT] */
        /* [INACTIVE] */
        /* [STANDBY]-----------------------------------------------------------A-- [AP_ACTIVE]              */
        /* l--> (WIFI_EVENT_STA_START) -> [WAITING_EVENT_STA_CONNECT]                 l--> soft_reset         */
        /* l--> (WIFI_EVENT_STA_CONNECTED) -> [WAITING_EVENT_STA_GOT_IP]                                             */
        /* l--> (IP_EVENT_STA_GOT_IP) -> [connection_ok]                                                    */


        for(;;)
        {
            auto wlan_iface_status = wlan_interface.get_wlan_state();

            ESP_LOGI(NETWORK::Wlan::network_tag, "Wlan state machine %s", Wlan::wlan_state_to_cstr(wlan_iface_status));
            if(Wlan::wlan_state_t::ER_UNABLE_TO_LOCK == wlan_iface_status){
                continue;
            }

            switch(wlan_iface_status)
            {
                case Wlan::wlan_state_t::NOT_INITIALIZED:
                {
                    esp_err_t res = wlan_interface.init();
                    if(ESP_OK == res)
                    {
                        ESP_LOGI(NETWORK::Wlan::network_tag, "WLAN interface initialized.");
                    }
                    wlan_interface.set_wlan_state(Wlan::wlan_state_t::INTERFACE_DOWN);

                    break;
                }

                case Wlan::wlan_state_t::INTERFACE_DOWN:
                {
                    auto set_wlam_mode_to_nan = []() -> esp_err_t
                    {
                        std::unique_lock<std::mutex> wlan_iface_guard(Wlan::wifi_driver_mutex, std::try_to_lock);
                        esp_err_t res = (true == wlan_iface_guard.owns_lock()) ? ESP_OK : ESP_FAIL;

                        wifi_mode_t mode = WIFI_MODE_NULL;
                        esp_err_t er = esp_wifi_get_mode(&mode);

                        if((ESP_OK == er) && (mode != WIFI_MODE_NAN))
                        {
                            esp_wifi_set_mode(WIFI_MODE_NAN);
                        }

                        return res;
                    };

                    (void)set_wlam_mode_to_nan();

                    if(Nvs_Manager::config_in_use != user_cfg_basic.rdy_for_update){
                        wlan_interface.start_ap();
                        user_cfg_http_server = start_webserver();
                    }
                    else
                    {
                        /* IF wifi enabled */
                        wlan_interface.set_wlan_state(Wlan::wlan_state_t::STA_CONNECT);
                    }
                    break;
                }

                case Wlan::wlan_state_t::STA_CONNECT:
                {
                    auto request_wlan_connect = []() -> esp_err_t {
                        std::unique_lock<std::mutex> wlan_iface_guard(Wlan::wifi_driver_mutex, std::try_to_lock);
                        esp_err_t res = (true == wlan_iface_guard.owns_lock()) ? ESP_OK : ESP_FAIL;

                        wifi_mode_t mode = WIFI_MODE_NULL;
                        esp_err_t er = esp_wifi_get_mode(&mode);

                        if((ESP_OK == er) && (mode != WIFI_MODE_STA))
                        {
                            esp_wifi_set_mode(WIFI_MODE_STA);
                        }
                        esp_wifi_start();
                        esp_wifi_set_max_tx_power(8.5 * 4);
                        esp_wifi_connect();
                        
                        return res;
                    };

                    request_wlan_connect();
                    wlan_interface.set_wlan_state(Wlan::wlan_state_t::WAITING_EVENT_STA_CONNECT);
                    wlan_interface.wifi_trying_to_connect = 0;
                    break;
                }

                case Wlan::wlan_state_t::WAITING_EVENT_STA_CONNECT:
                {
                    /* NOTE: Waiting for WIFI_EVENT_STA_CONNECTED which will switch state to WAITING_EVENT_STA_GOT_IP */

                    uint32_t seconds_now = esp_timer_get_sec();
                    if(( seconds_now - Wlan::switched_state_tm) <= 20)
                    {
                        wlan_interface.wifi_trying_to_connect = (seconds_now - Wlan::switched_state_tm);
                        ESP_LOGW(NETWORK::Wlan::network_tag, "waiting for wlan connect: %lld sec", wlan_interface.wifi_trying_to_connect);
                    }
                    else
                    {
                        wlan_interface.set_wlan_state(Wlan::wlan_state_t::STA_REQUEST_DISCONNECT);
                        ESP_LOGW(NETWORK::Wlan::network_tag, "was unable to connect for: %d sec, will retry in: %d secs", WIFI_CONNECTION_ATTEMPT_TIMEOUT_SEC, WIFI_PAUSE_BEFORE_RECONNECT_ATTEMPT_SEC);
                    }

                    break;
                }

                case Wlan::wlan_state_t::WAITING_EVENT_STA_GOT_IP:
                {
                    /* NOTE: Waiting for IP_EVENT_STA_GOT_IP Event which will switch state to STA_CONNECTION_ESTABLISHED */

                    uint32_t seconds_now = esp_timer_get_sec();
                    if(( seconds_now - Wlan::switched_state_tm) >= 30)
                    {
                        ESP_LOGI(NETWORK::Wlan::network_tag, "waiting for IP address for too long, will disconnect");
                        wlan_interface.set_wlan_state(Wlan::wlan_state_t::STA_REQUEST_DISCONNECT);  /* NOTE: not really well tested this one... */
                        /* wlan_interface.set_wlan_state(Wlan::wlan_state_t::INTERFACE_DOWN);*/
                    }
                    break;
                }

                case Wlan::wlan_state_t::STA_CONNECTION_ESTABLISHED:
                {
                    wlan_interface.set_wlan_state(Wlan::wlan_state_t::STA_CONNECTED_RDY);
                    break;
                }

                case Wlan::wlan_state_t::STA_CONNECTED_RDY:
                {
                    wlan_interface.statistics.connection_ok_sec += WIFI_CONNECTION_CHECK_DELAY_SEC;
                    wlan_interface.wifi_trying_to_connect = 0;
                    break;
                }

                case Wlan::wlan_state_t::STA_DISCONNECTED:
                {
                    /* TODO: wait before reconnecting. */
                    vTaskDelay((1000) / portTICK_PERIOD_MS);
                    wlan_interface.set_wlan_state(Wlan::wlan_state_t::INTERFACE_DOWN);
                    break;
                }

                /* TODO: think about if wlan should be completely deinitialized before going to sleep? probably not, disconnect is enough */
                case Wlan::wlan_state_t::POWERING_DOWN:
                case Wlan::wlan_state_t::STA_REQUEST_DISCONNECT:
                {
                    wlan_interface.statistics.connection_ok_sec = 0;
                    wlan_interface.wifi_trying_to_connect = 0;

                    auto disconnect_stop_sta = []() -> esp_err_t {
                        std::unique_lock<std::mutex> wlan_iface_guard(Wlan::wifi_driver_mutex, std::try_to_lock);
                        esp_err_t res = (true == wlan_iface_guard.owns_lock()) ? ESP_OK : ESP_FAIL;

                        esp_wifi_disconnect();
                        esp_wifi_stop();

                        /* esp_wifi_deinit(); */
                        /* esp_wifi_clear_default_wifi_driver_and_handlers(p_netif); */
                        /* esp_netif_destroy(p_netif); */
                        
                        return res;
                    };

                    disconnect_stop_sta();

                    ESP_LOGI(NETWORK::Wlan::log_label, "stopping wlan iface");
                    wlan_interface.set_wlan_state(Wlan::wlan_state_t::INTERFACE_DOWN);
                    break;
                }

                case Wlan::wlan_state_t::AP_ACTIVE_RDY:
                {
                    /* TODO: */
                    break;
                }

                default:
                {
                    /* something's not right, I can feel it. */
                    break;
                }
            }

/*#define NETWORK_SCAN_EN*/
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
                    case Wlan::wlan_state_t::STA_CONNECTION_ESTABLISHED:
                    case Wlan::wlan_state_t::STA_CONNECTED_RDY:
                    /*case Wlan::wlan_state_t::AP_ACTIVE_RDY:*/
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


