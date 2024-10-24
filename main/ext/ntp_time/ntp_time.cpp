#include "ntp_time.h"

esp_sntp_config_t Ntp_time::sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
time_t Ntp_time::unixTimeNow{time(nullptr)};
time_t Ntp_time::esp_uptime{0};
time_t Ntp_time::unixRTCTimeLastUpdatedAt{time(nullptr)};

Ntp_time::Ntp_time(void){

    time(&unixTimeNow);
    setenv("TZ", time_zone_tz_code, 1);
    tzset();

    Ntp_time::print_current_time();
}

esp_err_t Ntp_time::evaluate_rtc_time_validity(time_t& unix_time){

    /* NOTE: ESP32 NTP client blocks sync for 15sec after update/recieving response from server */

    if((Ntp_time::unixRTCTimeLastUpdatedAt > unix_time) || (Ntp_time::unix_time_anno_domini >= unix_time))
    {
        ESP_LOGW("TIME", "Time invalid, needs update: %lld", unix_time);
        return ESP_FAIL;
    }
    ESP_LOGI("TIME", "Unix: %lld, last updated: %lld", Ntp_time::unixTimeNow,Ntp_time::unixRTCTimeLastUpdatedAt);
    return ESP_OK;
}

esp_err_t Ntp_time::rtc_time_check(){
    time(&(Ntp_time::unixTimeNow));
    return Ntp_time::evaluate_rtc_time_validity(Ntp_time::unixTimeNow);
}

time_t Ntp_time::get_esp_rtc_time(){

    time(&(Ntp_time::unixTimeNow));
    ESP_LOGI("TIME", "Unix: %lld", Ntp_time::unixTimeNow);

    return Ntp_time::unixTimeNow;
}

time_t Ntp_time::get_valid_esp_rtc_time(){

    time(&(Ntp_time::unixTimeNow));
    if(ESP_OK == evaluate_rtc_time_validity(Ntp_time::unixTimeNow)){
        return Ntp_time::unixTimeNow;
    }

    return (time_t)0;
}

void Ntp_time::print_current_time(){

    time_t unix_time = get_esp_rtc_time();

    char strftimeBuffer[64] = {'\0'};
    struct tm tmCurrentTime;

    localtime_r(&unix_time, &tmCurrentTime);
    strftime(strftimeBuffer, sizeof(strftimeBuffer), "%c", &tmCurrentTime);
    
    if(ESP_OK == evaluate_rtc_time_validity(unix_time))
    {
        ESP_LOGI("TIME_RTC", "Uptime: %lld, Unix: %lld, Local: %s",Ntp_time::esp_uptime, unix_time, strftimeBuffer);
    }
    else{
        ESP_LOGI("TIME_RTC", "Uptime: %lld, Unix: %lld, Local: %s, [ Wr ] time is not valid!",Ntp_time::esp_uptime, unix_time, strftimeBuffer);
    }
    
    return;
}

void Ntp_time::print_time(time_t& unix_time){

    localtime_r(&unix_time, &tmCurrentTime);
    strftime(strftimeBuffer, sizeof(strftimeBuffer), "%c", &tmCurrentTime);
    ESP_LOGI("TIME_RTC", "Uptime: %lld, Unix: %lld, Local: %s",Ntp_time::esp_uptime, unix_time, strftimeBuffer);
    return;
}

void time_sync_notification_cb(struct timeval *tv)
{
    
    ESP_LOGW("TIME_SYNC_CB", "Time sync callback");
    Ntp_time::print_current_time();

    time_t new_rtc_update_time = Ntp_time::get_valid_esp_rtc_time();
    if(((time_t)0) != new_rtc_update_time){
        Ntp_time::unixRTCTimeLastUpdatedAt = new_rtc_update_time;
    }

    return;
}
/* NOTE: for some external source sync e.g. GNSS use:

    settimeofday(tv, NULL);
    ESP_LOGI(TAG, "Time is synchronized from custom code");
    sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
    */

    /* EXAMPLE @: https://github.com/espressif/esp-idf/blob/v5.3.1/examples/protocols/sntp/main/sntp_example_main.c*/
esp_err_t Ntp_time::sntp_init(const char* sntp_server_url, const uint32_t wait_sync, const uint32_t auto_sync_period = 0){
    
    esp_err_t init_result{ESP_OK};
    esp_sntp_config_t& sntp_config{Ntp_time::sntp_config};

    /* TODO: do I need multiple sntp configs? probably not, so static is ok*/
    sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG(sntp_server_url);
    sntp_config.start = false;
    sntp_config.server_from_dhcp = false;             // accept NTP offers from DHCP server, if any (need to enable *before* connecting)
    sntp_config.renew_servers_after_new_IP = false;   // let esp-netif update configured SNTP server(s) after receiving DHCP lease
    sntp_config.index_of_first_server = 1;           // updates from server num 1, leaving server 0 (from DHCP) intact
    sntp_config.ip_event_to_renew = IP_EVENT_STA_GOT_IP;
    sntp_config.sync_cb = time_sync_notification_cb; // only if we need the notification function

    init_result = esp_netif_sntp_init(&(sntp_config));

    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH); /* smooth only if max diff is less than 35mins. good for drift*/
    sntp_set_sync_interval(60*1000);

    if(ESP_OK == init_result){
        ESP_LOGI("sntp", "Starting SNTP");
        init_result = esp_netif_sntp_start();
    }
    else{
        init_result = ESP_FAIL;
    }
    
    if(ESP_OK == init_result){

        if(wait_sync > 0){
            ESP_LOGI("sntp", "Waiting for time sync...");
            init_result = ESP_FAIL;
            for(uint32_t time_sync_wait_timer = 0; time_sync_wait_timer < wait_sync; time_sync_wait_timer += 2000){
                if(ESP_ERR_TIMEOUT != esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS)){
                    init_result = ESP_OK;
                }
            }
        }
    }
   
    return init_result;
}



