#include "nvs_flash.h"
#include "nvs.h"
#include <string>

/* this flag will start user configuration after reboot, if the values are not set in NVS */
bool user_config_completed = false;

/* NOTE: idea is to put all the stuff into struct, convert it to uin8_t* cstring, write it to flash, than cast it back to struct when read */
/* to avoid having keys and individual read operations when I need all the data - or at least group them by priority, if not all
important data is read, the communication will not work, so might as well reboot and retry*/
/* High prio config is: (no point running the device if these are not correct)
(byte flag to show if data is outdated and user config should be executed again)
SSID
PASS
SENSOR_NAME
WEBSOCKET_URL 
*/
/* NOTE: just ignore the security issues related to storing this in unencripted flash... http is used to POST this data so it's unsecure anyways */
namespace Nvs_Manager{

    enum {
    config_failed_to_load   = 0u,
    config_in_use           = 1u,
    config_to_be_rewritten  = 2u
    };

    typedef struct user_cfg_critical_t{

        uint8_t rdy_for_update;
        char ssid[40];
        char passwd[40];
        char device_name[40];   /* TODO: consider using the MAC addr or something to get a unique ID, and give friendly name on server side? */
        char websoc_endpoint_url[40];

    }user_cfg_critical_t;

    typedef struct reboot_statistics_t{

        uint32_t numof_usb_reset; /* probably due to new program flashing, as I'm not using USB to reset the board */
        uint32_t numof_panic_reset;
        uint32_t numof_brown_out_reset;
        uint32_t numof_wake_since_reset;

        time_t total_active_time_sec;
        time_t total_sleep_time_sec;

        time_t active_time_since_reset_sec;
        time_t sleep_time_since_reset_sec;

        time_t record_up_time;

        uint64_t reboot_id; /* just a number that will change if sensor reboots (anything other than deep sleep)*/

    }reboot_statistics_t;

    constexpr static char* log_label_nvs{"\x1b[34mNVS"};

    nvs_handle_t nvs_handle;

    esp_err_t get_nvs(std::string key, void* destination, size_t bytes){

        esp_err_t res = ESP_OK;/* nvs_open("storage", NVS_READWRITE, &nvs_handle);*/

        if(ESP_OK == res)
        {
            res = nvs_get_blob(nvs_handle, key.c_str(), (void*)(destination), &bytes);
            if(ESP_OK == res)
            {
                //nvs_close(nvs_handle);
            }
            else
            {
                ESP_LOGE(Nvs_Manager::log_label_nvs, "failed to read blob: %s (0x%x)", esp_err_to_name(res), res);
            }
        }
        else
        {
            ESP_LOGE(Nvs_Manager::log_label_nvs, "failed to open nvs page: %s (0x%x)", esp_err_to_name(res), res);
        }

        return (ESP_OK == res) ? (ESP_OK) : (ESP_FAIL);
    }

    esp_err_t set_nvs(std::string key, void* source, size_t bytes){

        esp_err_t res = ESP_OK;/*nvs_open("storage", NVS_READWRITE, &nvs_handle);*/

        if(ESP_OK == res)
        {
            res = nvs_set_blob(nvs_handle, key.c_str(), (source), bytes);
            if(ESP_OK == res){

                res = nvs_commit(nvs_handle);
                //nvs_close(nvs_handle);
                if(ESP_OK != res)
                {
                    ESP_LOGE(Nvs_Manager::log_label_nvs, "failed to commit entry: %s (0x%x)", esp_err_to_name(res), res);
                }
            }
            else
            {
                ESP_LOGE(Nvs_Manager::log_label_nvs, "failed to set blob: %s (0x%x)", esp_err_to_name(res), res);
            }
        }
        else
        {
            ESP_LOGE(Nvs_Manager::log_label_nvs, "failed to open nvs page: %s (0x%x)", esp_err_to_name(res), res);
        }

        return (ESP_OK == res) ? (ESP_OK) : (ESP_FAIL);
    }
}

Nvs_Manager::user_cfg_critical_t user_cfg_basic = {
    .rdy_for_update = Nvs_Manager::config_in_use,
    .ssid = "hello",
    .passwd = "world",
    .device_name = "test_sensor",
    .websoc_endpoint_url = "hello2"};

Nvs_Manager::reboot_statistics_t reboot_statistics = {
    .numof_usb_reset = 0,
    .numof_panic_reset = 0,
    .numof_brown_out_reset = 0,
    .numof_wake_since_reset = 0,

    .total_active_time_sec = 0,
    .total_sleep_time_sec = 0,

    .active_time_since_reset_sec = 0,
    .sleep_time_since_reset_sec = 0,

    .record_up_time = 0,
    .reboot_id = 0
    };

std::string nvs_wlan_ssid = "";


esp_err_t load_user_config(void){

    esp_err_t nvs_read_res = ESP_OK;

    /*ESP_LOGI(Nvs_Manager::log_label_nvs,"checking user cfg in nvs");*/

    if(ESP_OK == Nvs_Manager::get_nvs("ucfg_critical", (void*)(&user_cfg_basic), sizeof(Nvs_Manager::user_cfg_critical_t)))
    {
        /*ESP_LOGI(Nvs_Manager::log_label_nvs,"staus: %d, SSID read as %s, passwd: %s, sensor: %s, url: %s", 
        user_cfg_basic.rdy_for_update,
        user_cfg_basic.ssid, 
        user_cfg_basic.passwd,
        user_cfg_basic.device_name,
        user_cfg_basic.websoc_endpoint_url);*/
    }
    else{
        ESP_LOGE(Nvs_Manager::log_label_nvs,"no critical config found!");
        user_cfg_basic = {0};

        nvs_read_res = ESP_FAIL;
    }

    return nvs_read_res;
}

esp_err_t save_user_config(void){

    esp_err_t nvs_operation_res = Nvs_Manager::set_nvs("ucfg_critical", (void*)(&user_cfg_basic), sizeof(Nvs_Manager::user_cfg_critical_t));
    if(ESP_OK == nvs_operation_res)
    {
        ESP_LOGI(Nvs_Manager::log_label_nvs,"userconfig saved as: status: %d, SSID:%s, passwd: %s, sensor: %s, url: %s", 
        user_cfg_basic.rdy_for_update,
        user_cfg_basic.ssid, 
        user_cfg_basic.passwd,
        user_cfg_basic.device_name,
        user_cfg_basic.websoc_endpoint_url);
    }
    else
    {
        ESP_LOGI(Nvs_Manager::log_label_nvs,"failed to set user cfg");
    }

    return nvs_operation_res;
}

