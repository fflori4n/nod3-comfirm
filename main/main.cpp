/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "lib/main.h"

MCUInfo mcuInfo;
Ntp_time ntpTime;

extern "C" void app_main(void)
{
    ESP_LOGI("MAIN","APPLICATION CODE STARTED.");
   /* ESP_ERROR_CHECK(esp_netif_init());*/
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /* init NVS flash - used by wi-fi bluetooth or BLE */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    Main::init();

    for(;;){
        Main::loop();
    } 
}

esp_err_t Main::init(void){

    printf("DBG_UART OK");
    printf("INIT");

    mcuInfo.lastResetCause = esp_reset_reason();
    mcuInfo.lastWakeUpCause = esp_sleep_get_wakeup_cause();
    mcuInfo.printLog();

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return (esp_err_t)ESP_OK;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    xTaskCreatePinnedToCore(
    NETWORK::task_manageWlanConnection,
    "manage_network",
    5000,
    NULL,
    1,
    NULL,
    0
  );

    return (esp_err_t)ESP_OK;

}

void Main::loop(void){


    //printf("Main task\n");
    //ntpTime.print();
    ntpTime.espTimerUptime = (esp_timer_get_time()/1000000);

    //ESP_LOGI("TIME", "TIME SINCE STARTUP: %ld",ntpTime.espTimerUptime);

    //time_t rtcTime = ntpTime.getCurrentRTCTime(false);
    //ntpTime.dbgLogLocalTimeT(rtcTime);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    fflush(stdout);
    return;
}
