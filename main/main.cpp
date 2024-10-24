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

/* Config for I2C bus */
constexpr i2c_config_t i2c_cfg{
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .sda_pullup_en = false,
        .scl_pullup_en = false,

        .master = {
            .clk_speed = 100000
        }
};

bmx280_sensor ambient_bme280;
ADC_input current_transformer_pin(ADC_UNIT_1,ADC_CHANNEL_4);

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

    constexpr uint32_t sntp_sync_seconds{30*60*1000};
    ntpTime.sntp_init("pool.ntp.org", 10000, sntp_sync_seconds);


    if(ESP_OK != i2c_param_config(I2C_NUM_0, &i2c_cfg)){
        ESP_LOGE("I2C_INIT", "failed to set I2C config");
    }
    if(ESP_OK != i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0)){
        ESP_LOGE("I2C_INIT", "driver install failed");
    }

    ambient_bme280.begin(I2C_NUM_0);
    // bmx280 = bmx280_create(I2C_NUM_0);

    // if (!bmx280) { 
    //     ESP_LOGE("test", "Could not create bmx280 driver.");
    //     return ESP_OK;
    // }

    // ESP_ERROR_CHECK(bmx280_init(bmx280));
    // bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    // ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));

    current_transformer_pin.configure(ADC_WIDTH_BIT_12, 1u);

    return (esp_err_t)ESP_OK;

}

void Main::loop(void){


    //printf("Main task\n");
    //ntpTime.print();
    Ntp_time::esp_uptime = (esp_timer_get_time()/1000000);
    

    //ESP_LOGI("TIME", "TIME SINCE STARTUP: %ld",ntpTime.espTimerUptime);

    /*time_t rtcTime = ntpTime.getCurrentRTCTime(false);
    ntpTime.print_time(rtcTime);*/

    // Ntp_time::print_current_time();

    /*Cron_Scheduler job_scheduler*/;

    // time_t now = Ntp_time::get_esp_rtc_time();

    // timed_job_t test_job(
    //     job_status_t::job_unprocessed,
    //     200,
    //     now + 10);

    // cron_job_t test_cron_job(
    //     job_status_t::job_unprocessed,
    //     200,
    //     "hello world");


    // job_scheduler.is_ready_to_execute(test_job, now);
    // job_scheduler.is_ready_to_execute(test_cron_job,now);

    /*if(job_scheduler.is_ready_to_execute(test_job)){
        ESP_LOGI("SCHEDULER", "RETURNED TRUE");
    }*/

    //  ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
    //     do {
    //         vTaskDelay(pdMS_TO_TICKS(1));
    //     } while(bmx280_isSampling(bmx280));

    //     float temp = 0, pres = 0, hum = 0;
    //     ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pres, &hum));

    //     ESP_LOGI("test", "Read Values: temp = %.2f, pres = %.2f, hum = %.2f", temp, pres, hum);
    
    
    ESP_LOGI("ADC0_READ", "%ld", current_transformer_pin.readAC_RMS(1000));
    /*current_transformer_pin.readAC_RMS_dma(1000);*/
    // adc1_config_width(ADC_WIDTH_BIT_12);
    // adc1_config_channel_atten((adc1_channel_t)ADC_CHANNEL_1, ADC_ATTEN_DB_11);

    // adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    // #define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
    // esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_13, DEFAULT_VREF, adc_chars);
    // print_char_val_type(val_type);

    // //Continuously sample ADC1
    // uint32_t adc_reading = 0;
    //     //Multisampling
    //     for (int i = 0; i < 10; i++) {
    //         adc_reading += adc1_get_raw((adc1_channel_t)ADC_CHANNEL_1);
    //     }
    //     adc_reading /= 10;
    // ESP_LOGI("ADC_READ", "%ld", adc_reading);

    // ambient_bme280.read();
    vTaskDelay(2000/ portTICK_PERIOD_MS);
    fflush(stdout);
    return;
}


