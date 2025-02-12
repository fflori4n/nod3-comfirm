/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "lib/main.h"

/*MCUInfo mcuInfo;*/
Ntp_time ntpTime;
//MCUInfo mcu_info;
Sleep_manager night_man;


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
/*ADC_input current_transformer_pin(ADC_UNIT_1,ADC_CHANNEL_4);*/
/*ADC_input ldr_resistor(ADC_UNIT_1,ADC_CHANNEL_4);*/
Homeassistant_websocket ha_websoc;
char* sensor_websoc_template = "{\"id\":%%d,\"type\":\"call_service\",\"domain\":\"websoc_sensor\",\"service\":\"acct_test_sens.set_values\",\"service_data\":{%s}}";
char msg[2048];
char mqtt_service_data_buffer[2048] = {};
AC_current_measurement c0_current_sensor;

extern "C" void app_main(void)
{
    ESP_LOGI("MAIN","APPLICATION CODE STARTED.");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /* init NVS flash - used by wi-fi bluetooth or BLE */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_LOGI("MAIN","INIT STARTED");
    Main::init();
    ESP_LOGI("MAIN","INIT COMPLETED");
    for(;;){
        Main::loop();
       /* ESP_LOGI("MAIN","LOOP COMPLETED, PET RTOS WDT");*/
    } 
}

esp_err_t Main::init(void){

    vTaskDelay(2000/ portTICK_PERIOD_MS); /* wait for debugger UART to connect, only for dev. */
    ESP_LOGI("DBG_UART", "[ OK ]");
    ESP_LOGI("INIT STUB", "");

    if(ESP_RST_PANIC == esp_reset_reason()){
        while(true){
            vTaskDelay((1000.0f) / portTICK_PERIOD_MS);
        }
    }

    night_man.execute_wake_stub();
    
    

    /*if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return (esp_err_t)ESP_OK;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");*/


    xTaskCreatePinnedToCore(
    NETWORK::task_manageWlanConnection,
    "manage_network",
    5000,
    (void*)1,
    7,
    NULL,
    0
  );
    xTaskCreatePinnedToCore(
    task_adc_continous_measurement,
    "adc continous",
    8000,
    (void*)1,
    2,
    NULL,
    0
  );

    // bmx280 = bmx280_create(I2C_NUM_0);

    // if (!bmx280) { 
    //     ESP_LOGE("test", "Could not create bmx280 driver.");
    //     return ESP_OK;
    // }

    // ESP_ERROR_CHECK(bmx280_init(bmx280));
    // bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    // ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));

    /*current_transformer_pin.configure(ADC_WIDTH_BIT_12, 1u);*/
    /*ldr_resistor.configure(ADC_WIDTH_BIT_12, 1u);*/

    // mcu_info.load_session_nonchanging();
    // mcu_info.update_mcu_telemetry();
    // mcu_info.print_detailed();

    if(ESP_OK != i2c_param_config(I2C_NUM_0, &i2c_cfg)){
        ESP_LOGE("I2C_INIT", "failed to set I2C config");
    }
    if(ESP_OK != i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0)){
        ESP_LOGE("I2C_INIT", "driver install failed");
    }

    ambient_bme280.begin(I2C_NUM_0, "BMX0", 19.0);
    

    constexpr uint32_t sntp_sync_seconds{30*60*1000};
    
    ntpTime.sntp_init("pool.ntp.org", 10000, sntp_sync_seconds);

    vTaskDelay(1000/portTICK_PERIOD_MS);

    
    return (esp_err_t)ESP_OK;

}

void Main::loop(void){

    vTaskDelay(100 / portTICK_PERIOD_MS);


    /*for(int i =0; i < 200; i++){*/

        
    /*}*/

    // wlan_interface.fastScan();
    //wlan_interface.fastScan();

    
    ambient_bme280.read();
    // mcu_info.update_mcu_telemetry();

   
    

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
    
    //current_transformer_pin.readAC_RMS(c0_current_sensor, ntpTime, 1000);

    

    // c0_current_sensor.get_service_data(mqtt_service_data_buffer, 2048);
    // ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
    // snprintf(msg, 2048, sensor_websoc_template, mqtt_service_data_buffer);
    // ha_websoc.send_text(msg, "\"success\":true", nullptr, 2000);

    wlan_interface.get_service_data(mqtt_service_data_buffer, 2048);
    // ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);

    

    esp_err_t connect_res = ha_websoc.connectAndAuthSocket(5, 1500);

    if (ESP_ERR_WIFI_BASE == connect_res)
    {
        ESP_LOGE("WEBSOC", "No network, unable to connect.");
    }
    else
    {

        snprintf(msg, 2048, sensor_websoc_template, mqtt_service_data_buffer);
        ha_websoc.send_text(msg, "\"success\":true", nullptr, 2000);

         //c0_current_sensor.get_service_data(mqtt_service_data_buffer, 2048);
        continous_adc_manager.get_service_data_ac_input(mqtt_service_data_buffer, 2048, 3);
        ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
        snprintf(msg, 2048, sensor_websoc_template, mqtt_service_data_buffer);
        ha_websoc.send_text(msg, "\"success\":true", nullptr, 2000);

        ambient_bme280.get_service_data(mqtt_service_data_buffer, 2048);
        ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
        snprintf(msg, 2048, sensor_websoc_template, mqtt_service_data_buffer);
        ha_websoc.send_text(msg, "\"success\":true", nullptr, 2000);

        // ldr_resistor.get_service_data_ldr_resistor(mqtt_service_data_buffer, 2048);
        // ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
        // snprintf(msg, 2048, sensor_websoc_template, mqtt_service_data_buffer);
        // ha_websoc.send_text(msg, "\"success\":true", nullptr, 2000);

        //mcu_info.get_service_data(mqtt_service_data_buffer, 2048);
        // ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
        // snprintf(msg, 2048, sensor_websoc_template, mqtt_service_data_buffer);
        // ha_websoc.send_text(msg, "\"success\":true", nullptr, 2000);

        //ESP_LOGI("hello", "awake vs sleep seconds: %lld : %lld", nv_mcu_awake_sec, nv_mcu_sleep_sec);

        ha_websoc.disconnect();

        // night_man.schedule_rtc_wakeup(180000);
        // night_man.enter_deep_sleep();

        
    }

    
    

    // ESP_LOGE("WEBSOC", " uptime: %lld", nv_mcu_uptime_sec);
    
    
    /*ESP_LOGI("", "mutex: %d", txrx_buffer_mutex.try_lock());*/
    /*vTaskDelay(10000/ portTICK_PERIOD_MS);*/
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

    // portMUX_TYPE adc_reg_lock = portMUX_INITIALIZER_UNLOCKED;
    // #define ADC_REG_LOCK_ENTER()       portENTER_CRITICAL(&adc_reg_lock)
    // #define ADC_REG_LOCK_EXIT()        portEXIT_CRITICAL(&adc_reg_lock)

    // ADC_REG_LOCK_ENTER();
    //REG_SET_BIT(, SYSTEM_APB_SARADC_RST_V); // Setting bit 28 to 1
    // ADC_REG_LOCK_EXIT();

    // ESP_LOGE("ADC_REG","APB_SARADC_DMA_CONF_REG: %x ", GET_PERI_REG_BITS(APB_SARADC_DMA_CONF_REG, 31, 0));  /* should be */
    // ESP_LOGE("ADC_REG","SYSTEM_PERIP_RST_EN0_REG: %x ", GET_PERI_REG_BITS(SYSTEM_PERIP_RST_EN0_REG, 31, 0));

    // portDISABLE_INTERRUPTS();
    // //SET_PERI_REG_MASK(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST); 
    // SET_PERI_REG_MASK(APB_SARADC_DMA_CONF_REG, APB_SARADC_APB_ADC_RESET_FSM);
    // SET_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_APB_SARADC_RST);
    // 
    // // SET_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_APB_SARADC_CLK_EN);
    // SET_PERI_REG_MASK(APB_SARADC_CTRL2_REG, APB_SARADC_TIMER_EN);
    // portENABLE_INTERRUPTS();
   
    //SET_PERI_REG_MASK(APB_SARADC_DMA_CONF_REG, APB_SARADC_DMA_CONF_REG);
    


                 /* no good, stops the dma callbacks */
    //WRITE_PERI_REG(APB_SARADC_DMA_CONF_REG, 0x80000140);
    /*SET_PERI_REG_MASK(APB_SARADC_DMA_CONF_REG, APB_SARADC_APB_ADC_EOF_NUM);*/

    /*CLEAR_PERI_REG_MASK(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_TSENS_CLK_EN);*/
    //CLEAR_PERI_REG_MASK(APB_SARADC_APB_TSENS_CTRL_REG, APB_SARADC_TSENS_PU);

    
     


    //SET_PERI_REG_MASK(SYSTEM_APB_SARADC_RST, 0xFFFF); /* clear the dma pointer */

    //SET_PERI_REG_MASK(APB_SARADC_START, APB_SARADC_START_M);    /* start SAR ADC */
    //SET_PERI_REG_MASK(APB_SARADC_START_FORCE, APB_SARADC_START_FORCE_M);    /* force start SAR ADC */

    // WRITE_PERI_REG( APB_SARADC_DMA_CONF_REG, 
	// 	1<<APB_SARADC_APB_ADC_TRANS_S |
	// 	(sizeof(link_buf[0])/sizeof(link_buf[0][0]))<<APB_SARADC_APB_ADC_EOF_NUM_S );
    // portDISABLE_INTERRUPTS();
    // SET_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_APB_SARADC_RST);
    // CLEAR_PERI_REG_MASK(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_APB_SARADC_RST);
    // WRITE_PERI_REG( APB_SARADC_DMA_CONF_REG, 1<<APB_SARADC_APB_ADC_TRANS_S | (sizeof(2) / sizeof(adc_channel_t)) << APB_SARADC_APB_ADC_EOF_NUM_S );
    // portENABLE_INTERRUPTS();

    printFreeRTOSStats();
  
    adc_continous_pause = false;
    

    vTaskDelay((2 * 60 * 1000.0f) / portTICK_PERIOD_MS);
    /*nv_mcu_uptime_sec += 30;*/
    fflush(stdout);
    adc_continous_pause = true;
    vTaskDelay(1500 / portTICK_PERIOD_MS);

    return;
}


