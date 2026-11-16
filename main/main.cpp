#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "lib/main.h"
#include <esp_wifi.h>

Ntp_time ntpTime;
Sleep_manager night_man;
Homeassistant_websocket ha_websoc;

esp_err_t i2c_num0_init_result = ESP_FAIL;
i2c_port_t i2c_driver_I2C0 = I2C_NUM_MAX; /* SET ID of active I2C interface, it will be I2C_NUM_MAX if port is not initialized or not usable for some reason */

bmx280_sensor bmx280;
/* Log boot measurements to timedMeasurement */
Timestamped_measurement_storage measurement_storage;

static adc_oneshot_unit_handle_t adc2_handle;

typedef struct mcu_long_term_stat_t{

        uint32_t num_of_program_flash = 0;
        uint32_t num_of_esp_swreset = 0;
        uint32_t num_of_esp_sleep_wake = 0;

        uint64_t total_recorded_uptime_sec = 0;
        uint64_t total_recorded_sleep_sec = 0;

    }mcu_long_term_stat_t;

    mcu_long_term_stat_t mcu_long_term_stats = {0};

extern "C" void app_main(void)
{
    /*ESP_LOGI("MAIN","APPLICATION CODE STARTED.");*/
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* first check flash, if it does not initialize all kind of functions will brake */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    /* If board was reset via button, wait for a bit. (useful aginst getting stuck in panic loop) */
    /* wait for debugger UART to connect, only for dev. */
    if(ESP_RST_POWERON ==  esp_reset_reason()){
        vTaskDelay(3000/ portTICK_PERIOD_MS); 
    }

    ESP_LOGI(main_tag,"INIT STARTED");
    Main::init();

    ESP_LOGI(main_tag,"INIT COMPLETED");
    for(;;){
        Main::loop();
    } 
}

void at_shutdown(){

        
        ESP_LOGW("USR_CFG", "Shutdown requested, running poweroff checlist");
        ESP_LOGW("USR_CFG", "**** **** **** **** **** **** **** ****");

        mcu_long_term_stats.total_recorded_uptime_sec += esp_timer_get_time() / 1000000;
        Nvs_Manager::set_nvs("mcu_long_stat", &mcu_long_term_stats, sizeof(mcu_long_term_stat_t));

        ESP_LOGW("USR_CFG", "**** **** **** **** **** **** **** ****");
        ESP_LOGW("USR_CFG", "ESP RDY FOR POWEROFF");
        ESP_LOGW("USR_CFG", "**** **** **** **** **** **** **** ****");
        ESP_LOGW("USR_CFG", "Device will reconnect on next wake...");
}

esp_err_t Main::init(void){

    /* Wait for UART to connect - otherwise welcome text is cut off */
    vTaskDelay(1000/ portTICK_PERIOD_MS);

    esp_err_t wake_tm_sts = night_man.execute_wake_stub();
    auto [mcu_time_unix, time_reliability, mcu_time_tm] = ntpTime.evaluate_mcu_rtc();

    (void)SYS_UTILS::log_welcome_message();
    (void)SYS_UTILS::log_mcu_info_message();
    (void)SYS_UTILS::log_wake_time(wake_tm_sts, (mcu_time_unix - nv_last_deep_sleep_entered_exited_at_unix));

    esp_err_t nvs_open_res = nvs_open("storage", NVS_READWRITE, &(Nvs_Manager::nvs_handle));
    esp_err_t ucfg_critical_presence = load_user_config();
    ESP_LOGW(sys_up_tag, "NVS:\tOPEN_RW: %s", ((ESP_OK == nvs_open_res) ? GREEN_OK : "ER"));
    ESP_LOGW(sys_up_tag, "\tucfg_critical read: %s", (ESP_OK == ucfg_critical_presence) ? "FOUND" : "NO CONFIG FOUND");
    user_cfg_basic.debug_print();

    esp_err_t mcu_reboot_statistics_presence = Nvs_Manager::get_nvs("reboot_stats", (void*)(&reboot_statistics), sizeof(Nvs_Manager::reboot_statistics_t));
    (void)SYS_UTILS::update_reset_statistics(reboot_statistics);
    Nvs_Manager::set_nvs("reboot_stats", &reboot_statistics, sizeof(Nvs_Manager::reboot_statistics_t));

    //esp_err_t mcu_long_stat_presence = Nvs_Manager::get_nvs("mcu_long_stat", (void*)(&mcu_long_term_stats), sizeof(mcu_long_term_stat_t));
    ESP_LOGW(sys_up_tag, "\treboot_stats read: %s", ((ESP_OK == mcu_reboot_statistics_presence) ? "FOUND" : "NOT FOUND"));
    ESP_LOGW(sys_up_tag, "\t>USB RST %ld", reboot_statistics.numof_usb_reset);
    ESP_LOGW(sys_up_tag, "\t>BROWN OUT RST: %ld", reboot_statistics.numof_brown_out_reset);
    ESP_LOGW(sys_up_tag, "\t>PANIC RST: %ld", reboot_statistics.numof_panic_reset);
    ESP_LOGW(sys_up_tag, "\t>NUMOF WAKE: %ld",reboot_statistics.numof_wake_since_reset);

    ESP_LOGW(sys_up_tag, "\t>UPTIME SINCE RST: %0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d (%0.2f percent in sleep)",SEC_TO_TIMESTAMP_PRINT(reboot_statistics.active_time_since_reset_sec + reboot_statistics.sleep_time_since_reset_sec),((100 * ((float)(reboot_statistics.sleep_time_since_reset_sec)/(reboot_statistics.active_time_since_reset_sec + reboot_statistics.sleep_time_since_reset_sec)))));
    ESP_LOGW(sys_up_tag, "\t>UPTIME TOTAL: %0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d (%0.2f percent in sleep)",SEC_TO_TIMESTAMP_PRINT((reboot_statistics.total_active_time_sec + reboot_statistics.total_sleep_time_sec)), (100 * ((float)(reboot_statistics.total_sleep_time_sec)/(reboot_statistics.total_active_time_sec + reboot_statistics.total_sleep_time_sec))));
    ESP_LOGW(sys_up_tag, "\t>RECORD UP TIME: %0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d",SEC_TO_TIMESTAMP_PRINT(reboot_statistics.record_up_time));
    ESP_LOGW(sys_up_tag, "\t>REBOOT ID: %lld",reboot_statistics.reboot_id);

    
  
    /* shutdown handler will run before every SWRESET*/
    esp_register_shutdown_handler(at_shutdown);
    
    /* evaluate BOOT button, if it is pressed, clear 'ucfg_critical' reset and enter http configuration mode */
    if (0u == gpio_get_level(GPIO_NUM_9))   /* TODO: debounce*/
    {
        user_cfg_basic.rdy_for_update = Nvs_Manager::config_to_be_rewritten;
        save_user_config();
        ESP_LOGW(usr_cfg_tag, "**** **** **** **** **** **** **** ****");
        ESP_LOGW(usr_cfg_tag, "GPIO_NUM_9 LONG PRESS Detected - 'ucfg_critical' to be reconfigured");
        ESP_LOGW(usr_cfg_tag, "New config can be set using HTTP, after reboot.");
        ESP_LOGW(usr_cfg_tag, "**** **** **** **** **** **** **** ****");
        esp_restart();
    }
    else
    {
        ESP_LOGI(usr_cfg_tag, "HINT: Use button (GPIO_NUM_9) after reboot to clear non-volatile config.");
    }

    #define ALL_GPIO_MASK ( \
        (1ULL << GPIO_NUM_0)  | (1ULL << GPIO_NUM_1)  | (1ULL << GPIO_NUM_2)  | \
        (1ULL << GPIO_NUM_3)  | (1ULL << GPIO_NUM_4)  | (1ULL << GPIO_NUM_5)  | \
        (1ULL << GPIO_NUM_6)  | (1ULL << GPIO_NUM_8)  | (1ULL << GPIO_NUM_9)  | \
        (1ULL << GPIO_NUM_10))

    gpio_config_t io_conf = {
        .pin_bit_mask = ALL_GPIO_MASK,
        .mode =         GPIO_MODE_INPUT,
        .pull_up_en =   GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type =    GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);

    /* set GPIO config and powerkey */
    io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_7),
        .mode =         GPIO_MODE_OUTPUT,
        .pull_up_en =   GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type =    GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);
    

    /* Init Network manager task */
    xTaskCreatePinnedToCore( NETWORK::task_wlan_manager, "manage_network", 5000, (void*)1, 7, NULL, 0);

    /* stop the program right here if waiting for user config, the actions are executed on network thread, and ESP will be reset on new config recieved */
    if(Nvs_Manager::config_in_use != user_cfg_basic.rdy_for_update){
        ESP_LOGW("USER_CFG", "Waiting for user config");
        for(;(Nvs_Manager::config_in_use != user_cfg_basic.rdy_for_update);){
            /* TODO: add some timeout or something */
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            ESP_LOGW("USER_CFG", ".");
        }
    }


    if((0 != gpio_get_level((gpio_num_t)GPIO_NUM_8)) || (0 != gpio_get_level((gpio_num_t)GPIO_NUM_9)))
    {
        ESP_LOGE("PWR_KEY", "Board sensor power seems to be active based on feedback pins 8, 9");
    }
    gpio_set_level(GPIO_NUM_7, 1);
    if((1 != gpio_get_level((gpio_num_t)GPIO_NUM_8)) || (1 != gpio_get_level((gpio_num_t)GPIO_NUM_9)))
    {
        ESP_LOGE("PWR_KEY", "Board sensor power didn't turn on based on feedback pins 8, 9");
    }


    /* Init ADC continous RTOS task */
    xTaskCreatePinnedToCore(task_adc_continous_measurement, "adc continous", 8000, (void*)1, 2, NULL, 0);

    auto init_i2c = [&](gpio_num_t sda, gpio_num_t scl, bool pullup_en) -> esp_err_t{

        esp_err_t res = ESP_OK;
        i2c_driver_I2C0 = I2C_NUM_MAX;
        /* Config for I2C bus */
        
        /* first check if SDA and SCL have pullup */

        /* configure SDA and SCL as high Z, to check if pullups are present */
        gpio_config_t sda_scl_floating_in_conf = {
            .pin_bit_mask = (1ULL << GPIO_NUM_8 | 1ULL << GPIO_NUM_9),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&sda_scl_floating_in_conf);

        uint8_t sda_gpio_level = gpio_get_level((gpio_num_t)GPIO_NUM_8);
        uint8_t scl_gpio_level = gpio_get_level((gpio_num_t)GPIO_NUM_9);
        ESP_LOGI(i2c_tag, "Checking GPIO pullups: SDA: %s, SCL: %s", ((1u == sda_gpio_level) ? GREEN_OK : RED_ER), ((1u == scl_gpio_level) ? GREEN_OK : RED_ER));

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

        ESP_LOGI(i2c_tag, "Initializing I2C_NUM_%d module, on pins: SDA: GPIO_NUM_%d, SCL: GPIO_NUM_%d", I2C_NUM_0, GPIO_NUM_8, GPIO_NUM_9);

        if((0u != sda_gpio_level) && (0u != scl_gpio_level))
        {

            res = i2c_param_config(I2C_NUM_0, &i2c_cfg);
            if(ESP_OK == res)
            {
                res = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
                if(ESP_OK == res)
                {
                    i2c_driver_I2C0 = I2C_NUM_0;
                }
                else
                {
                    ESP_LOGE("I2C_INIT", "Driver install failed: %s", esp_err_to_name(res));
                }
            }
            else
            {
                ESP_LOGE("I2C_INIT", "Failed to set I2C config: %s", esp_err_to_name(res));
            }
        }
        else
        {
            res = ESP_FAIL;
        }
                    
        return res;
    };
    /* initializing I2C port */

    i2c_num0_init_result = init_i2c(GPIO_NUM_8, GPIO_NUM_9, false);

    vTaskDelay(5000/portTICK_PERIOD_MS);
    constexpr uint32_t sntp_sync_seconds{30*60*1000};
    ntpTime.sntp_init("pool.ntp.org", 10000, sntp_sync_seconds);
    

    if(ESP_OK == i2c_num0_init_result)
    {
        bmx280.begin(i2c_driver_I2C0, 0x77, "BMX0", 19.0); //0x76
    }

    // if(ESP_OK == i2c_num0_init_result)
    // {
    //     tail_bmx280.begin(i2c_driver_I2C0, 0x77, "BMX0", 19.0);
    //     onboard_bmx280.begin(i2c_driver_I2C0, 0x76, "BMX1", 19.0);
    //     hall_angle_sensor.begin(i2c_driver_I2C0, 0x36);
    //     hall_angle_sensor.init();
        
    //     ccs811_sensor.begin(i2c_driver_I2C0, 0x5A);
    //     /*ccs811_sensor.init();*/
    //     ccs811_sensor.read_measurement(0, 0);

    //     gpio_reset_pin(GPIO_NUM_7);
    //     gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
    //     gpio_set_level(GPIO_NUM_7, true);

    //     gpio_config_t io_conf = {
    //         .pin_bit_mask = (1ULL << GPIO_NUM_4),
    //         .mode = GPIO_MODE_INPUT,
    //         .pull_up_en = GPIO_PULLUP_DISABLE,
    //         .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //         .intr_type = GPIO_INTR_DISABLE
    //     };
    //     gpio_config(&io_conf);

    //     #define WINDSENS_EN 1        
    //     #ifdef WINDSENS_EN
    //     wind_hall.configure_as_frequency_input();
    //     rain_hall.configure_as_frequency_input();
    //     #endif
    //     hall_angle_sensor.init();
    // }

    measurement_storage.insert_measurement(Timestamped_measurement(Measurement_type::MCU_SOL, (uint32_t)Ntp_time::get_sol()));
    // measurement_storage.insert_measurement(Timestamped_measurement(Measurement_type::MCU_TEMP, mcu_info.chip_internal_temp_degc));


    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_2,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc2_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle,ADC_CHANNEL_0,&config));

    mcu_sys_info_t sys_info;
    SYS_UTILS::get_mcu_sys_info(&sys_info);
    ESP_LOGW("SYS_INFO", "MAC: %s", sys_info.mac_addr);
    ESP_LOGW("SYS_INFO", "CORE_TEMP: %0.2f", sys_info.core_temperature);
    ESP_LOGW("SYS_INFO", "HEAP: %ld of %ld", sys_info.heap_watermark, sys_info.heap_internal);
    ESP_LOGW("SYS_INFO", "SOL: %ld", sys_info.device_sol );
    ESP_LOGW("SYS_INFO", "AWAKE_TIME: %0.2f", sys_info.hours_since_rst * 3600);
    ESP_LOGW("SYS_INFO", "POWERON/WAKE TOKENS: %s : %ld", sys_info.poweron_token, sys_info.wake_token);


    return (esp_err_t)ESP_OK;
}

time_t last_report_send_period = 0;
bool rdy_to_report = false;

void Main::loop(void){

    auto [mcu_time_unix, time_reliability, now_tm] = Ntp_time::evaluate_mcu_rtc(true);
    // time_t now_sys_uptime_secs = (esp_timer_get_time()/1000);

    mcu_sys_info_t sys_info;
    SYS_UTILS::get_mcu_sys_info(&sys_info);
    ESP_LOGW("SYS_INFO", "MAC: %s", sys_info.mac_addr);
    ESP_LOGW("SYS_INFO", "CORE_TEMP: %0.2f", sys_info.core_temperature);
    ESP_LOGW("SYS_INFO", "HEAP: %ld of %ld", sys_info.heap_watermark, sys_info.heap_internal);
    ESP_LOGW("SYS_INFO", "SOL: %ld", sys_info.device_sol );
    ESP_LOGW("SYS_INFO", "AWAKE_TIME: %0.2f", sys_info.hours_since_rst * 3600);
    ESP_LOGW("SYS_INFO", "POWERON/WAKE TOKENS: %s : %ld", sys_info.poweron_token, sys_info.wake_token);

    //measurement_storage.print_measurements();

    //mcu_info.update_mcu_telemetry();
    //measurement_storage.insert_measurement(Timestamped_measurement(Measurement_type::MCU_TEMP, mcu_info.chip_internal_temp_degc));
    //measurement_storage.mark_measurements_to_be_sent(0, 10, 10);
    //measurement_storage.get_servicedata_from_marked_measurements();

    /*mcu_info.set_bat_voltage(continous_adc_manager.read_dc_voltage_on_channel(GPIO_NUM_1));*/
    float bat_voltage = continous_adc_manager.read_dc_voltage_on_channel(GPIO_NUM_0);
    ESP_LOGI(COLOR_PINK"BAT:"COLOR_WHITE, "DC voltage: %0.2f", bat_voltage);
    float voltage_in = continous_adc_manager.read_dc_voltage_on_channel(GPIO_NUM_1);
    ESP_LOGI(COLOR_PINK"VIN:"COLOR_WHITE, "DC voltage: %0.2f", voltage_in);
    float analog_temp = continous_adc_manager.read_dc_voltage_on_channel(GPIO_NUM_5);
    ESP_LOGI(COLOR_PINK"TEMP:"COLOR_WHITE, "DC voltage: %0.2f", analog_temp);
    
    int raw_reading = 0;
    esp_err_t err = adc_oneshot_read(
        adc2_handle,
        ADC_CHANNEL_0,
        &raw_reading
    );
    ESP_LOGI(COLOR_PINK"TEMP:"COLOR_WHITE, "Temp on NTC: %0.2f", raw_reading);

    bmx280.read();
    vTaskDelay(1000/portTICK_PERIOD_MS);

    night_man.update_awake_time();
    night_man.schedule_rtc_wakeup(10 * 1000);
    //night_man.enter_deep_sleep();

    return;
 /*  if(ESP_OK == i2c_num0_init_result)
    {
        
        onboard_bmx280.read();
        
    
        #ifdef WINDSENS_EN
        hall_angle_sensor.read_wind_angle();
        //ESP_LOGI("rpmHall"," high/low [%lld:%lld] %0.2f, lowhigh ratio: %0.2f", raw_pulse_high_us/1000, raw_pulse_low_us/1000, 1000000.0/((raw_pulse_high_us) + (raw_pulse_low_us)), (1.0 * raw_pulse_low_us)/(1.0 * raw_pulse_high_us));
        wind_hall.dbgPrint();
        rain_hall.dbgPrint();
        #endif
        ccs811_sensor.read_measurement(onboard_bmx280.temperature, onboard_bmx280.humidity);
    }*/

    /*int32_t raw_scale = hive_scale.read_raw();*/
    #ifdef SCALE_EN
    DS18B20::read_all_sensors();
    ESP_LOGI(DS18B20::log_label_ds18b20, "pcb temp:%0.2f°C Scale temp:%0.2f°C", DS18B20::get_last_measurement(ds18temp_addr_pcb), DS18B20::get_last_measurement(ds18temp_addr_scale));
    #endif

    #ifdef SCALE_EN
    ESP_LOGE("scale", "processed value: %0.2f", hive_scale.read(30));
    #endif

    

    time_t last_full_minute_unix = (mcu_time_unix - (time_t)now_tm.tm_sec);
    rdy_to_report |= ((ESP_OK == Homeassistant_websocket::sensor_report_period_rdy(last_full_minute_unix, last_report_send_period, 60)) ? true : false);



    TaskHandle_t myTaskHandle = xTaskGetCurrentTaskHandle();
    ESP_LOGE("STACK SIZE", "stack %ld", (uint32_t)uxTaskGetStackHighWaterMark(myTaskHandle));



    if ((Ntp_time::rtc_time_sts_t::Not_synced_usable == time_reliability) || (Ntp_time::rtc_time_sts_t::OK == time_reliability))
    {
        Ntp_time::get_sol();
        /* if time is not ok, postpone send until there is valid time to use */
        // if (true == rdy_to_report)
        // {
        //     esp_err_t mcu_info_report_rdy = Homeassistant_websocket::sensor_report_period_rdy(mcu_time_unix, mcu_info.last_report_unix, MCUInfo::report_cycle_time_sec, false);
        //     esp_err_t network_report_rdy = Homeassistant_websocket::sensor_report_period_rdy(mcu_time_unix, wlan_interface.last_report_unix, NETWORK::Wlan::report_cycle_time_sec, false);
        //     esp_err_t bmx_report_rdy = Homeassistant_websocket::sensor_report_period_rdy(mcu_time_unix, tail_bmx280.last_report_unix, bmx280_sensor::report_cycle_time_sec, false);
        //     esp_err_t ac_input_report_rdy = Homeassistant_websocket::sensor_report_period_rdy(mcu_time_unix, continous_adc_manager.last_report_unix, ADC_continous::report_cycle_time_sec, false);

        //     if ((ESP_OK == mcu_info_report_rdy) || (ESP_OK == network_report_rdy) || (ESP_OK == bmx_report_rdy) || (ESP_OK == ac_input_report_rdy))
        //     {

        //         esp_err_t connect_res = ha_websoc.connectAndAuthSocket(5, 1500);

        //         if (ESP_ERR_WIFI_BASE == connect_res)
        //         {
        //             ESP_LOGE("WEBSOC", "No network, unable to connect.");
        //         }
        //         else
        //         {
        //             esp_err_t report_sent = ESP_FAIL;

        //             if (ESP_OK == mcu_info_report_rdy)
        //             {
        //               mcu_info.get_service_data_report(mqtt_service_data_buffer, 2048);
        //                 ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
        //                 snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
        //                 report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);
                        
        //                 if(ESP_OK == report_sent){
        //                     mcu_info.last_report_unix = mcu_time_unix;
        //                     mcu_info.last_report_sys_uptime = esp_timer_get_time();
        //                 }
        //             }

        //             if (ESP_OK == network_report_rdy)
        //             {
        //                 wlan_interface.get_service_data(mqtt_service_data_buffer, 2048);
        //                 ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
        //                 snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
        //                 report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

        //                 if(ESP_OK == report_sent){
        //                     wlan_interface.last_report_unix = mcu_time_unix;
        //                 }
        //             }

        //             if (/*ESP_OK == bmx_report_rdy*/ true)
        //             {
        //                 tail_bmx280.get_service_data_report(mqtt_service_data_buffer, 2048, "bmx0");
        //                 ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
        //                 snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
        //                 report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

        //                 if(ESP_OK == report_sent)
        //                 {
        //                     tail_bmx280.last_report_unix = mcu_time_unix;
        //                 }

        //                 onboard_bmx280.get_service_data_report(mqtt_service_data_buffer, 2048, "bmx1");
        //                 ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
        //                 snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
        //                 report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

        //                 if(ESP_OK == report_sent)
        //                 {
        //                     onboard_bmx280.last_report_unix = mcu_time_unix;
        //                 }
        //             }

        //             if (ESP_OK == ac_input_report_rdy)
        //             {
        //                 // /*continous_adc_manager.get_service_data_ac_input(mqtt_service_data_buffer, 2048, 3);*/
        //                 continous_adc_manager.get_service_data_report(mqtt_service_data_buffer, 2048, 3);
        //                 ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
        //                 snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
        //                 report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

        //                 // /*continous_adc_manager.get_service_data_ac_input(mqtt_service_data_buffer, 2048, 4);*/
        //                 //continous_adc_manager.get_service_data_report(mqtt_service_data_buffer, 2048, 4);
        //                 //ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
        //                 //snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
        //                 //report_sent = ha_websoc.send_text(msg, "\"success\":true", nullptr, 2000);

        //                 if(ESP_OK == report_sent){
        //                     continous_adc_manager.last_report_unix = mcu_time_unix;
        //                 }
        //             }

        //             ccs811_sensor.get_service_data_report(mqtt_service_data_buffer, 2048);
        //             ESP_LOGD("service_data", "%s", mqtt_service_data_buffer);
        //             snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
        //             report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

        //             hall_angle_sensor.get_service_data_report(mqtt_service_data_buffer, 2048, 60000000.0 / (1.0 + (float)(wind_hall.pulse_low_us + wind_hall.pulse_high_us)), wind_hall.edge_count);
        //             ESP_LOGD("service_data", "%s", mqtt_service_data_buffer);
        //             snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
        //             report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

        //             hive_scale.get_service_data_report(mqtt_service_data_buffer, 2048);
        //             ESP_LOGD("service_data", "%s", mqtt_service_data_buffer);
        //             snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
        //             report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

        //             // sensor_measurement_report.evaluate_send_time(mcu_time_unix);
                    
        //             // mcu_info.add_to_report(sensor_measurement_report);
        //             // tail_bmx280.add_to_report(sensor_measurement_report, 0u);
        //             // onboard_bmx280.add_to_report(sensor_measurement_report, 1u);
        //             // hall_angle_sensor.add_to_report(sensor_measurement_report, 10);
        //             // wlan_interface.add_to_report(sensor_measurement_report);
        //             // hive_scale.add_to_report(sensor_measurement_report);
        //             // DS18B20::add_to_report(sensor_measurement_report);
        //             // mcu_info.add_to_report(sensor_measurement_report);
        //             /*sensor_measurement_report.sort_keyval_array_by_group();*/
                    

                    

        //             // DS18B20::get_service_data_report(mqtt_service_data_buffer, 2048);
        //             // ESP_LOGD("service_data", "%s", mqtt_service_data_buffer);
        //             // snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
                    
        //             sensor_measurement_report.get_websoc_text(msg, 2048, ha_websoc.ha_websoc_header_template);
        //             report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

        //             wind_hall.pulse_low_us = 0;
        //             wind_hall.pulse_high_us = 0;
        //             wind_hall.edge_count = 0;

        //             ESP_LOGI("hello", "awake vs sleep seconds: %lld : %lld", nv_mcu_awake_sec, nv_mcu_sleep_sec);
        //         }

        //         ha_websoc.disconnect();
        //         rdy_to_report = false;

        //         wlan_interface.fastScan();
        //         vTaskDelay(200/portTICK_PERIOD_MS);
        //         night_man.update_awake_time();
        //         night_man.schedule_rtc_wakeup(40 * 1000);
        //         //night_man.enter_deep_sleep();
                
                
        //     }
        // }
    }
    //@TODO: add this back in, but with new methods for generating report
    // else
    // {
    //     if (true == rdy_to_report)
    //     {
    //         /* fallback in case NTP sync is not possible TODO: add timeout to not enter this part right away, try and get NTP sync before */

    //         /* use the ESP internal uptime counter to send a status/ alive massage at least every 5 minutes - fallback operation */
    //         esp_err_t mcu_info_report_rdy = Homeassistant_websocket::sensor_report_period_rdy(now_sys_uptime_secs, mcu_info.last_report_sys_uptime, (60*5), false);

    //         if (ESP_OK == mcu_info_report_rdy)
    //         {

    //             esp_err_t connect_res = ha_websoc.connectAndAuthSocket(5, 1500);

    //             if (ESP_ERR_WIFI_BASE == connect_res)
    //             {
    //                 ESP_LOGE("WEBSOC", "No network, unable to connect.");
    //             }
    //             else
    //             {
    //                 esp_err_t report_sent = ESP_FAIL;

    //                 mcu_info.get_service_data_report(mqtt_service_data_buffer, 2048);
    //                 ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);
    //                 snprintf(msg, 2048, ha_websoc.ha_websoc_header_template, mqtt_service_data_buffer);
    //                 report_sent = ha_websoc.send_text(msg, "\"success\":true", "\"success\":false", 2000);

    //                 if (ESP_OK == report_sent)
    //                 {
    //                     mcu_info.last_report_unix = mcu_time_unix;
    //                     mcu_info.last_report_sys_uptime = now_sys_uptime_secs;
    //                 }
    //             }
    //             ha_websoc.disconnect();
    //             rdy_to_report = false;
    //         }
    //     }
    // }

    /*ccs811_read_sensor(&ccs811_mox_sensor);*/

    

    night_man.update_awake_time();

    // wlan_interface.get_service_data(mqtt_service_data_buffer, 2048);
    // ESP_LOGI("service_data", "%s", mqtt_service_data_buffer);

    //esp_backtrace_print(32);
    /*log_backtrace_to_rtcmem();*/

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    fflush(stdout);

    return;
}


