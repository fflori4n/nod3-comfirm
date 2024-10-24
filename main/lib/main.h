#define pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "esp_event.h"

/*#include "esp_adc_cal.h"*/
#include "driver/gpio.h"
#include "driver/adc.h"

#include <driver/i2c.h>
#include <cmath>

#include "ntp_time/ntp_time.cpp"
#include "network_manager/network_manager.cpp"
/*#include "cron_scheduler/cron_scheduler.cpp"*/

#include "bmx280/bmx280.c"
#include "analogue_in/analogue_in.cpp"

class Main final{
public:
    static esp_err_t init(void);
    static void loop(void);
};

class MCUInfo{

    static constexpr double currentConsumptionAwake{0.0};
    static constexpr double currentConsumptionSleep{0.0};
    static constexpr char* log_label{"MCUINF"};

    public:

    esp_reset_reason_t lastResetCause;
    esp_sleep_source_t lastWakeUpCause;

    void printLog();

};

void MCUInfo::printLog(){

    ESP_LOGI(log_label, "MCU INFO: ");
}
/*

    uint8_t wakeUpCause;
  uint8_t resetCause;
  uint8_t softResetReason;

  uint32_t upTime;
  uint32_t nextScheduledWakeUp;
  uint32_t unixSendNextReportAt;

  uint32_t unixTimeLastWakeUp;
  uint32_t unixTimeLastSleepStarted;

  uint32_t upTimeStatistics_inDeepSleep;
  uint32_t upTimeStatistics_inAwake;

    public:
    mcuStateInfoESP32.wakeUpCause = esp_sleep_get_wakeup_cause();
    mcuStateInfoESP32.resetCause  = esp_reset_reason();
}*/

class bmx280_sensor{

    private:

        bmx280_t* bmx280_sens_pointer;
        bmx280_config_t bmx_sensor_cfg{BMX280_DEFAULT_CONFIG};

    public:

        static esp_err_t calculateAbsoluteHumidity(float& humidity, float& temperature, float& pressure, float& absolute_humidity, float& saturation_humidity){

            /* TODO: this is probably horible performance...*/
            float sat_vapor_pressure = std::pow(10.0, ((7.5 * temperature) / (237.3 + temperature)));
            ESP_LOGI("bmx280", "sat vapor pressure = %.2f",sat_vapor_pressure);

            return ESP_OK;
        }

        time_t last_reading{0};
        float prev_temperature{0xFFFF};
        float prev_pressure{0xFFFF};
        float prev_humidity{0xFFFF};

        float absolute_humidity;
        float saturation_humidity;

        float temperature{0xFFFF};
        float pressure{0xFFFF};
        float humidity{0xFFFF};

        float filtered_temperature{0xFFFF};
        float filtered_pressure{0xFFFF};
        float filtered_humidity{0xFFFF};

    esp_err_t begin(i2c_port_t i2c_port){

        esp_err_t res = ESP_OK;

        bmx280_sens_pointer = bmx280_create(i2c_port);

        if(bmx280_sens_pointer){
            
            res |= bmx280_init(bmx280_sens_pointer);
            res |= bmx280_configure(bmx280_sens_pointer, &bmx_sensor_cfg);
        }

        return res;
    }

    esp_err_t read(void){

        esp_err_t res = bmx280_setMode(bmx280_sens_pointer, BMX280_MODE_FORCE);

        uint16_t i =0;
        for(; (true == bmx280_isSampling(bmx280_sens_pointer)) && (i < 10000); i++){
           vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        if(i < 10000){
            res = bmx280_readoutFloat(bmx280_sens_pointer, &temperature, &pressure, &humidity);
        }
        else{
            res = ESP_FAIL;
        }

        time_t rtc_now = Ntp_time::get_esp_rtc_time();

        
        double dHumi = ((humidity - prev_humidity) * 60) / (rtc_now - last_reading);
        double dTemp = ((temperature - prev_temperature) * 60) / (rtc_now - last_reading);

        ESP_LOGI("bmx280", "temp = %.2f, dtemp = %.2f [degC/min], hum = %.2f, dhum: %.2f [%%/min], pres = %.2f", temperature, dTemp, humidity, dHumi, pressure);

        /*res = calculateAbsoluteHumidity(humidity, temperature, float& pressure, float& absolute_humidity, float& saturation_humidity);*/
        if(ESP_OK == res){
            last_reading = rtc_now;
            prev_temperature = temperature;
            prev_pressure = pressure;
            prev_humidity = humidity;
        }
        return res;
    }
        
};