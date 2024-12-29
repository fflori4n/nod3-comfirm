/* #include "esp_sleep.h" */

#include <cstdint>
#include <esp_err.h>
#include <esp_sleep.h>

RTC_FAST_ATTR uint64_t nv_mcu_sleep_sec;
RTC_FAST_ATTR uint64_t nv_mcu_awake_sec;
RTC_FAST_ATTR time_t nv_last_deep_sleep_entered_exited_at_unix;

class Sleep_manager{

    private:

        uint64_t scheduled_time_until_wake_ms;
    public:
    

    private:

    // esp_err_t enable_hardware_power_save(){

    //     rtc_gpio_isolate(GPIO_NUM_12);
    //     return ESP_OK;
    // };

    public:

    Sleep_manager(){};

    // TODO: esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL)

    esp_err_t schedule_rtc_wakeup(uint64_t time_until_wake_ms){
        scheduled_time_until_wake_ms = time_until_wake_ms;
        return esp_sleep_enable_timer_wakeup(time_until_wake_ms * 1000);
    }

   inline esp_err_t execute_wake_stub(){

        time_t mcu_time_now_unix = 0;
        esp_err_t time_status = Ntp_time::get_esp_rtc_time(mcu_time_now_unix);

        if((ESP_ERR_INVALID_STATE != time_status) && (ESP_FAIL != time_status)){

            if(0 != nv_last_deep_sleep_entered_exited_at_unix){
                ESP_LOGW("NIGHTMAN", "woke up after: %lld seconds of deep sleep, setting current time as wake time", (mcu_time_now_unix - nv_last_deep_sleep_entered_exited_at_unix));
            
                nv_mcu_sleep_sec += (mcu_time_now_unix - nv_last_deep_sleep_entered_exited_at_unix);
                nv_last_deep_sleep_entered_exited_at_unix = mcu_time_now_unix;
            }
            else{

                ESP_LOGW("NIGHTMAN", "woke up after: [unknown] seconds of deep sleep, setting current time as wake time");
                nv_last_deep_sleep_entered_exited_at_unix = mcu_time_now_unix;
            }
            
        }
        else{
            ESP_LOGW("NIGHTMAN", "woke up after [unknown] seconds of deep sleep");
            nv_last_deep_sleep_entered_exited_at_unix = 0;
        }

        
        return ESP_OK;
    }

    /* NOTE: https://stackoverflow.com/questions/76823215/deep-sleep-with-ext0-or-ext1-on-esp32-c3-mini-1 */
    /* a very clear answer from hcheung related to ESP C3 gpio wake sources, EXT0 and EXT1 are not available on C3 only S3.*/
    // esp_err_t enable_gpio_wakeup(void){

    //     gpio_wakeup_enable();
    //     esp_sleep_enable_gpio_wakeup();

    //     return ESP_OK;
    // }

    void enter_deep_sleep(void) { 

        /* TODO: Disconnect wifi*/
        wlan_interface.disconnect_power_off();
        
        /* TODO: check wakeup sources are enabled correctly*/
        /* TODO: check that sleep is scheduled for longer time than minimum */
        time_t mcu_time_now_unix = 0;
        esp_err_t time_status = Ntp_time::get_esp_rtc_time(mcu_time_now_unix);

        if ((ESP_ERR_INVALID_STATE != time_status) && (ESP_FAIL != time_status))
        {
            

            if(0 != nv_last_deep_sleep_entered_exited_at_unix){
                ESP_LOGW("NIGHTMAN", "MCU was awake for: %lld seconds, now setting unix deep sleep started at to current time.", (mcu_time_now_unix - nv_last_deep_sleep_entered_exited_at_unix));
                nv_mcu_awake_sec += (mcu_time_now_unix - nv_last_deep_sleep_entered_exited_at_unix);
                nv_last_deep_sleep_entered_exited_at_unix = mcu_time_now_unix;
            }
            else{
                ESP_LOGW("NIGHTMAN", "MCU was awake for: [unknown] seconds, now setting unix deep sleep started at to current time.");
                nv_last_deep_sleep_entered_exited_at_unix = mcu_time_now_unix;
            }
            
        }
        else
        {
            ESP_LOGW("NIGHTMAN", "could't get valid time before deep sleep start!");
            nv_last_deep_sleep_entered_exited_at_unix = 0;
        }

        ESP_LOGW("NIGHTMAN", "**** **** **** **** **** **** **** ****");
        ESP_LOGW("NIGHTMAN", "ESP IS READY TO ENTER DEEP SLEEP WITH THE FOLLOWING CONFIG:");
        if(scheduled_time_until_wake_ms != 0){
            ESP_LOGW("NIGHTMAN", "TIMER WAKE: [ enabled ] : timer is preloded to trigger after: %lldms", scheduled_time_until_wake_ms);
        }
        else{
             ESP_LOGW("NIGHTMAN", "TIMER WAKE: [ disabled ]");
        }
        ESP_LOGW("NIGHTMAN", "**** **** **** **** **** **** **** ****");
        ESP_LOGW("NIGHTMAN", "ESP IS ENTERING DEEP SLEEP NOW");
        ESP_LOGW("NIGHTMAN", "**** **** **** **** **** **** **** ****");
        ESP_LOGW("NIGHTMAN", "Device will reconnect on next wake...");

        return esp_deep_sleep_start(); 
    }

};