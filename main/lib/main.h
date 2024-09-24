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

#include "ntp_time/ntp_time.cpp"
#include "network_manager/network_manager.cpp"
#include "cron_scheduler/cron_scheduler.cpp"

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