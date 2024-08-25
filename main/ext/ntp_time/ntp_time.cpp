#include "ntp_time.h"

Ntp_time::Ntp_time(void){

    time(&unixTimeNow);
    // Set timezone to China Standard Time
    setenv("TZ", "CST-8", 1);
    tzset();

    localtime_r(&unixTimeNow, &tmCurrentTime);
    strftime(strftimeBuffer, sizeof(strftimeBuffer), "%c", &tmCurrentTime);
    ESP_LOGI("YM", "The current date/time in Shanghai is: %s", strftimeBuffer);
}

void Ntp_time::print(void){

    time(&unixTimeNow);
    setenv("TZ", envTz, 1);
    tzset();
    localtime_r(&unixTimeNow, &tmCurrentTime);
    strftime(strftimeBuffer, sizeof(strftimeBuffer), "%c", &tmCurrentTime);
    ESP_LOGI("YM", "The current date/time in Shanghai is: %s", strftimeBuffer);
}

time_t Ntp_time::getCurrentRTCTime(bool timeNonValidReturn){

    time_t rtcTime = time(NULL);
    /*time(&rtcTime);*/

    ESP_LOGI("TIME","%lld",rtcTime);

    if((true == timeNonValidReturn) || (Ntp_time::unixTimeAnnoDomini < rtcTime)){
        return rtcTime;
    }
    else{
        return (time_t)0;
    }
}

void Ntp_time::dbgLogLocalTimeT(time_t& rtcTime){

    ESP_LOGI("TIME","%lld",rtcTime);

    localtime_r(&rtcTime, &tmCurrentTime);
    strftime(strftimeBuffer, sizeof(strftimeBuffer), "%c", &tmCurrentTime);
    ESP_LOGI("TIME", "RTC_TIME_IS: %s", strftimeBuffer);
}



