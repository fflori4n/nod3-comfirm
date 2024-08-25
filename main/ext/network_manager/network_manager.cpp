#include "network_manager.h"

namespace NETWORK
{

Wlan::Wlan(){
    ESP_LOGI(logLabel,"WLAN constructor called.");
};


/* from: https://www.youtube.com/watch?v=qdbbZYdT9Dg*/
esp_err_t Wlan::loadMACAddress(void)
{
    esp_err_t retStatus = ESP_FAIL;

    if('\0' == macAddressCStr[0]){
        uint8_t mac_byte_buffer[6]{};
    
        retStatus = esp_efuse_mac_get_default(mac_byte_buffer);
        if (ESP_OK == retStatus)
        {
        snprintf(macAddressCStr, sizeof(macAddressCStr), 
        "%02x%02x%02x%02x%02x%02x",
            mac_byte_buffer[0],
            mac_byte_buffer[1],
            mac_byte_buffer[2],
            mac_byte_buffer[3],
            mac_byte_buffer[4],
            mac_byte_buffer[5]
          );
        }

        ESP_LOGI(logLabel,"MAC address loaded: %s",macAddressCStr);
    }
    else{
        ESP_LOGI(logLabel,"mac already loaded: %s",macAddressCStr);
    }

    return retStatus;
}

void task_manageWlanConnection(void *parameters){

        Wlan wifiIF;
        wifiIF.loadMACAddress();

        for(;;){
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            printf("MAC: %s\n", wifiIF.getMACAddressCStr());
        }  
}





} /* NETWORK */