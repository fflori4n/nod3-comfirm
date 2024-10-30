idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS lib ext
                    REQUIRES freertos 
                    esp_common 
                    esp_event
                    nvs_flash
                    esp_timer
                    esp_wifi
                    esp_netif
                    lwip
                    driver)
