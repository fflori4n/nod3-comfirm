set(SOURCES )

idf_component_register(SRCS "nvs_manager.cpp"
                        INCLUDE_DIRS .
                        REQUIRES lwip SPI_FLASH "esp_https_server")