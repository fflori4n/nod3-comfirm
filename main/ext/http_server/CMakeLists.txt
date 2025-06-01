set(SOURCES )

idf_component_register(SRCS "http_server.cpp"
                        INCLUDE_DIRS .
                        REQUIRES lwip SPI_FLASH "esp_https_server")