set(SOURCES )

idf_component_register(SRCS "ntp_time.cpp"
                        INCLUDE_DIRS .
                        REQUIRES lwip )