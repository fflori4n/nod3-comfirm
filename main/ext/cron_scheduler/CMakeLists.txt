set(SOURCES )

idf_component_register(SRCS "cron_scheduler.cpp"
                        INCLUDE_DIRS .
                        REQUIRES lwip )