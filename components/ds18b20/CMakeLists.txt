idf_component_register(SRCS "ds18b20.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver
                    REQUIRES esp_timer
                    )

target_include_directories(${COMPONENT_LIB} PRIVATE ${COMPONENT_PATH}/path/to/driver)
