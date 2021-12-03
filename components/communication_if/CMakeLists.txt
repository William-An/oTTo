idf_component_register(SRCS "esp_now_wireless.cpp" "communication_if.cpp" "uart_wired.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES spi_flash nvs_flash)
