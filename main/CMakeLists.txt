idf_component_register(SRCS "main2.c"
                    INCLUDE_DIRS "."
                    REQUIRES
                        nvs_flash
                        esp_wifi
                        esp_netif
                        esp_event
                        ota_http
                        esp_timer
                        esp_driver_gptimer
                        esp_driver_i2c
                        esp_driver_mcpwm
                        esp_driver_uart
                        esp_driver_gpio
                        modbus_params
                        servo_motor)
