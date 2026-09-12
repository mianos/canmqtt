set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

idf_component_register(SRCS "main.cpp" "CanBus.cpp" "FrameTable.cpp" "CanWebServer.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES
                         nvsstoragemanager
                         wifimanager
                         settingsbase
                         mqttwrapper
                         webserver
                         jsonwrapper
                         espressif__mqtt
                         esp_driver_twai
                         esp_netif
                         esp_event
                         nvs_flash
                         app_update
                         esp_partition
                         esp_timer)
