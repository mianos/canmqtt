set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

idf_component_register(SRCS "main.cpp" "CanBus.cpp" "FrameTable.cpp" "CanWebServer.cpp" "SignalTable.cpp"
                            "Actions.cpp" "ModbusBus.cpp"
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
                         esp_driver_gpio
                         esp_driver_uart
                         espressif__esp-modbus
                         spiffs
                         esp_netif
                         esp_event
                         nvs_flash
                         app_update
                         esp_partition
                         esp_timer)

# The decode table is embedded so a freshly flashed board decodes immediately;
# signalstore::ensureDefault() copies it onto SPIFFS when none is stored.
target_add_binary_data(${COMPONENT_TARGET} "../data/signals.json" TEXT)
