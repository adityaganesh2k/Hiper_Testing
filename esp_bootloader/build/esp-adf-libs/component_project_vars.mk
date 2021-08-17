# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += /home/aditya/esp/esp-adf/components/esp-adf-libs/esp_audio/include /home/aditya/esp/esp-adf/components/esp-adf-libs/esp_codec/include/codec /home/aditya/esp/esp-adf/components/esp-adf-libs/esp_codec/include/processing /home/aditya/esp/esp-adf/components/esp-adf-libs/recorder_engine/include /home/aditya/esp/esp-adf/components/esp-adf-libs/esp_ssdp/include /home/aditya/esp/esp-adf/components/esp-adf-libs/esp_upnp/include /home/aditya/esp/esp-adf/components/esp-adf-libs/esp_sip/include /home/aditya/esp/esp-adf/components/esp-adf-libs/audio_misc/include
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/esp-adf-libs -lesp-adf-libs -L/home/aditya/esp/esp-adf/components/esp-adf-libs/esp_audio/lib/esp32 -L/home/aditya/esp/esp-adf/components/esp-adf-libs/esp_codec/lib/esp32 -L/home/aditya/esp/esp-adf/components/esp-adf-libs/recorder_engine/lib/esp32 -L/home/aditya/esp/esp-adf/components/esp-adf-libs/esp_ssdp/lib/esp32 -L/home/aditya/esp/esp-adf/components/esp-adf-libs/esp_upnp/lib/esp32 -lesp_processing -lesp_audio -lesp_codec -lesp_upnp -lrecorder_engine  -L/home/aditya/esp/esp-adf/components/esp-adf-libs/esp_sip/lib/esp32 -lesp_sip-v4x -L/home/aditya/esp/esp-adf/components/esp-adf-libs/esp_ssdp/lib/esp32 -lesp_ssdp-v4x
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += esp-adf-libs
COMPONENT_LDFRAGMENTS += 
component-esp-adf-libs-build: 
