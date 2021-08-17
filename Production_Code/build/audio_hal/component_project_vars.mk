# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += /home/aditya/esp/esp-adf/components/audio_hal/include /home/aditya/esp/esp-adf/components/audio_hal/driver/include /home/aditya/esp/esp-adf/components/audio_hal/driver/es8388 /home/aditya/esp/esp-adf/components/audio_hal/driver/es8374 /home/aditya/esp/esp-adf/components/audio_hal/driver/es8311 /home/aditya/esp/esp-adf/components/audio_hal/driver/es7243 /home/aditya/esp/esp-adf/components/audio_hal/driver/zl38063 /home/aditya/esp/esp-adf/components/audio_hal/driver/zl38063/api_lib /home/aditya/esp/esp-adf/components/audio_hal/driver/zl38063/example_apps /home/aditya/esp/esp-adf/components/audio_hal/driver/zl38063/firmware /home/aditya/esp/esp-adf/components/audio_hal/driver/tas5805m /home/aditya/esp/esp-adf/components/audio_hal/driver/es7148 /home/aditya/esp/esp-adf/components/audio_hal/driver/es7210
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/audio_hal -laudio_hal -L/home/aditya/esp/esp-adf/components/audio_hal/driver/zl38063/firmware -lfirmware
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += audio_hal
COMPONENT_LDFRAGMENTS += 
component-audio_hal-build: 
