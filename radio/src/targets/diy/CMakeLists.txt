option(SHUTDOWN_CONFIRMATION "Shutdown confirmation" OFF)
option(LCD_DUAL_BUFFER "Dual LCD Buffer" OFF)
option(PXX1 "PXX1 protocol support" ON)
option(PXX2 "PXX2 protocol support" OFF)
option(INTERNAL_MODULE_PPM "Support for PPM internal module" OFF)

add_definitions(-DEEPROM_VARIANT=0)
set(HAPTIC YES)
set(LUA_EXPORT lua_export_x9d)
set(GUI_DIR 128x64)
set(NAVIGATION_TYPE x9d)
set(BITMAPS_TARGET taranis_bitmaps)
set(FONTS_TARGET taranis_fonts)
set(LCD_DRIVER lcd_driver_spi.cpp)
set(GVAR_SCREEN model_gvars.cpp)
option(INTERNAL_MODULE_PXX1 "Support for PXX1 internal module" ON)
option(INTERNAL_MODULE_PXX2 "Support for PXX2 internal module" OFF)
set(FLAVOUR x9d+)
set(CPU_TYPE STM32F2)
set(CPU_TYPE_FULL STM32F205xE)  # for size report
set(LINKER_SCRIPT targets/taranis/stm32f2_flash.ld)
set(STATUS_LEDS YES)
set(AUX_SERIAL_DRIVER ../common/arm/stm32/aux_serial_driver.cpp)
set(INTERNAL_MODULE_SERIAL YES)

if(INTERNAL_MODULE_SERIAL)
  add_definitions(-DINTERNAL_MODULE_SERIAL)
  if(PXX_FREQUENCY STREQUAL HIGH)
    add_definitions(-DPXX_FREQUENCY_HIGH)
  endif()
  set(TARGET_SRC
    ${TARGET_SRC}
    ../common/arm/stm32/intmodule_serial_driver.cpp
    )
else()
  set(TARGET_SRC
    ${TARGET_SRC}
    intmodule_pulses_driver.cpp
    )
endif()

if(ROTARY_ENCODER)
  set(TARGET_SRC
    ${TARGET_SRC}
    ../common/arm/stm32/rotary_encoder_driver.cpp
    )
endif()

if(BLUETOOTH)
  add_definitions(-DBLUETOOTH)
  set(TARGET_SRC
    ${TARGET_SRC}
    ../common/arm/stm32/bluetooth_driver.cpp
    )
  set(SRC
    ${SRC}
    bluetooth.cpp
    )
endif()

set(HSE_VALUE 12000000)
set(SDCARD NO)
set(EEPROM EEPROM_RLC)
set(TARGET_DIR diy)
set(PPM_LIMITS_SYMETRICAL YES)

add_definitions(-DPCBHEL2020)
add_definitions(-DAUDIO -DVOICE -DRTCLOCK)

set(SRC
  ${SRC}
  io/frsky_firmware_update.cpp
  io/multi_firmware_update.cpp
  )

set(GUI_SRC
  ${GUI_SRC}
  model_input_edit.cpp
  model_mix_edit.cpp
  model_display.cpp
  radio_diagkeys.cpp
  radio_diaganas.cpp
  view_channels.cpp
  view_telemetry.cpp
  view_about.cpp
  bmp.cpp
  )

if(STATUS_LEDS)
  set(LED_DRIVER led_driver.cpp)
endif()

set(SRC
  ${SRC}
  sbus.cpp
  )

set(TARGET_SRC
  ${TARGET_SRC}
  ${LED_DRIVER}
  backlight_driver.cpp
  extmodule_driver.cpp
  trainer_driver.cpp
  ../common/arm/stm32/timers_driver.cpp
  ../common/arm/stm32/audio_dac_driver.cpp
  ../common/arm/stm32/adc_driver.cpp
  ../common/arm/stm32/heartbeat_driver.cpp
  )

if(PCB STREQUAL XLITE OR PCB STREQUAL XLITES)
  set(TARGET_SRC
    ${TARGET_SRC}
    ../common/arm/stm32/sticks_pwm_driver.cpp
    )
endif()

set(FIRMWARE_TARGET_SRC
  ${FIRMWARE_TARGET_SRC}
  ${LCD_DRIVER}
  board.cpp
  i2c_driver.cpp
  ../common/arm/stm32/pwr_driver.cpp
  ../common/arm/loadboot.cpp
  )

if(LCD_DUAL_BUFFER)
  add_definitions(-DLCD_DUAL_BUFFER)
endif()

if(INTERNAL_MODULE_PXX1)
  add_definitions(-DHARDWARE_INTERNAL_MODULE)
  add_definitions(-DINTERNAL_MODULE_PXX1)
endif()

if(INTERNAL_MODULE_PXX2)
  set(PXX2 ON)
  add_definitions(-DHARDWARE_INTERNAL_MODULE)
  add_definitions(-DINTERNAL_MODULE_PXX2)
endif()

if(INTERNAL_MODULE_MULTI)
  add_definitions(-DHARDWARE_INTERNAL_MODULE)
  add_definitions(-DINTERNAL_MODULE_MULTI)
  set(TARGET_SRC
    ${TARGET_SRC}
    ../common/arm/stm32/intmodule_serial_driver.cpp
    )
endif()

if(INTERNAL_MODULE_PPM)
  add_definitions(-DHARDWARE_INTERNAL_MODULE)
  add_definitions(-DINTERNAL_MODULE_PPM)
endif()

set(RADIO_DEPENDENCIES ${RADIO_DEPENDENCIES} ${BITMAPS_TARGET})
