/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include <inttypes.h>
#include "definitions.h"
#include "opentx_constants.h"
#include "board_common.h"
#include "hal.h"


#if defined(ROTARY_ENCODER_NAVIGATION)
// Rotary Encoder driver
void rotaryEncoderInit();
void rotaryEncoderCheck();
#endif

#define FLASHSIZE                       0x80000
#define BOOTLOADER_SIZE                 0x8000
#define FIRMWARE_ADDRESS                0x08000000

#define LUA_MEM_MAX                     (0)    // max allowed memory usage for complete Lua  (in bytes), 0 means unlimited

#if defined(STM32F4)
  #define PERI1_FREQUENCY               42000000
  #define PERI2_FREQUENCY               84000000
#else
  #define PERI1_FREQUENCY               30000000
  #define PERI2_FREQUENCY               60000000
#endif

#define TIMER_MULT_APB1                 2
#define TIMER_MULT_APB2                 2

extern uint16_t sessionTimer;

// Board driver
void boardInit();
void boardOff();

// Timers driver
void init2MhzTimer();
void init5msTimer();

// SD driver
#define BLOCK_SIZE                      512 /* Block Size in Bytes */
#define SD_IS_HC()                     (0)
#define SD_GET_SPEED()                 (0)
#define sdInit()
#define sdMount()
#define sdDone()
#define SD_CARD_PRESENT()              false

// Flash Write driver
#define FLASH_PAGESIZE 256
void unlockFlash();
void lockFlash();
void flashWrite(uint32_t * address, const uint32_t * buffer);
uint32_t isFirmwareStart(const uint8_t * buffer);
uint32_t isBootloaderStart(const uint8_t * buffer);

// Pulses driver
#define INTERNAL_MODULE_ON()            GPIO_SetBits(INTMODULE_PWR_GPIO, INTMODULE_PWR_GPIO_PIN)
#if defined(INTMODULE_USART)
  #define INTERNAL_MODULE_OFF()         intmoduleStop()
#else
  #define INTERNAL_MODULE_OFF()         GPIO_ResetBits(INTMODULE_PWR_GPIO, INTMODULE_PWR_GPIO_PIN)
#endif

#define IS_INTERNAL_MODULE_ON()         false

void intmoduleSerialStart(uint32_t baudrate, uint8_t rxEnable, uint16_t parity, uint16_t stopBits, uint16_t wordLength);
#if defined(INTERNAL_MODULE_MULTI)
void intmoduleTimerStart(uint32_t periodMs);
#endif
void intmoduleSendByte(uint8_t byte);
void intmoduleSendBuffer(const uint8_t * data, uint8_t size);
void intmoduleSendNextFrame();

// Trainer driver
#define SLAVE_MODE()                    (g_model.trainerData.mode == TRAINER_MODE_SLAVE)

// Trainer detect is a switch on the jack
#if defined(TRAINER_DETECT_GPIO)
  #define TRAINER_CONNECTED()           (GPIO_ReadInputDataBit(TRAINER_DETECT_GPIO, TRAINER_DETECT_GPIO_PIN) == Bit_RESET)
#else
  #define TRAINER_CONNECTED()   true
#endif

#if defined(TRAINER_GPIO)
  void init_trainer_ppm();
  void stop_trainer_ppm();
  void init_trainer_capture();
  void stop_trainer_capture();
#else
  #define init_trainer_ppm()
  #define stop_trainer_ppm()
  #define init_trainer_capture()
  #define stop_trainer_capture()
#endif
#if defined(TRAINER_MODULE_CPPM)
  void init_trainer_module_cppm();
  void stop_trainer_module_cppm();
#else
  #define init_trainer_module_cppm()
  #define stop_trainer_module_cppm()
#endif
#if defined(TRAINER_MODULE_SBUS)
  void init_trainer_module_sbus();
  void stop_trainer_module_sbus();
#else
  #define init_trainer_module_sbus()
  #define stop_trainer_module_sbus()
#endif

#if defined(INTMODULE_HEARTBEAT_GPIO)
void init_intmodule_heartbeat();
void stop_intmodule_heartbeat();
void check_intmodule_heartbeat();
#else
#define init_intmodule_heartbeat()
#define stop_intmodule_heartbeat()
#define check_intmodule_heartbeat()
#endif

void check_telemetry_exti();

// SBUS
int sbusGetByte(uint8_t * byte);

// Keys driver
enum EnumKeys
{
#if defined(KEYS_GPIO_REG_SHIFT)
  KEY_SHIFT,
#endif

#if defined(KEYS_GPIO_REG_MENU)
  KEY_MENU,
#endif

  KEY_EXIT,
  KEY_ENTER,

#if defined(KEYS_GPIO_REG_DOWN)
  KEY_DOWN,
  KEY_UP,
#endif

#if defined(KEYS_GPIO_REG_RIGHT)
  KEY_RIGHT,
  KEY_LEFT,
#endif

#if defined(KEYS_GPIO_REG_PAGE)
  KEY_PAGE,
#endif

#if defined(KEYS_GPIO_REG_PLUS)
  KEY_PLUS,
  KEY_MINUS,
#endif

  KEY_COUNT,
  KEY_MAX = KEY_COUNT - 1,

#if defined(ROTARY_ENCODER_NAVIGATION)
  KEY_PLUS,
  KEY_MINUS,
#endif

  TRM_BASE,
  TRM_LH_DWN = TRM_BASE,
  TRM_LH_UP,
  TRM_LV_DWN,
  TRM_LV_UP,
  TRM_RV_DWN,
  TRM_RV_UP,
  TRM_RH_DWN,
  TRM_RH_UP,
  TRM_LAST = TRM_RH_UP,

  NUM_KEYS
};

#if !defined(SIMU)
  #define KEY_UP                        KEY_MINUS
  #define KEY_DOWN                      KEY_PLUS
  #define KEY_RIGHT                     KEY_PLUS
  #define KEY_LEFT                      KEY_MINUS
#else
  #define KEY_UP                        KEY_PLUS
  #define KEY_DOWN                      KEY_MINUS
  #define KEY_RIGHT                     KEY_MINUS
  #define KEY_LEFT                      KEY_PLUS
#endif

#if defined(KEYS_GPIO_PIN_SHIFT)
#define IS_SHIFT_KEY(index)             (index == KEY_SHIFT)
#if defined(SIMU)
#define IS_SHIFT_PRESSED()              (readKeys() & (1 << KEY_SHIFT))
#else
#define IS_SHIFT_PRESSED()              (~KEYS_GPIO_REG_SHIFT & KEYS_GPIO_PIN_SHIFT)
#endif
#else
#define IS_SHIFT_KEY(index)             (false)
#define IS_SHIFT_PRESSED()              (false)
#endif

enum EnumSwitches
{
  SW_SA,
  SW_SB,
  SW_SC,
  SW_SD,
};
#define IS_3POS(x)                      ((x) != SW_SD)

enum EnumSwitchesPositions
{
  SW_SA0,
  SW_SA1,
  SW_SA2,
  SW_SB0,
  SW_SB1,
  SW_SB2,
  SW_SC0,
  SW_SC1,
  SW_SC2,
  SW_SD0,
  SW_SD1,
  SW_SD2,
  NUM_SWITCHES_POSITIONS
};

#define NUM_SWITCHES                  4
#define STORAGE_NUM_SWITCHES          NUM_SWITCHES
#define DEFAULT_SWITCH_CONFIG         (SWITCH_2POS << 6) + (SWITCH_2POS << 4) + (SWITCH_3POS << 2) + (SWITCH_3POS << 0)
#define DEFAULT_POTS_CONFIG           (POT_WITHOUT_DETENT << 4) +(POT_WITHOUT_DETENT << 2) + (POT_WITHOUT_DETENT << 0)

#define STORAGE_NUM_SWITCHES_POSITIONS  (STORAGE_NUM_SWITCHES * 3)

void keysInit();
uint32_t switchState(uint8_t index);
uint32_t readKeys();
uint32_t readTrims();
#define TRIMS_PRESSED()                 (readTrims())
#define KEYS_PRESSED()                  (readKeys())

// WDT driver
#define WDG_DURATION                      500 /*ms*/
#if !defined(WATCHDOG) || defined(SIMU)
  #define WDG_ENABLE(x)
  #define WDG_RESET()
#else
  #define WDG_ENABLE(x)                 watchdogInit(x)
  #define WDG_RESET()                   IWDG->KR = 0xAAAA
#endif
void watchdogInit(unsigned int duration);
#define WAS_RESET_BY_SOFTWARE()             (RCC->CSR & RCC_CSR_SFTRSTF)
#define WAS_RESET_BY_WATCHDOG()             (RCC->CSR & (RCC_CSR_WDGRSTF | RCC_CSR_WWDGRSTF))
#define WAS_RESET_BY_WATCHDOG_OR_SOFTWARE() (RCC->CSR & (RCC_CSR_WDGRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_SFTRSTF))

// ADC driver
enum Analogs {
  STICK1,
  STICK2,
  STICK3,
  STICK4,
  POT_FIRST,
  POT1 = POT_FIRST,
  POT2,
  POT3,
  POT_LAST = POT3,
  SLIDER1,
  SLIDER2,
  TX_VOLTAGE,
  NUM_ANALOGS
};

#define NUM_POTS                        3 // TODO X9D has only 2 pots
#define NUM_SLIDERS                     2
#define STORAGE_NUM_POTS                3
#define STORAGE_NUM_SLIDERS             2

#define NUM_XPOTS                       STORAGE_NUM_POTS
#define NUM_TRIMS                       4
#define NUM_MOUSE_ANALOGS               0
#define STORAGE_NUM_MOUSE_ANALOGS       0

#define NUM_TRIMS_KEYS                (NUM_TRIMS * 2)

#if defined(STICKS_PWM)
  #define NUM_PWMSTICKS                 4
  #define STICKS_PWM_ENABLED()          (!hardwareOptions.sticksPwmDisabled)
  void sticksPwmInit();
  void sticksPwmRead(uint16_t * values);
  extern volatile uint32_t pwm_interrupt_count; // TODO => reusable buffer (boot section)
#else
  #define STICKS_PWM_ENABLED()          false
#endif

PACK(typedef struct {
  uint8_t pcbrev:2;
  uint8_t sticksPwmDisabled:1;
  uint8_t pxx2Enabled:1;
}) HardwareOptions;

extern HardwareOptions hardwareOptions;

#if !defined(PXX2)
  #define IS_PXX2_INTERNAL_ENABLED()            (false)
  #define IS_PXX1_INTERNAL_ENABLED()            (true)
#elif !defined(PXX1) || defined(PCBXLITES) || defined(PCBX9LITE)
  #define IS_PXX2_INTERNAL_ENABLED()            (true)
  #define IS_PXX1_INTERNAL_ENABLED()            (false)
#elif defined(INTERNAL_MODULE_PXX1)
  #define IS_PXX2_INTERNAL_ENABLED()            (false)
  #define IS_PXX1_INTERNAL_ENABLED()            (true)
#else
  // TODO #define PXX2_PROBE
  // TODO #define IS_PXX2_INTERNAL_ENABLED()            (hardwareOptions.pxx2Enabled)
  #define IS_PXX2_INTERNAL_ENABLED()            (true)
  #define IS_PXX1_INTERNAL_ENABLED()            (true)
#endif

enum CalibratedAnalogs {
  CALIBRATED_STICK1,
  CALIBRATED_STICK2,
  CALIBRATED_STICK3,
  CALIBRATED_STICK4,
  CALIBRATED_POT_FIRST,
  CALIBRATED_POT_LAST = CALIBRATED_POT_FIRST + NUM_POTS - 1,
  CALIBRATED_SLIDER_FIRST,
  CALIBRATED_SLIDER_LAST = CALIBRATED_SLIDER_FIRST + NUM_SLIDERS - 1,
  NUM_CALIBRATED_ANALOGS
};

#define IS_POT(x)                       ((x)>=POT_FIRST && (x)<=POT_LAST)
#define IS_SLIDER(x)                    ((x)>POT_LAST && (x)<TX_VOLTAGE)

extern uint16_t adcValues[NUM_ANALOGS];

// Battery driver
// 2 x Li-Ion
#define BATTERY_WARN                    66 // 6.6V
#define BATTERY_MIN                     67 // 6.7V
#define BATTERY_MAX                     83 // 8.3V

#define BATT_SCALE                      117

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif

// Power driver
#define SOFT_PWR_CTRL
void pwrInit();
uint32_t pwrCheck();
void pwrOn();
void pwrOff();
bool pwrPressed();
#if defined(PWR_BUTTON_PRESS)
#define STARTUP_ANIMATION
uint32_t pwrPressedDuration();
#endif
void pwrResetHandler();

#if defined(SIMU)
  #define UNEXPECTED_SHUTDOWN()         false
#else
  #define UNEXPECTED_SHUTDOWN()         (WAS_RESET_BY_WATCHDOG() || g_eeGeneral.unexpectedShutdown)
#endif

// Backlight driver
void backlightInit();
void backlightDisable();
#define BACKLIGHT_DISABLE()             backlightDisable()
uint8_t isBacklightEnabled();
void backlightEnable(uint8_t level = 0);
#define BACKLIGHT_ENABLE()              backlightEnable(g_eeGeneral.backlightBright)

#if !defined(SIMU)
  void usbJoystickUpdate();
#endif
#define USB_NAME                        "Zombie Graupner"
#define USB_MANUFACTURER                'G', 'r', 'a', 'u', 'p', 'n', 'e', 'r'  /* 8 bytes */
#define USB_PRODUCT                     'Z', 'o', 'm', 'b', 'i', 'e', ' ', ' '  /* 8 Bytes */

#if defined(__cplusplus) && !defined(SIMU)
}
#endif

// I2C driver: EEPROM + Audio Volume
#define EEPROM_SIZE                   (8*32768)

void i2cInit();
void eepromReadBlock(uint8_t * buffer, size_t address, size_t size);
void eepromStartWrite(uint8_t * buffer, size_t address, size_t size);
uint8_t eepromIsTransferComplete();

// Debug driver
void debugPutc(const char c);

// Telemetry driver
void telemetryPortInit(uint32_t baudrate, uint8_t mode);
void telemetryPortSetDirectionInput();
void telemetryPortSetDirectionOutput();
void sportSendByte(uint8_t byte);
void sportSendByteLoop(uint8_t byte);
void sportStopSendByteLoop();
void sportSendBuffer(const uint8_t * buffer, uint32_t count);
bool telemetryGetByte(uint8_t * byte);
void telemetryClearFifo();
extern uint32_t telemetryErrors;

// soft-serial
void telemetryPortInvertedInit(uint32_t baudrate);

// PCBREV driver
#define HAS_SPORT_UPDATE_CONNECTOR()  false

// Sport update driver
#define sportUpdateInit()
#define SPORT_UPDATE_POWER_ON()
#define SPORT_UPDATE_POWER_OFF()

// Audio driver
void audioInit() ;
void audioEnd() ;
void dacStart();
void dacStop();
void setSampleRate(uint32_t frequency);
#define VOLUME_LEVEL_MAX  23
#define VOLUME_LEVEL_DEF  12
void setScaledVolume(uint8_t volume);
void setVolume(uint8_t volume);
int32_t getVolume();
static inline void initSpeakerEnable() { }
static inline void enableSpeaker() { }
static inline void disableSpeaker() { }
static inline void initHeadphoneTrainerSwitch() { }
static inline void enableHeadphone() { }
static inline void enableTrainer() { }
void audioConsumeCurrentBuffer();
#define audioDisableIrq()               __disable_irq()
#define audioEnableIrq()                __enable_irq()

// Haptic driver
void hapticInit();
void hapticOff();
#if defined(HAPTIC_PWM)
  void hapticOn(uint32_t pwmPercent);
#else
  void hapticOn();
#endif

// Second serial port driver
#if defined(AUX_SERIAL_GPIO)
#define DEBUG_BAUDRATE                  115200
#define AUX_SERIAL
extern uint8_t auxSerialMode;
void auxSerialInit(unsigned int mode, unsigned int protocol);
void auxSerialPutc(char c);
#define auxSerialTelemetryInit(protocol) auxSerialInit(UART_MODE_TELEMETRY, protocol)
void auxSerialSbusInit();
void auxSerialStop();
#define AUX_SERIAL_POWER_ON()
#define AUX_SERIAL__POWER_OFF()
#endif

// BT driver
#define BLUETOOTH_BOOTLOADER_BAUDRATE   230400
#define BLUETOOTH_DEFAULT_BAUDRATE      115200
#if defined(PCBX9E)
#define BLUETOOTH_FACTORY_BAUDRATE      9600
#else
#define BLUETOOTH_FACTORY_BAUDRATE      57600
#endif
#define BT_TX_FIFO_SIZE    64
#define BT_RX_FIFO_SIZE    128
void bluetoothInit(uint32_t baudrate, bool enable);
void bluetoothWriteWakeup();
uint8_t bluetoothIsWriting();
void bluetoothDisable();
#define IS_BLUETOOTH_CHIP_PRESENT()     (false)


// LED driver
void ledInit();
void ledOff();
void ledRed();
void ledGreen();
void ledBlue();

// LCD driver
#define LCD_W                           128
#define LCD_H                           64
#define LCD_DEPTH                       1
#define IS_LCD_RESET_NEEDED()           true
#define LCD_CONTRAST_MIN                10
#define LCD_CONTRAST_MAX                30
#define LCD_CONTRAST_DEFAULT            20

void lcdInit();
void lcdInitFinish();
void lcdOff();

// TODO lcdRefreshWait() stub in simpgmspace and remove LCD_DUAL_BUFFER
#if defined(LCD_DMA) && !defined(LCD_DUAL_BUFFER) && !defined(SIMU)
void lcdRefreshWait();
#else
#define lcdRefreshWait()
#endif
#if defined(PCBX9D) || defined(SIMU) || !defined(__cplusplus)
void lcdRefresh();
#else
void lcdRefresh(bool wait=true); // TODO uint8_t wait to simplify this
#endif
void lcdSetRefVolt(unsigned char val);
void lcdSetContrast();


#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)

#if defined(__cplusplus)
#include "fifo.h"
#include "dmafifo.h"

#if defined(CROSSFIRE)
#define TELEMETRY_FIFO_SIZE             128
#else
#define TELEMETRY_FIFO_SIZE             64
#endif

extern Fifo<uint8_t, TELEMETRY_FIFO_SIZE> telemetryFifo;
typedef DMAFifo<32> AuxSerialRxFifo;
extern AuxSerialRxFifo auxSerialRxFifo;
#endif

#endif // _BOARD_H_
