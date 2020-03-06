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

#ifndef _HAL_H_
#define _HAL_H_

// Keys
#define KEYS_GPIO_REG_MENU            GPIOD->IDR
#define KEYS_GPIO_PIN_MENU            GPIO_Pin_7  // PD.07
#define KEYS_GPIO_REG_EXIT            GPIOD->IDR
#define KEYS_GPIO_PIN_EXIT            GPIO_Pin_2  // PD.02
#define KEYS_GPIO_REG_PAGE            GPIOD->IDR
#define KEYS_GPIO_PIN_PAGE            GPIO_Pin_3  // PD.03
#define KEYS_GPIO_REG_PLUS            GPIOE->IDR
#define KEYS_GPIO_PIN_PLUS            GPIO_Pin_10 // PE.10
#define KEYS_GPIO_REG_MINUS           GPIOE->IDR
#define KEYS_GPIO_PIN_MINUS           GPIO_Pin_11 // PE.11
#define KEYS_GPIO_REG_ENTER           GPIOE->IDR
#define KEYS_GPIO_PIN_ENTER           GPIO_Pin_12 // PE.12

// Rotary Encoder -- we do not have one
#if 0
  #define ROTARY_ENCODER_NAVIGATION
  #define ROTARY_ENCODER_GPIO           GPIOD
  #define ROTARY_ENCODER_GPIO_PIN_A     GPIO_Pin_12 // PD.12
  #define ROTARY_ENCODER_GPIO_PIN_B     GPIO_Pin_13 // PD.13
  #define ROTARY_ENCODER_POSITION()     (ROTARY_ENCODER_GPIO->IDR >> 12) & 0x03
  #define ROTARY_ENCODER_EXTI_LINE1     EXTI_Line12
  #define ROTARY_ENCODER_EXTI_LINE2     EXTI_Line13
  #define ROTARY_ENCODER_EXTI_IRQn1        EXTI15_10_IRQn
  #define ROTARY_ENCODER_EXTI_IRQHandler1  EXTI15_10_IRQHandler
  #define ROTARY_ENCODER_EXTI_PortSource   EXTI_PortSourceGPIOD
  #define ROTARY_ENCODER_EXTI_PinSource1   EXTI_PinSource12
  #define ROTARY_ENCODER_EXTI_PinSource2   EXTI_PinSource13
#endif
#if defined(ROTARY_ENCODER_NAVIGATION)
  #define ROTARY_ENCODER_RCC_APB1Periph   RCC_APB1Periph_TIM5
  #define ROTARY_ENCODER_TIMER            TIM5
  #define ROTARY_ENCODER_TIMER_IRQn       TIM5_IRQn
  #define ROTARY_ENCODER_TIMER_IRQHandler TIM5_IRQHandler
#else
  #define ROTARY_ENCODER_RCC_APB1Periph   0
#endif

// Trims
#define TRIMS_GPIO_REG_LHL            GPIOG->IDR
#define TRIMS_GPIO_PIN_LHL            GPIO_Pin_1  // PG.01
#define TRIMS_GPIO_REG_LHR            GPIOG->IDR
#define TRIMS_GPIO_PIN_LHR            GPIO_Pin_0  // PG.00
#define TRIMS_GPIO_REG_LVD            GPIOE->IDR
#define TRIMS_GPIO_PIN_LVD            GPIO_Pin_4  // PE.04
#define TRIMS_GPIO_REG_LVU            GPIOE->IDR
#define TRIMS_GPIO_PIN_LVU            GPIO_Pin_3  // PE.03
#define TRIMS_GPIO_REG_RVD            GPIOC->IDR
#define TRIMS_GPIO_PIN_RVD            GPIO_Pin_3  // PC.03
#define TRIMS_GPIO_REG_RHL            GPIOC->IDR
#define TRIMS_GPIO_PIN_RHL            GPIO_Pin_1  // PC.01
#define TRIMS_GPIO_REG_RVU            GPIOC->IDR
#define TRIMS_GPIO_PIN_RVU            GPIO_Pin_2  // PC.02
#define TRIMS_GPIO_REG_RHR            GPIOC->IDR
#define TRIMS_GPIO_PIN_RHR            GPIO_Pin_13 // PC.13

// Switches
#define STORAGE_SWITCH_A
#define HARDWARE_SWITCH_A
#define SWITCHES_GPIO_REG_A_H         GPIOB->IDR
#define SWITCHES_GPIO_PIN_A_H         GPIO_Pin_5  // PB.05
#define SWITCHES_GPIO_REG_A_L         GPIOE->IDR
#define SWITCHES_GPIO_PIN_A_L         GPIO_Pin_0  // PE.00

#define STORAGE_SWITCH_B
#define HARDWARE_SWITCH_B
#define SWITCHES_GPIO_REG_B_H         GPIOE->IDR
#define SWITCHES_GPIO_PIN_B_H         GPIO_Pin_1  // PE.01
#define SWITCHES_GPIO_REG_B_L         GPIOE->IDR
#define SWITCHES_GPIO_PIN_B_L         GPIO_Pin_2  // PE.02

#define STORAGE_SWITCH_C
#define HARDWARE_SWITCH_C
#define SWITCHES_GPIO_REG_C_H         GPIOE->IDR
#define SWITCHES_GPIO_PIN_C_H         GPIO_Pin_15 // PE.15
#define SWITCHES_GPIO_REG_C_L         GPIOA->IDR
#define SWITCHES_GPIO_PIN_C_L         GPIO_Pin_5  // PA.05

#define STORAGE_SWITCH_D
#define HARDWARE_SWITCH_D
#define SWITCHES_GPIO_REG_D           GPIOE->IDR
#define SWITCHES_GPIO_PIN_D           GPIO_Pin_7  // PE.07


#define KEYS_RCC_AHB1Periph           (RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE)
#define KEYS_GPIOA_PINS               (SWITCHES_GPIO_PIN_C_L)
#define KEYS_GPIOB_PINS               (SWITCHES_GPIO_PIN_E_L | SWITCHES_GPIO_PIN_E_H | SWITCHES_GPIO_PIN_A_H | SWITCHES_GPIO_PIN_D_L)
#define KEYS_GPIOC_PINS               (TRIMS_GPIO_PIN_RVD | TRIMS_GPIO_PIN_RVU | TRIMS_GPIO_PIN_RHL | TRIMS_GPIO_PIN_RHR)
#define KEYS_GPIOD_PINS               (KEYS_GPIO_PIN_MENU | KEYS_GPIO_PIN_EXIT | KEYS_GPIO_PIN_PAGE)
#define KEYS_GPIOE_PINS               (KEYS_GPIO_PIN_PLUS | KEYS_GPIO_PIN_ENTER | KEYS_GPIO_PIN_MINUS | TRIMS_GPIO_PIN_LHR | TRIMS_GPIO_PIN_LHL | TRIMS_GPIO_PIN_LVD | TRIMS_GPIO_PIN_LVU | SWITCHES_GPIO_PIN_F | SWITCHES_GPIO_PIN_A_L | SWITCHES_GPIO_PIN_B_H | SWITCHES_GPIO_PIN_B_L | SWITCHES_GPIO_PIN_C_H | SWITCHES_GPIO_PIN_D_H | SWITCHES_GPIO_PIN_G_H | SWITCHES_GPIO_PIN_G_L | SWITCHES_GPIO_PIN_H)


// ADC
#define ADC_MAIN                      ADC1
#define ADC_DMA                       DMA2
#define ADC_DMA_SxCR_CHSEL            0
#define ADC_DMA_Stream                DMA2_Stream4
#define ADC_SET_DMA_FLAGS()           ADC_DMA->HIFCR = (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4)
#define ADC_TRANSFER_COMPLETE()       (ADC_DMA->HISR & DMA_HISR_TCIF4)
#define ADC_SAMPTIME                  2   // sample time = 28 cycles
#define HARDWARE_POT1
#define HARDWARE_POT2
#define HARDWARE_POT3
#define ADC_RCC_AHB1Periph            (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2)
#define ADC_RCC_APB1Periph            0
#define ADC_RCC_APB2Periph            RCC_APB2Periph_ADC1
#define ADC_GPIO_PIN_STICK_RV         GPIO_Pin_0  // PA.00
#define ADC_GPIO_PIN_STICK_RH         GPIO_Pin_1  // PA.01
#define ADC_GPIO_PIN_STICK_LH         GPIO_Pin_2  // PA.02
#define ADC_GPIO_PIN_STICK_LV         GPIO_Pin_3  // PA.03
#define ADC_CHANNEL_STICK_RV          ADC_Channel_0  // ADC1_IN0
#define ADC_CHANNEL_STICK_RH          ADC_Channel_1  // ADC1_IN1
#define ADC_CHANNEL_STICK_LH          ADC_Channel_2  // ADC1_IN2
#define ADC_CHANNEL_STICK_LV          ADC_Channel_3  // ADC1_IN3
#define ADC_GPIO_PIN_POT1             GPIO_Pin_6  // PA.06
#define ADC_GPIO_PIN_POT2             GPIO_Pin_0  // PB.00
#define ADC_GPIO_PIN_POT3             GPIO_Pin_1  // PB.01
#define ADC_GPIOB_PINS                (ADC_GPIO_PIN_POT2 | ADC_GPIO_PIN_POT3)
#define ADC_CHANNEL_POT3              ADC_Channel_9
#define ADC_VREF_PREC2                330
#define ADC_GPIO_PIN_SLIDER1          GPIO_Pin_4  // PC.04
#define ADC_GPIO_PIN_SLIDER2          GPIO_Pin_5  // PC.05
#define ADC_GPIO_PIN_BATT             GPIO_Pin_0  // PC.00
#define ADC_GPIOA_PINS                (ADC_GPIO_PIN_STICK_RV | ADC_GPIO_PIN_STICK_RH | ADC_GPIO_PIN_STICK_LH | ADC_GPIO_PIN_STICK_LV | ADC_GPIO_PIN_POT1)
#define ADC_GPIOC_PINS                (ADC_GPIO_PIN_SLIDER1 | ADC_GPIO_PIN_SLIDER2 | ADC_GPIO_PIN_BATT)
#define ADC_CHANNEL_POT1              ADC_Channel_6
#define ADC_CHANNEL_POT2              ADC_Channel_8
#define ADC_CHANNEL_POT3              ADC_Channel_8
#define ADC_CHANNEL_SLIDER1           ADC_Channel_14
#define ADC_CHANNEL_SLIDER2           ADC_Channel_15
#define ADC_CHANNEL_BATT              ADC_Channel_10


// PWR and LED driver
#define PWR_RCC_AHB1Periph              (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE)

#define PWR_SWITCH_GPIO               GPIOD
#define PWR_SWITCH_GPIO_PIN           GPIO_Pin_1  // PD.01
#define PWR_ON_GPIO                   GPIOD
#define PWR_ON_GPIO_PIN               GPIO_Pin_0  // PD.00

#define STATUS_LEDS
#define GPIO_LED_GPIO_ON              GPIO_ResetBits
#define GPIO_LED_GPIO_OFF             GPIO_SetBits
#define LED_RED_GPIO                  GPIOE
#define LED_RED_GPIO_PIN              GPIO_Pin_4  // PE.04
#define LED_GREEN_GPIO                GPIOE
#define LED_GREEN_GPIO_PIN            GPIO_Pin_5  // PE.05

// Internal Module
#define INTMODULE_RCC_APB1Periph      0
#define INTMODULE_RCC_APB2Periph      RCC_APB2Periph_USART1
#define INTMODULE_RCC_AHB1Periph      (RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA2)
#define INTMODULE_PWR_GPIO            GPIOD
#define INTMODULE_PWR_GPIO_PIN        GPIO_Pin_9  // PD.09
#define INTMODULE_GPIO                GPIOB
#define INTMODULE_TX_GPIO_PIN         GPIO_Pin_6  // PB.06
#define INTMODULE_RX_GPIO_PIN         GPIO_Pin_7  // PB.07
#define INTMODULE_GPIO_PinSource_TX   GPIO_PinSource6
#define INTMODULE_GPIO_PinSource_RX   GPIO_PinSource7
#define INTMODULE_USART               USART1
#define INTMODULE_GPIO_AF             GPIO_AF_USART1
#define INTMODULE_USART_IRQHandler    USART1_IRQHandler
#define INTMODULE_USART_IRQn          USART1_IRQn
#define INTMODULE_DMA_STREAM          DMA2_Stream7
#define INTMODULE_DMA_STREAM_IRQ      DMA2_Stream7_IRQn
#define INTMODULE_DMA_STREAM_IRQHandler  DMA2_Stream7_IRQHandler
#define INTMODULE_DMA_FLAG_TC         DMA_IT_TCIF7
#define INTMODULE_DMA_CHANNEL         DMA_Channel_4


// External Module
#define EXTMODULE_RCC_AHB1Periph      (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA2)
#define EXTMODULE_RCC_APB2Periph      RCC_APB2Periph_TIM8
#define EXTMODULE_PWR_GPIO            GPIOD
#define EXTMODULE_PWR_GPIO_PIN        GPIO_Pin_8  // PD.08
#define EXTERNAL_MODULE_PWR_ON()      GPIO_SetBits(EXTMODULE_PWR_GPIO, EXTMODULE_PWR_GPIO_PIN)
#define EXTERNAL_MODULE_PWR_OFF()     GPIO_ResetBits(EXTMODULE_PWR_GPIO, EXTMODULE_PWR_GPIO_PIN)
#define IS_EXTERNAL_MODULE_ON()       (GPIO_ReadInputDataBit(EXTMODULE_PWR_GPIO, EXTMODULE_PWR_GPIO_PIN) == Bit_SET)
#define EXTMODULE_TX_GPIO             GPIOA
#define EXTMODULE_TX_GPIO_PIN         GPIO_Pin_7  // PA.07
#define EXTMODULE_TX_GPIO_PinSource   GPIO_PinSource7
#define EXTMODULE_TIMER               TIM8
#define EXTMODULE_TIMER_TX_GPIO_AF    GPIO_AF_TIM8 // TIM8_CH1N
#define EXTMODULE_TIMER_CC_IRQn       TIM8_CC_IRQn
#define EXTMODULE_TIMER_CC_IRQHandler TIM8_CC_IRQHandler
#define EXTMODULE_TIMER_DMA_CHANNEL   DMA_Channel_7 // TIM8_UP
#define EXTMODULE_TIMER_DMA_STREAM    DMA2_Stream1 // TIM8_UP
#define EXTMODULE_TIMER_DMA_STREAM_IRQn       DMA2_Stream1_IRQn
#define EXTMODULE_TIMER_DMA_STREAM_IRQHandler DMA2_Stream1_IRQHandler
#define EXTMODULE_TIMER_DMA_FLAG_TC           DMA_IT_TCIF1
#define EXTMODULE_TIMER_OUTPUT_ENABLE         TIM_CCER_CC1NE
#define EXTMODULE_TIMER_OUTPUT_POLARITY       TIM_CCER_CC1NP
#define EXTMODULE_TIMER_FREQ                  (PERI2_FREQUENCY * TIMER_MULT_APB2)


// Trainer Port
#define TRAINER_RCC_AHB1Periph        (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1)
#define TRAINER_RCC_APB1Periph        RCC_APB1Periph_TIM3
#define TRAINER_GPIO                  GPIOC
#define TRAINER_IN_GPIO_PIN           GPIO_Pin_8  // PC.08
#define TRAINER_IN_GPIO_PinSource     GPIO_PinSource8
#define TRAINER_OUT_GPIO_PIN          GPIO_Pin_9  // PC.09
#define TRAINER_OUT_GPIO_PinSource    GPIO_PinSource9
#define TRAINER_DETECT_GPIO           GPIOA
#define TRAINER_DETECT_GPIO_PIN       GPIO_Pin_8  // PA.08
#define TRAINER_TIMER                 TIM3
#define TRAINER_TIMER_IRQn            TIM3_IRQn
#define TRAINER_GPIO_AF               GPIO_AF_TIM3
#define TRAINER_DMA                   DMA1
#define TRAINER_DMA_CHANNEL           DMA_Channel_5
#define TRAINER_DMA_STREAM            DMA1_Stream2
#define TRAINER_DMA_IRQn              DMA1_Stream2_IRQn
#define TRAINER_DMA_IRQHandler        DMA1_Stream2_IRQHandler
#define TRAINER_DMA_FLAG_TC           DMA_IT_TCIF2
#define TRAINER_TIMER_IRQn            TIM3_IRQn
#define TRAINER_TIMER_IRQHandler      TIM3_IRQHandler
#define TRAINER_TIMER_FREQ            (PERI1_FREQUENCY * TIMER_MULT_APB1)
#define TRAINER_OUT_CCMR2             TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;
#define TRAINER_IN_CCMR2              TIM_CCMR2_IC3F_0 | TIM_CCMR2_IC3F_1 | TIM_CCMR2_CC3S_0;
#define TRAINER_OUT_COUNTER_REGISTER  TRAINER_TIMER->CCR4
#define TRAINER_IN_COUNTER_REGISTER   TRAINER_TIMER->CCR3
#define TRAINER_SETUP_REGISTER        TRAINER_TIMER->CCR1
#define TRAINER_OUT_INTERRUPT_FLAG    TIM_SR_CC1IF
#define TRAINER_OUT_INTERRUPT_ENABLE  TIM_DIER_CC1IE
#define TRAINER_IN_INTERRUPT_ENABLE   TIM_DIER_CC3IE
#define TRAINER_IN_INTERRUPT_FLAG     TIM_SR_CC3IF
#define TRAINER_OUT_CCER              TIM_CCER_CC4E
#define TRAINER_IN_CCER               TIM_CCER_CC3E
#define TRAINER_CCER_POLARYTY         TIM_CCER_CC4P

// Serial Port
#define TRAINER_BATTERY_COMPARTMENT
#define AUX_SERIAL_RCC_AHB1Periph         (RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1)
#define AUX_SERIAL_RCC_APB1Periph         RCC_APB1Periph_USART3
#define AUX_SERIAL_RCC_APB2Periph         0
#define AUX_SERIAL_GPIO                   GPIOB
#define AUX_SERIAL_GPIO_PIN_TX            GPIO_Pin_10 // PB.10
#define AUX_SERIAL_GPIO_PIN_RX            GPIO_Pin_11 // PB.11
#define AUX_SERIAL_GPIO_PinSource_TX      GPIO_PinSource10
#define AUX_SERIAL_GPIO_PinSource_RX      GPIO_PinSource11
#define AUX_SERIAL_GPIO_AF                GPIO_AF_USART3
#define AUX_SERIAL_USART                  USART3
#define AUX_SERIAL_USART_IRQHandler       USART3_IRQHandler
#define AUX_SERIAL_USART_IRQn             USART3_IRQn
#define AUX_SERIAL_DMA_Stream_RX          DMA1_Stream1
#define AUX_SERIAL_DMA_Channel_RX         DMA_Channel_4

// Telemetry
#define TELEMETRY_RCC_AHB1Periph        (RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1)
#define TELEMETRY_RCC_APB1Periph        RCC_APB1Periph_USART2
#define TELEMETRY_RCC_APB2Periph        RCC_APB2Periph_TIM11
#define TELEMETRY_DIR_GPIO              GPIOD
#define TELEMETRY_DIR_GPIO_PIN          GPIO_Pin_4  // PD.04
#define TELEMETRY_DIR_OUTPUT()          TELEMETRY_DIR_GPIO->BSRRL = TELEMETRY_DIR_GPIO_PIN
#define TELEMETRY_DIR_INPUT()           TELEMETRY_DIR_GPIO->BSRRH = TELEMETRY_DIR_GPIO_PIN
#define TELEMETRY_GPIO                  GPIOD
#define TELEMETRY_TX_GPIO_PIN           GPIO_Pin_5  // PD.05
#define TELEMETRY_RX_GPIO_PIN           GPIO_Pin_6  // PD.06
#define TELEMETRY_GPIO_PinSource_TX     GPIO_PinSource5
#define TELEMETRY_GPIO_PinSource_RX     GPIO_PinSource6
#define TELEMETRY_GPIO_AF               GPIO_AF_USART2
#define TELEMETRY_USART                 USART2
#define TELEMETRY_DMA_Stream_TX         DMA1_Stream6
#define TELEMETRY_DMA_Channel_TX        DMA_Channel_4
#define TELEMETRY_DMA_TX_Stream_IRQ     DMA1_Stream6_IRQn
#define TELEMETRY_DMA_TX_IRQHandler     DMA1_Stream6_IRQHandler
#define TELEMETRY_DMA_TX_FLAG_TC        DMA_IT_TCIF6
#define TELEMETRY_USART_IRQHandler      USART2_IRQHandler
#define TELEMETRY_USART_IRQn            USART2_IRQn
#define TELEMETRY_EXTI_PortSource       EXTI_PortSourceGPIOD
#define TELEMETRY_EXTI_PinSource        EXTI_PinSource6
#define TELEMETRY_EXTI_LINE             EXTI_Line6
#define TELEMETRY_EXTI_IRQn             EXTI9_5_IRQn
#define TELEMETRY_EXTI_TRIGGER          EXTI_Trigger_Rising

#if defined(RADIO_X7) || defined(RADIO_X7ACCESS)
  #define TELEMETRY_EXTI_REUSE_INTERRUPT_ROTARY_ENCODER
#elif defined(PCBXLITE) || defined(PCBX9LITE) || defined(RADIO_X9DP2019)
  #define TELEMETRY_EXTI_IRQHandler       EXTI9_5_IRQHandler
#else
  #define TELEMETRY_EXTI_REUSE_INTERRUPT_INTMODULE_HEARTBEAT
#endif

#define TELEMETRY_TIMER                 TIM11
#define TELEMETRY_TIMER_IRQn            TIM1_TRG_COM_TIM11_IRQn
#define TELEMETRY_TIMER_IRQHandler      TIM1_TRG_COM_TIM11_IRQHandler


// S.Port update connector
#define SPORT_MAX_BAUDRATE            400000
#define SPORT_UPDATE_RCC_AHB1Periph   0


// Heartbeat for iXJT / ISRM synchro
#define INTMODULE_HEARTBEAT_TRIGGER               EXTI_Trigger_Falling
#if 0
  #define INTMODULE_HEARTBEAT
  #define INTMODULE_HEARTBEAT_RCC_AHB1Periph      RCC_AHB1Periph_GPIOC
  #define INTMODULE_HEARTBEAT_GPIO                GPIOC
  #define INTMODULE_HEARTBEAT_GPIO_PIN            GPIO_Pin_7
  #define INTMODULE_HEARTBEAT_EXTI_PortSource     EXTI_PortSourceGPIOC
  #define INTMODULE_HEARTBEAT_EXTI_PinSource      GPIO_PinSource7
  #define INTMODULE_HEARTBEAT_EXTI_LINE           EXTI_Line7
  #define INTMODULE_HEARTBEAT_EXTI_IRQn           EXTI9_5_IRQn
  #define INTMODULE_HEARTBEAT_EXTI_IRQHandler     EXTI9_5_IRQHandler
#endif

// Trainer / Trainee from the module bay
#if 0
  #define TRAINER_MODULE_CPPM
  #define TRAINER_MODULE_SBUS
  #define TRAINER_MODULE_RCC_AHB1Periph      RCC_AHB1Periph_GPIOC
  #define TRAINER_MODULE_RCC_APB2Periph      RCC_APB2Periph_USART6
  #define TRAINER_MODULE_RCC_APB1Periph      RCC_APB1Periph_TIM3
  #define TRAINER_MODULE_CPPM_TIMER          TRAINER_TIMER
  #define TRAINER_MODULE_CPPM_GPIO           INTMODULE_HEARTBEAT_GPIO
  #define TRAINER_MODULE_CPPM_GPIO_PIN       INTMODULE_HEARTBEAT_GPIO_PIN
  #define TRAINER_MODULE_CPPM_GPIO_PinSource INTMODULE_HEARTBEAT_EXTI_PinSource
  #define TRAINER_MODULE_CPPM_INTERRUPT_ENABLE TIM_DIER_CC2IE
  #define TRAINER_MODULE_CPPM_INTERRUPT_FLAG   TIM_SR_CC2IF
  #define TRAINER_MODULE_CPPM_CCMR1            (TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_CC2S_0)
  #define TRAINER_MODULE_CPPM_CCER             TIM_CCER_CC2E
  #define TRAINER_MODULE_CPPM_COUNTER_REGISTER TRAINER_TIMER->CCR2
  #define TRAINER_MODULE_CPPM_TIMER_IRQn       TRAINER_TIMER_IRQn
  #define TRAINER_MODULE_CPPM_GPIO_AF          GPIO_AF_TIM3
  #define TRAINER_MODULE_SBUS_GPIO_AF          GPIO_AF_USART6
  #define TRAINER_MODULE_SBUS_USART            USART6
  #define TRAINER_MODULE_SBUS_GPIO             INTMODULE_HEARTBEAT_GPIO
  #define TRAINER_MODULE_SBUS_GPIO_PIN         INTMODULE_HEARTBEAT_GPIO_PIN
  #define TRAINER_MODULE_SBUS_GPIO_PinSource   INTMODULE_HEARTBEAT_EXTI_PinSource
  #define TRAINER_MODULE_SBUS_DMA_STREAM       DMA2_Stream1
  #define TRAINER_MODULE_SBUS_DMA_CHANNEL      DMA_Channel_5
#endif

// USB
#define USB_RCC_AHB1Periph_GPIO         RCC_AHB1Periph_GPIOA
#define USB_GPIO                        GPIOA
#define USB_GPIO_PIN_VBUS               GPIO_Pin_9  // PA.09
#define USB_GPIO_PIN_DM                 GPIO_Pin_11 // PA.11
#define USB_GPIO_PIN_DP                 GPIO_Pin_12 // PA.12
#define USB_GPIO_PinSource_DM           GPIO_PinSource11
#define USB_GPIO_PinSource_DP           GPIO_PinSource12
#define USB_GPIO_AF                     GPIO_AF_OTG1_FS

// BackLight
#define BACKLIGHT_RCC_AHB1Periph      RCC_AHB1Periph_GPIOB
#define BACKLIGHT_RCC_APB1Periph      0
#define BACKLIGHT_RCC_APB2Periph      RCC_APB2Periph_TIM10
#define BACKLIGHT_TIMER_FREQ          (PERI2_FREQUENCY * TIMER_MULT_APB2)
#define BACKLIGHT_TIMER               TIM10
#define BACKLIGHT_GPIO                GPIOB
#define BACKLIGHT_GPIO_PIN            GPIO_Pin_8  // PB.08
#define BACKLIGHT_GPIO_PinSource      GPIO_PinSource8
#define BACKLIGHT_GPIO_AF             GPIO_AF_TIM10

// LCD driver
#define LCD_RCC_AHB1Periph            (RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1)
#define LCD_RCC_APB1Periph            RCC_APB1Periph_SPI3
#define LCD_SPI_GPIO                  GPIOC
#define LCD_MOSI_GPIO_PIN             GPIO_Pin_12 // PC.12
#define LCD_MOSI_GPIO_PinSource       GPIO_PinSource12
#define LCD_CLK_GPIO_PIN              GPIO_Pin_10 // PC.10
#define LCD_CLK_GPIO_PinSource        GPIO_PinSource10
#define LCD_A0_GPIO_PIN               GPIO_Pin_11 // PC.11
#define LCD_NCS_GPIO                  GPIOD
#define LCD_NCS_GPIO_PIN              GPIO_Pin_3  // PD.03
#define LCD_RST_GPIO                  GPIOD
#define LCD_RST_GPIO_PIN              GPIO_Pin_2  // PD.02
#define LCD_DMA                       DMA1
#define LCD_DMA_Stream                DMA1_Stream7
#define LCD_DMA_Stream_IRQn           DMA1_Stream7_IRQn
#define LCD_DMA_Stream_IRQHandler     DMA1_Stream7_IRQHandler
#define LCD_DMA_FLAGS                 (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7)
#define LCD_DMA_FLAG_INT              DMA_HIFCR_CTCIF7
#define LCD_SPI                       SPI3
#define LCD_GPIO_AF                   GPIO_AF_SPI3
#define LCD_RCC_APB2Periph              0

// I2C Bus: EEPROM and CAT5137 digital pot for volume control
#define I2C_RCC_APB1Periph              RCC_APB1Periph_I2C1
#define I2C                             I2C1
#define I2C_GPIO_AF                     GPIO_AF_I2C1
#define I2C_RCC_AHB1Periph            RCC_AHB1Periph_GPIOB
#define I2C_SPI_GPIO                  GPIOB
#define I2C_SCL_GPIO_PIN              GPIO_Pin_6  // PB.06
#define I2C_SDA_GPIO_PIN              GPIO_Pin_7  // PB.07
#define I2C_WP_GPIO                   GPIOB
#define I2C_WP_GPIO_PIN               GPIO_Pin_9  // PB.09
#define I2C_SCL_GPIO_PinSource        GPIO_PinSource6
#define I2C_SDA_GPIO_PinSource        GPIO_PinSource7
#define I2C_ADDRESS_VOLUME            0x5C
#define I2C_SPEED                       400000
#define I2C_ADDRESS_EEPROM              0xA2
#define I2C_FLASH_PAGESIZE              64

// Second I2C Bus: IMU
#define GYRO_RCC_AHB1Periph           0
#define GYRO_RCC_APB1Periph           0

// SD -- no sdcard


// Audio
#define AUDIO_RCC_APB1Periph            (RCC_APB1Periph_TIM6 | RCC_APB1Periph_DAC)
#define AUDIO_OUTPUT_GPIO               GPIOA
#define AUDIO_OUTPUT_GPIO_PIN           GPIO_Pin_4  // PA.04
#define AUDIO_DMA_Stream                DMA1_Stream5
#define AUDIO_DMA_Stream_IRQn           DMA1_Stream5_IRQn
#define AUDIO_TIM_IRQn                  TIM6_DAC_IRQn
#define AUDIO_TIM_IRQHandler            TIM6_DAC_IRQHandler
#define AUDIO_DMA_Stream_IRQHandler     DMA1_Stream5_IRQHandler
#define AUDIO_TIMER                     TIM6
#define AUDIO_DMA                       DMA1
#define AUDIO_RCC_AHB1Periph          (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1)


// Haptic
#define HAPTIC_PWM
#define HAPTIC_RCC_AHB1Periph         RCC_AHB1Periph_GPIOB
#define HAPTIC_RCC_APB1Periph         RCC_APB1Periph_TIM2
#define HAPTIC_RCC_APB2Periph         0
#define HAPTIC_GPIO_PinSource         GPIO_PinSource3
#define HAPTIC_GPIO                   GPIOB
#define HAPTIC_GPIO_PIN               GPIO_Pin_3  // PB.03
#define HAPTIC_GPIO_AF                GPIO_AF_TIM2
#define HAPTIC_TIMER                  TIM2
#define HAPTIC_TIMER_FREQ             (PERI1_FREQUENCY * TIMER_MULT_APB1)
#define HAPTIC_COUNTER_REGISTER       HAPTIC_TIMER->CCR2
#define HAPTIC_CCMR1                  TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2
#define HAPTIC_CCER                   TIM_CCER_CC2E
#define BACKLIGHT_BDTR                TIM_BDTR_MOE


// Top LCD on X9E


// Bluetooth
#define BT_RCC_AHB1Periph             0
#define BT_RCC_APB1Periph             0
#define BT_RCC_APB2Periph             0


// Xms Interrupt
#define INTERRUPT_xMS_RCC_APB1Periph    RCC_APB1Periph_TIM14
#define INTERRUPT_xMS_TIMER             TIM14
#define INTERRUPT_xMS_IRQn              TIM8_TRG_COM_TIM14_IRQn
#define INTERRUPT_xMS_IRQHandler        TIM8_TRG_COM_TIM14_IRQHandler


// 2MHz Timer
#define TIMER_2MHz_RCC_APB1Periph       RCC_APB1Periph_TIM7
#define TIMER_2MHz_TIMER                TIM7


#endif // _HAL_H_
