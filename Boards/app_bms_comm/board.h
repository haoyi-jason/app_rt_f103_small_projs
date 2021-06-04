/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the Artery AT32F413 Board
 */

/*
 * Board identifier.
 */
#define BOARD_AT32F4
#define BOARD_NAME              "BMS_COMM"
#define BOARD_ID                0x32f4001      // 0x06080002: BMU, 0x11150001: HV-ADS1015, 0x29490001: HV-2949, 0x32F40001: comm board
#define FW_VERSION              0x21050301      // datecode for firmware version            
#define NOF_CANFRAME_HANDLER             7
/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            8000000


/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 * Note: Older board revisions should define STM32F10X_HD instead, please
 *       verify the STM32 model mounted on your board. The change also
 *       affects your linker script.
 */
#define STM32F103xB
#define AT32F413Cx_MD
#define AT32F413xx
#define AT32F4XX

#define STM32_VDD 300U

/*
  IO Pin Assignnment
*/

#define GPIOA_P0	        0
#define GPIOA_P1        	1
#define GPIOA_ISM_INT1        	2
#define GPIOA_ISM_INT2        	3
#define GPIOA_SPI1_ADXL_CS	4
#define GPIOA_SPI1_SCK		5
#define GPIOA_SPI1_MISO		6
#define GPIOA_SPI1_MOSI		7

#define GPIOA_LED_R        	8
#define GPIOA_UART1_TX		9
#define GPIOA_UART1_RX		10
#define GPIOA_UART1_CTS		11 //IRQ
#define GPIOA_UART1_RTS		12 // HIB
#define GPIOA_P13        	13
#define GPIOA_P14        	14
#define GPIOA_P15        	15

#define GPIOB_ADXL_DRDY	        0
#define GPIOB_ADXL_INT2        	1
#define GPIOB_ADXL_INT1        	2
#define GPIOB_3                	3
#define GPIOB_4        	        4
#define GPIOB_5        	        5
#define GPIOB_I2C1_SCK        	6
#define GPIOB_I2C1_SDA        	7
#define GPIOB_P8        	8
#define GPIOB_P9        	9
#define GPIOB_P10        	10
#define GPIOB_P11        	11
#define GPIOB_SPI2_CS		12
#define GPIOB_SPI2_SCK		13
#define GPIOB_SPI2_MISO		14
#define GPIOB_SPI2_MOSI		15
                            
/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_ANALOG(n)           (0 << (((n) & 7) * 4))
#define PIN_OUTPUT_PP_10(n)     (1 << (((n) & 7) * 4))
#define PIN_OUTPUT_PP_2(n)      (2 << (((n) & 7) * 4))
#define PIN_OUTPUT_PP_50(n)     (3 << (((n) & 7) * 4))
#define PIN_INPUT(n)            (4 << (((n) & 7) * 4))
#define PIN_OUTPUT_OD_10(n)     (5 << (((n) & 7) * 4))
#define PIN_OUTPUT_OD_2(n)      (6 << (((n) & 7) * 4))
#define PIN_OUTPUT_OD_50(n)     (7 << (((n) & 7) * 4))
#define PIN_INPUT_PUD(n)        (8 << (((n) & 7) * 4))
#define PIN_ALTERNATE_PP_10(n)  (9 << (((n) & 7) * 4))
#define PIN_ALTERNATE_PP_2(n)   (10 << (((n) & 7) * 4))
#define PIN_ALTERNATE_PP_50(n)  (11 << (((n) & 7) * 4))
#define PIN_ALTERNATE_OD_10(n)  (13 << (((n) & 7) * 4))
#define PIN_ALTERNATE_OD_2(n)   (14 << (((n) & 7) * 4))
#define PIN_ALTERNATE_OD_50(n)  (15 << (((n) & 7) * 4))
#define PIN_UNDEFINED(n)        PIN_INPUT_PUD(n)


/*
 * Port A setup.
 */
#define VAL_GPIOACRL    (PIN_INPUT(0)   | /* SMU_CPCK     */  \
                         PIN_UNDEFINED(1)    | /* SMU_CPOL        */  \
                         PIN_UNDEFINED(2)   | /* SMU_CPOH        */  \
                         PIN_INPUT(3)           | /* Not Used         */  \
                         PIN_UNDEFINED(4)    | /* SPI1_CS */            \
                         PIN_UNDEFINED(5) | /* SPI1_SCK.          */  \
                         PIN_INPUT(6)           | /* SPI1_MISO.         */  \
                         PIN_UNDEFINED(7))  /* SPI1_MOSI.         */
#define VAL_GPIOACRH    (PIN_OUTPUT_PP_2(8) | /* I2C2.SCL              */  \
                         PIN_OUTPUT_PP_2(9) | /* USART1_TX.         */  \
                         PIN_UNDEFINED(10)          | /* USART1_RX.         */  \
                         PIN_INPUT_PUD(11)      | /* USART1_CTS            */  \
                         PIN_INPUT_PUD(12)      | /* USART1_RTS            */  \
                         PIN_UNDEFINED(13)          | /* Not use              */  \
                         PIN_UNDEFINED(14)          | /* not use            */  \
                         PIN_OUTPUT_PP_2(15))     /* SMU_STB               */
#define VAL_GPIOAODR    0xFFFF8EF7

/*
 * Port B setup.
 */
#define VAL_GPIOBCRL    (PIN_UNDEFINED(0)    | /* SmartCard_3/5V.    */  \
                         PIN_UNDEFINED(1)       | /* Unconnected.       */  \
                         PIN_INPUT_PUD(2)    | /* SPI1_CS.           */  \
                         PIN_OUTPUT_PP_2(3)           | /* TDO.               */  \
                         PIN_INPUT_PUD(4)           | /* TRST.              */  \
                         PIN_INPUT_PUD(5)       | /* CAN2_RX  */  \
                         PIN_ALTERNATE_PP_50(6) | /* CAN2_TX          */  \
                         PIN_INPUT_PUD(7))  /* I2C1_SDA.          */
#define VAL_GPIOBCRH    (PIN_INPUT_PUD(8)           | /* CAN_RX.            */  \
                         PIN_ALTERNATE_PP_50(9) | /* CAN_TX.            */  \
                         PIN_ALTERNATE_OD_2(10)| /* SmartCard IO.      */  \
                         PIN_ALTERNATE_OD_2(11)|           /* SPI2.CS     */  \
                         PIN_UNDEFINED(12)|           /* SPI2.CLK     */  \
                         PIN_UNDEFINED(13)|       /* SPI2.MOSI */                    \
                         PIN_UNDEFINED(14)|       /* SPI2.MISO.    */  \
                         PIN_OUTPUT_PP_2(15))
#define VAL_GPIOBODR    0xFFFF7FF7

/*
 * Port C setup.
 */
#define VAL_GPIOCCRL    (PIN_UNDEFINED(0)       |                           \
                         PIN_UNDEFINED(1)       |                           \
                         PIN_OUTPUT_PP_2(2)       |                           \
                         PIN_OUTPUT_PP_2(3)       |                           \
                         PIN_INPUT_PUD(4)          | /* Ethernet IRQ.     */  \
                         PIN_OUTPUT_PP_2(5)       |                           \
                         PIN_UNDEFINED(6)    | /* SmartCard CMDVCC.  */  \
                         PIN_UNDEFINED(7))            /* SmartCard OFF.     */
#define VAL_GPIOCCRH    (PIN_UNDEFINED(8) | /* SDIO D0.           */  \
                         PIN_ALTERNATE_OD_10(9) | /* I2C2.SDA           */  \
                         PIN_ALTERNATE_PP_50(10)| /* UART4.TX          */  \
                         PIN_INPUT_PUD(11)| /* UART4.RX         */  \
                         PIN_ALTERNATE_PP_50(12)| /* UART5.TX          */  \
                         PIN_INPUT_PUD(13)          | /* Tamper Button.     */  \
                         PIN_OUTPUT_PP_50(14)          | /* OSC IN.            */  \
                         PIN_OUTPUT_PP_2(15))           /* OSC OUT.           */
#define VAL_GPIOCODR    0xFFFFDFFF

/*
 * Port D setup.
 * Everything input with pull-up except:
 * PD0  - Normal input (XTAL).
 * PD1  - Normal input (XTAL).
 */
#define VAL_GPIODCRL    (PIN_UNDEFINED(0)       |                           \
                         PIN_UNDEFINED(1)       |                           \
                         PIN_INPUT(2)       |               /*UART5.RX*/            \
                         PIN_INPUT(3)       |                           \
                         PIN_UNDEFINED(4)          | /* Potentiometer.     */  \
                         PIN_INPUT(5)       |                           \
                         PIN_UNDEFINED(6)    | /* SmartCard CMDVCC.  */  \
                         PIN_UNDEFINED(7))            /* SmartCard OFF.     */
#define VAL_GPIODCRH    (PIN_UNDEFINED(8) | /* SDIO D0.           */  \
                         PIN_UNDEFINED(9) | /* SDIO D1.           */  \
                         PIN_UNDEFINED(10)| /* SDIO D2.           */  \
                         PIN_UNDEFINED(11)| /* SDIO D3.           */  \
                         PIN_UNDEFINED(12)| /* SDIO CLK.          */  \
                         PIN_OUTPUT_PP_50(13)          | /* Tamper Button.     */  \
                         PIN_OUTPUT_PP_50(14)          | /* OSC IN.            */  \
                         PIN_INPUT_PUD(15))           /* OSC OUT.           */
#define VAL_GPIODODR            0xFFFFFFFF

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  
#define LINE_ISO_DI_0        PAL_LINE(GPIOC,13)
#define LINE_ISO_DI_1        PAL_LINE(GPIOB,7)
#define LINE_RELAY_OUT_0     PAL_LINE(GPIOB,15)
#define LINE_RELAY_OUT_1     PAL_LINE(GPIOA,8)
#define LINE_VSOURCE_0       PAL_LINE(GPIOA,15)
#define LINE_VSOURCE_1       PAL_LINE(GPIOB,3)
#define LINE_AD5593_RST      PAL_LINE(GPIOA,9)
#define LINE_PWM_0           PAL_LINE(GPIOA,3)
#define LINE_PWM_1           PAL_LINE(GPIOA,6)
  
void boardInit(void);

void board_relay_set(uint8_t x);
void board_relay_clr(uint8_t x);
void board_power_set(uint8_t x);
void board_power_clr(uint8_t x);
void baord_adc_reset();
int8_t board_input_state(uint8_t x);
int8_t board_output_state(uint8_t x);
  
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
