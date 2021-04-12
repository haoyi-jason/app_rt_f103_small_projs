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
#define BOARD_NAME              "BMU-6811"
#define BOARD_ID                0x06080002

#define NOF_MAX_CELL_PER_MODULE         12
#define NOF_MAX_AUXIO                   5

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
#define AT32F413xx
#define AT32F4XX
#define AT32F413Rx_HD
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
#define VAL_GPIOACRL    (PIN_ALTERNATE_PP_50(0)   | /* SMU_CPCK     */  \
                         PIN_ALTERNATE_PP_50(1)    | /* SMU_CPOL        */  \
                         PIN_ALTERNATE_PP_50(2)   | /* SMU_CPOH        */  \
                         PIN_OUTPUT_PP_50(3)           | /* Not Used         */  \
                         PIN_OUTPUT_PP_50(4)    | /* SPI1_CS */            \
                         PIN_ALTERNATE_PP_50(5) | /* SPI1_SCK.          */  \
                         PIN_ALTERNATE_PP_50(6)           | /* SPI1_MISO.         */  \
                         PIN_ALTERNATE_PP_50(7))  /* SPI1_MOSI.         */
#define VAL_GPIOACRH    (PIN_ALTERNATE_OD_10(8) | /* I2C2.SCL              */  \
                         PIN_OUTPUT_PP_50(9) | /* USART1_TX.         */  \
                         PIN_INPUT(10)          | /* USART1_RX.         */  \
                         PIN_INPUT_PUD(11)      | /* USART1_CTS            */  \
                         PIN_INPUT_PUD(12)      | /* USART1_RTS            */  \
                         PIN_UNDEFINED(13)          | /* Not use              */  \
                         PIN_UNDEFINED(14)          | /* not use            */  \
                         PIN_INPUT_PUD(15))     /* SMU_STB               */
#define VAL_GPIOAODR    0xFFFFFFF7

/*
 * Port B setup.
 */
#define VAL_GPIOBCRL    (PIN_OUTPUT_PP_2(0)    | /* SmartCard_3/5V.    */  \
                         PIN_OUTPUT_PP_2(1)       | /* Unconnected.       */  \
                         PIN_INPUT_PUD(2)    | /* SPI1_CS.           */  \
                         PIN_OUTPUT_PP_2(3)           | /* TDO.               */  \
                         PIN_INPUT_PUD(4)           | /* TRST.              */  \
                         PIN_ALTERNATE_PP_50(5)       | /* CAN2_RX  */  \
                         PIN_ALTERNATE_PP_50(6) | /* CAN2_TX          */  \
                         PIN_ALTERNATE_OD_10(7))  /* I2C1_SDA.          */
#define VAL_GPIOBCRH    (PIN_INPUT_PUD(8)           | /* CAN_RX.            */  \
                         PIN_ALTERNATE_PP_50(9) | /* CAN_TX.            */  \
                         PIN_ALTERNATE_PP_50(10)| /* SmartCard IO.      */  \
                         PIN_OUTPUT_PP_2(11)|           /* SPI2.CS     */  \
                         PIN_OUTPUT_PP_2(12)|           /* SPI2.CLK     */  \
                         PIN_ALTERNATE_PP_50(13)|       /* SPI2.MOSI */                    \
                         PIN_ALTERNATE_PP_50(14)|       /* SPI2.MISO.    */  \
                         PIN_ALTERNATE_PP_50(15))
#define VAL_GPIOBODR    0xFFFFFFFE

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
                         PIN_OUTPUT_PP_50(13)          | /* Tamper Button.     */  \
                         PIN_OUTPUT_PP_50(14)          | /* OSC IN.            */  \
                         PIN_INPUT_PUD(15))           /* OSC OUT.           */
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
                         PIN_OUTPUT_PP_2(3)       |                           \
                         PIN_UNDEFINED(4)          | /* Potentiometer.     */  \
                         PIN_OUTPUT_PP_2(5)       |                           \
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
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
