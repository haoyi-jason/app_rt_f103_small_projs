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
 * Setup for the STMicroelectronics STM3210E-EVAL evaluation board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM3210E_EVAL
#define BOARD_NAME              "SNODE_F4"
#define SERIAL_LOADER_IF        SD1

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
#define STM32F411xE
#define STM32F4XX

#define STM32_VDD 300U

#define ADXL355_USE_SPI 1

//#define STM32_I2S_SPI3_MODE STM32_I2S_MODE_MASTER | STM32_I2S_MODE_TX
//#define STM32_SDC_USE_SDMMC1    true
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
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))


/*
 * GPIOA setup:
 *
 * PA0  - GPIOA_UART4_TX      (AF.8)
 * PA1  - GPIOA_UART4_RX      (AF.8)
 * PA2  - GPIOA_2             (IN)
 * PA3  - GPIOA_3             (IN)
 * PA4  - GPIOA_SPI1_CS       (OUT.PP)
 * PA5  - GPIOA_SPI1_SCK      (AF.5)
 * PA6  - GPIOA_SPI1_MISO     (AF.5)
 * PA7  - GPIOA_SPI1_MOSI     (AF.5)
 * PA8  - GPIOA_ADC_INT       (IN.PU)
 * PA9  - GPIOA_UART1_TX      (AF.7)
 * PA10 - GPIOA_UART1_RX      (AF.7)
 * PA11 - GPIOA_11            (IN)
 * PA12 - GPIOA_12            (IN)
 * PA13 - GPIOA_SWDIO         (AF.0)
 * PA14 - GPIOA_SWCLK         (AF.0)
 * PA15 - GPIOA_YLED          (OUT.PP)
 */

#define VAL_GPIOA_MODER             (PIN_MODE_ANALOG(GPIOA_P0)            |\
                                     PIN_MODE_ANALOG(GPIOA_P1)            |\
                                     PIN_MODE_INPUT(GPIOA_ISM_INT1)             |\
                                     PIN_MODE_INPUT(GPIOA_ISM_INT2 )      |\
                                     PIN_MODE_OUTPUT(GPIOA_SPI1_ADXL_CS)  |\
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_SCK )  |\
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MISO)      |\
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MOSI)  |\
                                     PIN_MODE_OUTPUT(GPIOA_LED_R )        |\
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_TX )      |\
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_RX )     |\
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_CTS)     |\
                                     PIN_MODE_ALTERNATE(GPIOA_UART1_RTS)     |\
                                     PIN_MODE_ALTERNATE(GPIOA_P13)        |\
                                     PIN_MODE_ALTERNATE(GPIOA_P14)        |\
                                     PIN_MODE_OUTPUT(GPIOA_P15  ))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_P0	  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_P1          )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_ISM_INT1          )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_ISM_INT2     )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_ADXL_CS)        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_SCK	  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MISO	  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MOSI	  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_LED_R       )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_TX	  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_RX	  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_CTS	  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_RTS	  )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_P13         )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_P14         )        |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_P15         ))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_P0	       )           |\
                                     PIN_OSPEED_HIGH(GPIOA_P1          )           |\
                                     PIN_OSPEED_HIGH(GPIOA_ISM_INT1          )           |\
                                     PIN_OSPEED_HIGH(GPIOA_ISM_INT2     )           |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_ADXL_CS)           |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_SCK)            |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MISO)           |\
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MOSI)           |\
                                     PIN_OSPEED_HIGH(GPIOA_LED_R)           |\
                                     PIN_OSPEED_HIGH(GPIOA_UART1_TX)           |\
                                     PIN_OSPEED_HIGH(GPIOA_UART1_RX)           |\
                                     PIN_OSPEED_HIGH(GPIOA_UART1_CTS)               |\
                                     PIN_OSPEED_HIGH(GPIOA_UART1_RTS)           |\
                                     PIN_OSPEED_HIGH(GPIOA_P13)           |\
                                     PIN_OSPEED_HIGH(GPIOA_P14)           |\
                                     PIN_OSPEED_HIGH(GPIOA_P15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_P0	  )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_P1          )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_ISM_INT1          )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_ISM_INT2     )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_ADXL_CS)           |\
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_SCK	  )            |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_SPI1_MISO	  )           |\
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_MOSI	  )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_LED_R         )         |\
                                     PIN_PUPDR_FLOATING(GPIOA_UART1_TX	  )         |\
                                     PIN_PUPDR_FLOATING(GPIOA_UART1_RX	  )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_UART1_CTS	    )             |\
                                     PIN_PUPDR_PULLUP(GPIOA_UART1_RTS	    )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_P13         )         |\
                                     PIN_PUPDR_PULLDOWN(GPIOA_P14         )         |\
                                     PIN_PUPDR_PULLUP(GPIOA_P15           ))
#define VAL_GPIOA_ODR               ( PIN_ODR_LOW(GPIOA_P0	  )               |\
                                      PIN_ODR_LOW(GPIOA_P1          )               |\
                                     PIN_ODR_HIGH(GPIOA_ISM_INT1          )               |\
                                     PIN_ODR_HIGH(GPIOA_ISM_INT2     )               |\
                                     PIN_ODR_HIGH(GPIOA_SPI1_ADXL_CS)               |\
                                     PIN_ODR_HIGH(GPIOA_SPI1_SCK	  )                |\
                                     PIN_ODR_HIGH(GPIOA_SPI1_MISO	  )               |\
                                     PIN_ODR_HIGH(GPIOA_SPI1_MOSI	  )               |\
                                     PIN_ODR_HIGH(GPIOA_LED_R)                      |\
                                     PIN_ODR_HIGH(GPIOA_UART1_TX	  )               |\
                                     PIN_ODR_HIGH(GPIOA_UART1_RX	  )               |\
                                     PIN_ODR_HIGH(GPIOA_UART1_CTS	    )                   |\
                                     PIN_ODR_HIGH(GPIOA_UART1_RTS	    )               |\
                                     PIN_ODR_HIGH(GPIOA_P13         )               |\
                                     PIN_ODR_HIGH(GPIOA_P14         )               |\
                                     PIN_ODR_HIGH(GPIOA_P15           ))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_P0	   ,0)            |\
                                     PIN_AFIO_AF(GPIOA_P1          ,0)            |\
                                     PIN_AFIO_AF(GPIOA_ISM_INT1          ,0)            |\
                                     PIN_AFIO_AF(GPIOA_ISM_INT2     ,0)            |\
                                     PIN_AFIO_AF(GPIOA_SPI1_ADXL_CS,0)            |\
                                     PIN_AFIO_AF(GPIOA_SPI1_SCK	   ,5)             |\
                                     PIN_AFIO_AF(GPIOA_SPI1_MISO   ,5)            |\
                                     PIN_AFIO_AF(GPIOA_SPI1_MOSI   ,5))            
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_LED_R       ,0)             |\
                                     PIN_AFIO_AF(GPIOA_UART1_TX	   ,7)             |\
                                     PIN_AFIO_AF(GPIOA_UART1_RX	   ,7)             |\
                                     PIN_AFIO_AF(GPIOA_UART1_CTS   ,10)                 |\
                                     PIN_AFIO_AF(GPIOA_UART1_RTS   ,10)             |\
                                     PIN_AFIO_AF(GPIOA_P13         ,0)             |\
                                     PIN_AFIO_AF(GPIOA_P14         ,0)             |\
                                     PIN_AFIO_AF(GPIOA_P15         ,0))
/*
 * GPIOB setup:
 *
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_ADXL_DRDY)        |\
                                     PIN_MODE_INPUT(GPIOB_ADXL_INT2)        |\
                                     PIN_MODE_INPUT(GPIOB_ADXL_INT1)     |\
                                     PIN_MODE_INPUT(GPIOB_3        )        |\
                                     PIN_MODE_INPUT(GPIOB_4        )       |\
                                     PIN_MODE_INPUT(GPIOB_5        )        |\
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCK )       |\
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA )       |\
                                     PIN_MODE_INPUT(GPIOB_P8        )       |\
                                     PIN_MODE_INPUT(GPIOB_P9           )          |\
                                     PIN_MODE_INPUT(GPIOB_P10      )       |\
                                     PIN_MODE_INPUT(GPIOB_P11      )       |\
                                     PIN_MODE_OUTPUT(GPIOB_SPI2_CS	)       |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_SCK	)       |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MISO)          |\
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_OTYPER            ( PIN_OTYPE_PUSHPULL(GPIOB_ADXL_DRDY)       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_ADXL_INT2)       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_ADXL_INT1)    |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_3        )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_4        )      |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_5        )       |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCK )       |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_P8        )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_P9           )       |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_P10      )       |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_P11      )       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI2_CS	)       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI2_SCK	)       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MISO)       |\
                                      PIN_OTYPE_PUSHPULL(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_ADXL_DRDY)          |\
                                     PIN_OSPEED_HIGH(GPIOB_ADXL_INT2)          |\
                                     PIN_OSPEED_HIGH(GPIOB_ADXL_INT1)       |\
                                     PIN_OSPEED_HIGH(GPIOB_3        )          |\
                                     PIN_OSPEED_HIGH(GPIOB_4        )         |\
                                     PIN_OSPEED_HIGH(GPIOB_5        )          |\
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SCK )          |\
                                     PIN_OSPEED_HIGH(GPIOB_I2C1_SDA )          |\
                                     PIN_OSPEED_HIGH(GPIOB_P8        )          |\
                                     PIN_OSPEED_HIGH(GPIOB_P9           )          |\
                                     PIN_OSPEED_HIGH(GPIOB_P10      )          |\
                                     PIN_OSPEED_HIGH(GPIOB_P11      )          |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_CS	)          |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_SCK	)          |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_MISO)          |\
                                     PIN_OSPEED_HIGH(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_ADXL_DRDY)       |\
                                     PIN_PUPDR_PULLUP(GPIOB_ADXL_INT2)       |\
                                     PIN_PUPDR_PULLUP(GPIOB_ADXL_INT1)    |\
                                     PIN_PUPDR_PULLUP(GPIOB_3        )       |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_4        )      |\
                                     PIN_PUPDR_PULLUP(GPIOB_5        )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SCK )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SDA )       |\
                                       PIN_PUPDR_PULLUP(GPIOB_P8         )       |\
                                       PIN_PUPDR_PULLUP(GPIOB_P9           )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_P10      )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_P11      )       |\
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_CS	)       |\
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_SCK	)       |\
                                     PIN_PUPDR_PULLDOWN(GPIOB_SPI2_MISO)       |\
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_ADXL_DRDY)             |\
                                     PIN_ODR_HIGH(GPIOB_ADXL_INT2)             |\
                                     PIN_ODR_HIGH(GPIOB_ADXL_INT1)          |\
                                     PIN_ODR_HIGH(GPIOB_3        )             |\
                                     PIN_ODR_HIGH(GPIOB_4        )            |\
                                     PIN_ODR_HIGH(GPIOB_5        )             |\
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCK )             |\
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA )             |\
                                     PIN_ODR_HIGH(GPIOB_P8        )             |\
                                     PIN_ODR_HIGH(GPIOB_P9           )             |\
                                     PIN_ODR_HIGH(GPIOB_P10      )             |\
                                     PIN_ODR_HIGH(GPIOB_P11      )             |\
                                     PIN_ODR_HIGH(GPIOB_SPI2_CS	)             |\
                                     PIN_ODR_HIGH(GPIOB_SPI2_SCK	)             |\
                                     PIN_ODR_HIGH(GPIOB_SPI2_MISO)             |\
                                     PIN_ODR_HIGH(GPIOB_SPI2_MOSI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_ADXL_DRDY,0)           |\
                                     PIN_AFIO_AF(GPIOB_ADXL_INT2,0)           |\
                                     PIN_AFIO_AF(GPIOB_ADXL_INT1,0)        |\
                                     PIN_AFIO_AF(GPIOB_3        ,0)           |\
                                     PIN_AFIO_AF(GPIOB_4        ,0)          |\
                                     PIN_AFIO_AF(GPIOB_5        ,0)           |\
                                     PIN_AFIO_AF(GPIOB_I2C1_SCK ,4 )            |\
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA ,4 ))           
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_P8       ,0)           |\
                                     PIN_AFIO_AF(GPIOB_P9       ,0)           |\
                                     PIN_AFIO_AF(GPIOB_P10      ,0)           |\
                                     PIN_AFIO_AF(GPIOB_P11      ,0)           |\
                                     PIN_AFIO_AF(GPIOB_SPI2_CS	,0 )           |\
                                     PIN_AFIO_AF(GPIOB_SPI2_SCK	,5)           |\
                                     PIN_AFIO_AF(GPIOB_SPI2_MISO,5)            |\
                                     PIN_AFIO_AF(GPIOB_SPI2_MOSI,5))

/*
 * GPIOC setup:
 *
 */
 
//#define VAL_GPIOC_MODER             ( PIN_MODE_ANALOG(GPIOC_AD0   )        |\
//                                     PIN_MODE_ANALOG(GPIOC_AD1  )        |\
//                                     PIN_MODE_ANALOG(GPIOC_AD2  )        |\
//                                      PIN_MODE_OUTPUT(GPIOC_LOG_LED)        |\
//                                      PIN_MODE_OUTPUT(GPIOC_4)        |\
//                                      PIN_MODE_INPUT(GPIOC_5        )        |\
//                                      PIN_MODE_OUTPUT(GPIOC_FS_CS1        )        |\
//                                      PIN_MODE_OUTPUT(GPIOC_FS_CS2        )        |\
//                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_D0  )        |\
//                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_D1  )        |\
//                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_D2  )        |\
//                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_D3  )        |\
//                                  PIN_MODE_ALTERNATE(GPIOC_SDIO_CLK )        |\
//                                      PIN_MODE_INPUT(GPIOC_USER_BTN  )        |\
//                                      PIN_MODE_INPUT(GPIOC_OSC32_IN )        |\
//                                      PIN_MODE_INPUT(GPIOC_OSC32_OUT))
//#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_AD0   )        |\
//                                     PIN_OTYPE_PUSHPULL(GPIOC_AD1  )        |\
//                                     PIN_OTYPE_PUSHPULL(GPIOC_AD2  )        |\
//                                     PIN_OTYPE_OPENDRAIN(GPIOC_LOG_LED)        |\
//                                     PIN_OTYPE_OPENDRAIN(GPIOC_4)        |\
//                                     PIN_OTYPE_PUSHPULL(GPIOC_5        )        |\
//                                     PIN_OTYPE_PUSHPULL(GPIOC_FS_CS1        )        |\
//                                     PIN_OTYPE_PUSHPULL(GPIOC_FS_CS2        )        |\
//                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D0  )        |\
//                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D1  )        |\
//                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D2  )        |\
//                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D3  )        |\
//                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_CLK )        |\
//                                     PIN_OTYPE_PUSHPULL(GPIOC_USER_BTN  )        |\
//                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN )        |\
//                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
//#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_AD0   )           |\
//                                     PIN_OSPEED_HIGH(GPIOC_AD1  )           |\
//                                     PIN_OSPEED_HIGH(GPIOC_AD2  )           |\
//                                     PIN_OSPEED_HIGH(GPIOC_LOG_LED)           |\
//                                     PIN_OSPEED_HIGH(GPIOC_4)           |\
//                                     PIN_OSPEED_HIGH(GPIOC_5        )           |\
//                                     PIN_OSPEED_HIGH(GPIOC_FS_CS1        )           |\
//                                     PIN_OSPEED_HIGH(GPIOC_FS_CS2        )           |\
//                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D0  )           |\
//                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D1  )           |\
//                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D2  )           |\
//                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D3  )           |\
//                                     PIN_OSPEED_HIGH(GPIOC_SDIO_CLK )           |\
//                                     PIN_OSPEED_HIGH(GPIOC_USER_BTN  )           |\
//                                     PIN_OSPEED_HIGH(GPIOC_OSC32_IN )           |\
//                                     PIN_OSPEED_HIGH(GPIOC_OSC32_OUT))
//#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_AD0   )        |\
//                                     PIN_PUPDR_PULLUP(GPIOC_AD1  )        |\
//                                     PIN_PUPDR_PULLUP(GPIOC_AD2  )        |\
//                                     PIN_PUPDR_FLOATING(GPIOC_LOG_LED )        |\
//                                     PIN_PUPDR_FLOATING(GPIOC_4 )        |\
//                                       PIN_PUPDR_PULLUP(GPIOC_5        )        |\
//                                     PIN_PUPDR_PULLUP(GPIOC_FS_CS1        )        |\
//                                     PIN_PUPDR_PULLUP(GPIOC_FS_CS2        )        |\
//                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D0  )        |\
//                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D1  )        |\
//                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D2  )        |\
//                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D3  )        |\
//                                     PIN_PUPDR_PULLDOWN(GPIOC_SDIO_CLK )        |\
//                                     PIN_PUPDR_PULLUP(GPIOC_USER_BTN  )        |\
//                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN )        |\
//                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
//#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_AD0   )              |\
//                                     PIN_ODR_HIGH(GPIOC_AD1  )              |\
//                                     PIN_ODR_HIGH(GPIOC_AD2  )              |\
//                                     PIN_ODR_HIGH(GPIOC_LOG_LED)              |\
//                                     PIN_ODR_HIGH(GPIOC_4)              |\
//                                     PIN_ODR_HIGH(GPIOC_5        )              |\
//                                     PIN_ODR_HIGH(GPIOC_FS_CS1        )              |\
//                                     PIN_ODR_HIGH(GPIOC_FS_CS2        )              |\
//                                     PIN_ODR_HIGH(GPIOC_SDIO_D0  )              |\
//                                     PIN_ODR_HIGH(GPIOC_SDIO_D1  )              |\
//                                     PIN_ODR_HIGH(GPIOC_SDIO_D2  )              |\
//                                     PIN_ODR_HIGH(GPIOC_SDIO_D3  )              |\
//                                     PIN_ODR_HIGH(GPIOC_SDIO_CLK )              |\
//                                     PIN_ODR_HIGH(GPIOC_USER_BTN  )              |\
//                                     PIN_ODR_HIGH(GPIOC_OSC32_IN )              |\
//                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
//#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_AD0   ,0   )               |\
//                                     PIN_AFIO_AF(GPIOC_AD1  ,0   )               |\
//                                     PIN_AFIO_AF(GPIOC_AD2  ,0  )               |\
//                                     PIN_AFIO_AF(GPIOC_LOG_LED,0)               |\
//                                     PIN_AFIO_AF(GPIOC_4,0  )               |\
//                                     PIN_AFIO_AF(GPIOC_5        ,0  )               |\
//                                     PIN_AFIO_AF(GPIOC_FS_CS1        ,0      )               |\
//                                     PIN_AFIO_AF(GPIOC_FS_CS2        ,0      ))              
//#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_SDIO_D0  ,12      )               |\
//                                     PIN_AFIO_AF(GPIOC_SDIO_D1  ,12     )               |\
//                                     PIN_AFIO_AF(GPIOC_SDIO_D2  ,12  )               |\
//                                     PIN_AFIO_AF(GPIOC_SDIO_D3  ,12 )               |\
//                                     PIN_AFIO_AF(GPIOC_SDIO_CLK ,12 )               |\
//                                     PIN_AFIO_AF(GPIOC_USER_BTN  ,0     )               |\
//                                     PIN_AFIO_AF(GPIOC_OSC32_IN ,0  )               |\
//                                     PIN_AFIO_AF(GPIOC_OSC32_OUT,0 ))


#define VAL_GPIOC_MODER         0
#define VAL_GPIOC_OTYPER        0
#define VAL_GPIOC_OSPEEDR       0
#define VAL_GPIOC_PUPDR         0
#define VAL_GPIOC_ODR           0
#define VAL_GPIOC_AFRL          0
#define VAL_GPIOC_AFRH          0

#define VAL_GPIOD_MODER         0
#define VAL_GPIOD_OTYPER        0
#define VAL_GPIOD_OSPEEDR       0
#define VAL_GPIOD_PUPDR         0
#define VAL_GPIOD_ODR           0
#define VAL_GPIOD_AFRL          0
#define VAL_GPIOD_AFRH          0

#define VAL_GPIOE_MODER         0
#define VAL_GPIOE_OTYPER        0
#define VAL_GPIOE_OSPEEDR       0
#define VAL_GPIOE_PUPDR         0
#define VAL_GPIOE_ODR           0
#define VAL_GPIOE_AFRL          0
#define VAL_GPIOE_AFRH          0

#define VAL_GPIOG_MODER         0
#define VAL_GPIOG_OTYPER        0
#define VAL_GPIOG_OSPEEDR       0
#define VAL_GPIOG_PUPDR         0
#define VAL_GPIOG_ODR           0
#define VAL_GPIOG_AFRL          0
#define VAL_GPIOG_AFRH          0

#define VAL_GPIOH_MODER         0
#define VAL_GPIOH_OTYPER        0
#define VAL_GPIOH_OSPEEDR       0
#define VAL_GPIOH_PUPDR         0
#define VAL_GPIOH_ODR           0
#define VAL_GPIOH_AFRL          0
#define VAL_GPIOH_AFRH          0

#define VAL_GPIOI_MODER         0
#define VAL_GPIOI_OTYPER        0
#define VAL_GPIOI_OSPEEDR       0
#define VAL_GPIOI_PUPDR         0
#define VAL_GPIOI_ODR           0
#define VAL_GPIOI_AFRL          0
#define VAL_GPIOI_AFRH          0

#define SPI1_CS_LOW()   palClearPad(GPIOA,GPIOA_SPI1_CS)
#define SPI1_CS_HIGH()   palSetPad(GPIOA,GPIOA_SPI1_CS)
#define SPI2_CS_LOW()   palClearPad(GPIOB,GPIOB_SPI2_CS)
#define SPI2_CS_HIGH()   palSetPad(GPIOB,GPIOB_SPI2_CS)
#define SPI3_CS_LOW()   palClearPad(GPIOA,GPIOA_SPI3_CS)
#define SPI3_CS_HIGH()   palSetPad(GPIOA,GPIOA_SPI3_CS)

#define LED_ON()        palClearPad(GPIOC,3)
#define LED_OFF()       palSetPad(GPIOC,3)
#define LED_TOG()       palTogglePad(GPIOC,3)

//#define LED_ON()        
//#define LED_OFF()       
//#define LED_TOG()       

//#define DBG_LED_ON()        palClearPad(GPIOB,2)
//#define DBG_LED_OFF()       palSetPad(GPIOB,2)
//#define DBG_LED_TOG()       palTogglePad(GPIOB,2)

#define DBG_LED_ON()        
#define DBG_LED_OFF()       
#define DBG_LED_TOG()       

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
