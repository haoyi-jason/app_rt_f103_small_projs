/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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

#include "hal.h"

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config =
{
  {VAL_GPIOAODR, VAL_GPIOACRL, VAL_GPIOACRH},
  {VAL_GPIOBODR, VAL_GPIOBCRL, VAL_GPIOBCRH},
  {VAL_GPIOCODR, VAL_GPIOCCRL, VAL_GPIOCCRH},
  {VAL_GPIODODR, VAL_GPIODCRL, VAL_GPIODCRH},
};
#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {

  stm32_clock_init();
}

/*
 * Board-specific initialization code.
 */
void boardInit(void) {
}

void board_relay_set(uint8_t x)
{
  switch(x){
  case 0:palSetLine(LINE_RELAY_OUT_0);
    break;
  case 1:palSetLine(LINE_RELAY_OUT_1);
    break;
  default:break;
  }
}
void board_relay_clr(uint8_t x)
{
  switch(x){
  case 0:palClearLine(LINE_RELAY_OUT_0);
    break;
  case 1:palClearLine(LINE_RELAY_OUT_1);
    break;
  default:break;
  }
}
void board_power_set(uint8_t x)
{
  switch(x){
  case 0:palSetLine(LINE_VSOURCE_0);
    break;
  case 1:palSetLine(LINE_VSOURCE_1);
    break;
  default:break;
  }
}
void board_power_clr(uint8_t x)
{
  switch(x){
  case 0:palClearLine(LINE_VSOURCE_0);
    break;
  case 1:palClearLine(LINE_VSOURCE_1);
    break;
  default:break;
  }
}
void baord_adc_reset()
{
  palClearLine(LINE_AD5593_RST);
  chThdSleepMilliseconds(5);
  palSetLine(LINE_AD5593_RST);
}

int8_t board_input_state(uint8_t x)
{
  switch(x){
  case 0:return palReadLine(LINE_ISO_DI_0)==PAL_HIGH?0:1;break;
  case 1:return palReadLine(LINE_ISO_DI_1)==PAL_HIGH?0:1;break;
  default: return 0;break;
  }
}

int8_t board_output_state(uint8_t x)
{
  switch(x){
  case 0:return palReadLine(LINE_RELAY_OUT_0)==PAL_HIGH?1:0;break;
  case 1:return palReadLine(LINE_RELAY_OUT_1)==PAL_HIGH?1:0;break;
  default: return 0;break;
  }
}

int8_t board_vout_state(uint8_t x)
{
  switch(x){
  case 0:return palReadLine(LINE_VSOURCE_0)==PAL_HIGH?1:0;break;
  case 1:return palReadLine(LINE_VSOURCE_1)==PAL_HIGH?1:0;break;
  default: return 0;break;
  }
}
