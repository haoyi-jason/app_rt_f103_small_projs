/*
 * port_hal.c
 *
 *  Created on: 2020¦~6¤ë30¤é
 *      Author: Jason
 */


#include <port_20004.h>
#include "ch.h"
#include "hal.h"

const ioline_t lora_do_map[] = {
  PAL_LINE(GPIOB,1),
  PAL_LINE(GPIOB,3),
  PAL_LINE(GPIOB,0),
  PAL_LINE(GPIOC,14),
  PAL_LINE(GPIOA,3),
  PAL_LINE(GPIOC,13),
};

const ioline_t lora_di_map[] = {
  PAL_LINE(GPIOB,4),
  PAL_LINE(GPIOB,9),
  PAL_LINE(GPIOB,8),
};

void port_set_digit_out(uint8_t id, uint8_t state)
{
  switch(id){
  case RADIO_ANT_SWITCH_POWER:
    if(state)
      palClearLine(lora_do_map[id]);
    else
      palSetLine(lora_do_map[id]);
    break;
  case RADIO_RESET:
    if(state)
      palSetLine(lora_do_map[id]);
    else
      palClearLine(lora_do_map[id]);
    break;
  case RADIO_ANT_SWITCH_LF:
    if(state)
      palSetLine(lora_do_map[id]);
    else
      palClearLine(lora_do_map[id]);
    break;
  case RADIO_ANT_SWITCH_HF:
    if(state)
      palSetLine(lora_do_map[id]);
    else
      palClearLine(lora_do_map[id]);
    break;
  default:
    if(state)
      palSetLine(lora_do_map[id]);
    else
      palClearLine(lora_do_map[id]);
    break;
  }
}

uint8_t port_get_digital_in(uint8_t id)
{
  uint8_t ret = 0;
  switch(id){
  case 0:
   ret = palReadLine(lora_di_map[id]) == PAL_LOW?0:1;
   break;
  default:
   ret = palReadLine(lora_di_map[id]) == PAL_LOW?1:0;
   break;
  }    
  return ret;
}

