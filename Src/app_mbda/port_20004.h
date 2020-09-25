/*
 * port_hal.h
 *
 *  Created on: 2020¦~6¤ë30¤é
 *      Author: Jason
 */

#ifndef SOURCE_PORT_20004_H_
#define SOURCE_PORT_20004_H_

#include "hal.h"

enum LORA_DO_CTRL{
  RADIO_ANT_SWITCH_POWER,
  RADIO_RESET,
  RADIO_ANT_SWITCH_LF,
  RADIO_ANT_SWITCH_HF,
  IDO_0,
  IDO_1
};

enum LORA_DI_STATE{
  BUSY,
  IDI_0,
  IDI_1
};


uint8_t port_get_digital_in(uint8_t id);
void port_set_digit_out(uint8_t id, uint8_t state);


#endif /* SOURCE_PORT_20004_H_ */
