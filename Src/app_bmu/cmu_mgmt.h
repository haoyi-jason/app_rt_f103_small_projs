#ifndef _CMU_MGMT_
#define _CMU_MGMT_

#include "hal.h"

typedef enum{
  CAN_RX_SW_RESET = 0x95,
  CAN_RX_DBG_MSG = 0x100,
  CAN_RX_STATE_REQ = 0x120,
  CAN_RX_GER_VER = 0x777,
}can_rx_e;

typedef enum{
  CAN_TX_BOOT = 0x101,
  CAN_TX_SYS_STATE_0 = 0x110,
  CAN_TX_SYS_STATE_1 = 0x120,
  CAN_TX_SYS_STATE_2 = 0x130,
  CAN_TX_CELL_VOLT_0 = 0x200,   
  CAN_TX_CELL_VOLT_1 = 0x201,   
  CAN_TX_CELL_VOLT_2 = 0x202,   
  CAN_TX_CELL_VOLT_3 = 0x203,
  CAN_TX_CELL_VOLT_SIMP_0 = 0x210, // 2.5V based voltage send, 8-bit/cell, 10mV resolution, cell 1~6
  CAN_TX_CELL_VOLT_SIMP_1 = 0x211,
  CAN_TX_PACK_TEMP = 0x220,
  CAN_TX_BALANCE_STATE = 0x221,
}can_tx_e;

#endif