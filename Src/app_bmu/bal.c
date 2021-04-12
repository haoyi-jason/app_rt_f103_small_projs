#include "ch.h"
#include "hal.h"
#include "bal.h"
#include "database_cfg.h"
#include "nvm.h"

#define BAL_THRESHOLD_MV 3000
#define BAL_HYSTERESIS_MV 200

#define SAVE_LASTSTATE(x)       x.laststate = x.state; \
                                x.lastsubstate = x.substate

static thread_t *thisThd;
static thread_reference_t thd_trp = NULL;
static DATA_BLOCK_MINMAX_s bal_minmax;
static DATA_BLOCK_BALANCING_CONTROL_s bal_balancing;
static DATA_BLOCK_CELLVOLTAGE_s bal_cellvoltage;
DATA_BLOCK_STATEREQUEST_s bal_request;

static BAL_STATE_s bal_state = {
    .timer                    = 0,
    .statereq                 = BAL_STATE_NO_REQUEST,
    .state                    = BAL_STATEMACH_UNINITIALIZED,
    .substate                 = BAL_ENTRY,
    .laststate                = BAL_STATEMACH_UNINITIALIZED,
    .lastsubstate             = 0,
    .triggerentry             = 0,
    .ErrRequestCounter        = 0,
    .initFinished             = E_NOT_OK,
    .active                   = FALSE,
    .balancing_threshold      = BAL_THRESHOLD_MV + BAL_HYSTERESIS_MV,
    .balancing_allowed        = TRUE,
    .balancing_global_allowed = FALSE,
};

static bal_state_s balState;

static void Deactive()
{
  chSysLock();
  
  for(uint8_t i=0;i<BS_NR_OF_BAT_CELLS_PER_MODULE;i++){
    bal_balancing.balancing_state[i] = 0;
    bal_balancing.delta_charge[i] = 0;
  }
  bal_balancing.enable_balancing = 0;
//  bal_balancing.timestamp = chVTGetSystemtime();
  
  chSysUnlock();
  
}

static msg_t bal_active_balancing_voltage2()
{
  uint16_t cellVolt[BS_NR_OF_BAT_CELLS_PER_MODULE];
  bmu_ltc_read_cellVolt(0,0xff,(uint8_t*)cellVolt);
  
  uint16_t min = bal_minmax.voltage_min;
  for(uint8_t i=0;i<BS_NR_OF_BAT_CELLS_PER_MODULE;i++){
    if(cellVolt[i] > min + bal_state.balancing_threshold){
      bal_balancing.balancing_state[i] = 1;
      bal_state.balancing_threshold = BAL_THRESHOLD_MV;
      bal_state.active = TRUE;
      bal_balancing.enable_balancing = 1;
    }
    else{
      bal_balancing.balancing_state[i] = 0;
    }
  }
  
  
  
  return MSG_OK;
}

static _cell_bal_state_s balCells[BS_NR_OF_BAT_CELLS_PER_MODULE];

static msg_t bal_active_balancing_voltage()
{
  uint16_t minVolt;
  uint8_t minId;
  // find minVolt from cells that greater than balVolt;
  minVolt = 5000;
  minId = 0;
  _nvm_board_s board;
  _nvm_balance_cfg_s bal_config;
  uint16_t volts[BS_NR_OF_BAT_CELLS_PER_MODULE];
  uint16_t openWire;
  uint16_t cellMask;
  uint8_t nofCells;
  
  //nvm_get_bal_config(&bal_config);
  nvm_get_block(PG_BOARD,(uint8_t*)&board);
  nvm_get_block(PG_CMU_BC,(uint8_t*)&bal_config);
  
//  nvm_get_bal_config(&bal_config);
//  balState.enableBalancing = bal_config.enableBalance;
//  balState.balancingVolt = bal_config.balanceVoltMv;
//  balState.balancingHystersis = bal_config.balanceHystersisMv;
//  balState.OnTimeSecond = bal_config.onTime;
//  balState.OffTimeSecond = bal_config.offTime;
    // read voltage from bmu
//    bmu_lte_read_cellVolt(0,12,(uint8_t*)balState.volt);
    nvm_runtime_get_cellVoltage(volts,&nofCells);
  nvm_runtime_get_openWireQueued(&openWire);
  nvm_get_cellQueueEx(&cellMask);
  openWire &= cellMask;
  for(uint8_t i = 0; i<BS_NR_OF_BAT_CELLS_PER_MODULE;i++){
    if((volts[i] > bal_config.balanceVoltMv) && (volts[i] < minVolt)){
      minVolt = volts[i];
      minId = i;
    }
  }
  uint8_t balEnable[BS_NR_OF_BAT_CELLS_PER_MODULE];
  minVolt = bal_config.balanceVoltMv;
  minVolt += bal_config.balanceHystersisMv; // offset 8 mv
  if(bal_config.enableBalance){
    for(uint8_t i=0;i<BS_NR_OF_BAT_CELLS_PER_MODULE;i++){
      balEnable[i] = 0;
      if(balCells[i].timer > 0){
        balCells[i].timer--;
        balEnable[i] = balCells[i].dir;
      }
      else{ // force each switch at timer == 0
        // check if balancing
        if(balCells[i].balancing == 1){
          // force idle 
          balCells[i].balancing = 0;
          balCells[i].dir = 0; // cooldown
          balCells[i].timer = bal_config.offTime;
          balEnable[i] = 0;
        }
        else{ // not balancing
          if(volts[i] > minVolt){
            balCells[i].balancing = 1;
            balCells[i].dir = 1;
            balCells[i].timer = bal_config.onTime;
            balEnable[i] = 1;
          }
        }
      }
      if(board.cellQueue[i] == 0){
        balEnable[i] = 0;
      }
      if(openWire != 0){
        balEnable[i] = 0;
        balCells[i].balancing = 0;
        balCells[i].dir = 0;
        balCells[i].timer = 0;
      }
    }
  }
    
  // update balancing state
  nvm_runtime_set_balancing(balEnable);
//  bmu_ltc_set_balance(balEnable);
  return MSG_OK;
}
static THD_WORKING_AREA(waBalance,512);
static THD_FUNCTION(procBalance,p){
  (void)p;
  chThdResume(&thd_trp, MSG_OK);
  while(1){
    bal_active_balancing_voltage();
    // update balancing state
    
    chThdSleepSeconds(1); // balancing task runs every seconds
  }
  
}

msg_t balInit(void)
{
  bal_balancing.enable_balancing = false;
  thisThd = chThdCreateStatic(waBalance, sizeof(waBalance), NORMALPRIO, procBalance, NULL);
  chSysLock();
  msg_t ret = chThdSuspendS(&thd_trp);
  chSysUnlock();
  return ret;  
}