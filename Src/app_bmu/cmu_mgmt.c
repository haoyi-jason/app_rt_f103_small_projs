#include "ch.h"
#include "hal.h"
#include <string.h>
#include "cmu_mgmt.h"
#include "nvm.h"
#include "bal.h"
#include "bmu_ltc.h"
#include "ntc.h"
#include "bmu_tle.h"
#include "can_frame_handler.h"

#define REPORT_INTERVAL_NORMAL  1000 // ms
#define REPORT_INTERVAL_FAST    100 // ms

int8_t config_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t id_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t report_config_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t balancing_config_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t ntc_config_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t cell_queue_config_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t cell_balance_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t report_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t heartBeatHandler(CANRxFrame *prx,CANTxFrame *ptx);


struct can_frame_handler PacketHandler[] = {
  {0x01,config_handler},
  {0x20,heartBeatHandler},
  {0x80,cell_balance_handler},
  {0x81,report_handler},
  {0x99,id_handler},
  {0x131,report_config_handler},
  {0x132,balancing_config_handler},
  {0x133,ntc_config_handler},
  {0x134,cell_queue_config_handler},  
//  {0x180,power_output_handler},  
};


static thread_t *thisThd;
static _nvm_board_s nvmBoardc;

static struct{
  uint8_t active;
  uint8_t emgReport;
  uint16_t report_interval_ms;
  uint16_t balancing_voltage;
  uint8_t enableBalancing;
  uint8_t balancing_band;
  uint8_t heartBeatLost;
}commState;

static CANConfig canCfg500K = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_BRP(3) | CAN_BTR_TS1(14) | CAN_BTR_TS2(1) | CAN_BTR_SJW(0)
};

// can config for 250K baud
static CANConfig canCfg250K = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_BRP(7) | CAN_BTR_TS1(14) | CAN_BTR_TS2(1) | CAN_BTR_SJW(0)
};

static void sendHeartBeat(void)
{
  CANTxFrame ptx;
  
  ptx.EID = (runTime.id<<12);
  ptx.DLC = 0;
  canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
}

static void sendCellVoltage(void)
{
  CANTxFrame ptx;
  ptx.DLC = 8;
  for(uint8_t i=0;i<3;i++){
    ptx.EID = (runTime.id<<12) + CAN_TX_SYS_STATE_0+i;
    memcpy(&ptx.data8[0],(uint8_t*)&runTime.cells.voltages[4*i],8);
    canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
  }
}

static void sendCellVoltageById(uint8_t id)
{
  CANTxFrame ptx;
  ptx.DLC = 8;
  for(uint8_t i=0;i<3;i++){
    ptx.EID = (runTime.id<<12) + CAN_TX_SYS_STATE_1;
    memcpy(&ptx.data8[0],(uint8_t*)&runTime.cells.voltages[id],2);
    canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
  }
}

static void sendAUXTemperature(void)
{
  CANTxFrame ptx;
  ptx.DLC = 8;
  ptx.EID = (runTime.id<<12) + CAN_TX_SYS_STATE_0+3;
  memcpy(&ptx.data8[0],(uint8_t*)&runTime.ntcTemp.gpioVolt[0],8);
  canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));

  ptx.EID = (runTime.id<<12) + CAN_TX_SYS_STATE_0+4;
  memcpy(&ptx.data8[0],(uint8_t*)&runTime.ntcTemp.gpioVolt[4],4);
  memcpy(&ptx.data8[4],(uint8_t*)&runTime.cells.balMask[0],2);
  canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
}

static void sendBalancingInfo(void)
{
  CANTxFrame ptx;
  ptx.DLC = 8;
  ptx.EID = (runTime.id<<12) + CAN_TX_SYS_STATE_0+6;
  memcpy(&ptx.data8[0],(uint8_t*)&runTime.cells.balMask[0],2);
  canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
}

static msg_t messageSend(CANDriver *p, uint32_t msgId, uint8_t *msg)
{
  CANTxFrame ptx;
  
  ptx.EID = msgId;
  ptx.IDE = CAN_IDE_EXT;
  ptx.RTR = CAN_RTR_DATA;
  ptx.DLC = 8;
  memcpy(ptx.data8,msg,8);
  
  return canTransmit(p,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
  
}

static virtual_timer_t vtReport;
static uint16_t reportMS;
static void report_cb(void *arg)
{
  chSysLockFromISR();
  if(commState.active){
    chEvtSignalI(thisThd, EVENT_MASK(1));
  }
  chVTSetI(&vtReport,TIME_MS2I(commState.report_interval_ms),report_cb,NULL);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waCANRX,4096);
static THD_FUNCTION(procCANRx,p){
 // thread_t *parent = (thread_t)p;
  CANDriver *ip = &CAND1;
  CANRxFrame rxMsg;
  event_listener_t el;
  CANTxFrame ptx;
  nvm_get_block(PG_BOARD,(uint8_t*)&nvmBoardc);
  
  ptx.data64[0] = 0x1;
  ptx.IDE = CAN_IDE_EXT;
  ptx.RTR = CAN_RTR_DATA;
  ptx.DLC = 8;
  reportMS = 1000;
  chEvtRegister(&ip->rxfull_event,&el,0);
  chVTObjectInit(&vtReport);
  uint8_t reportID = 0;
  uint16_t volt[12];
  bool bRun = true;
  _nvm_balance_cfg_s bal_config;
  nvmBoardc.autoStart = 1;
  uint32_t eidActive;
  uint8_t dest;
  if(nvmBoardc.autoStart == 1){
//    bmu_tle_init();
    bmu_ltc_init();
    balInit();
  }
  
  commState.active = 1;
  commState.report_interval_ms = 1000;
  commState.balancing_voltage = 5000;
  commState.enableBalancing = 0;
  chVTSet(&vtReport,TIME_MS2I(reportMS),report_cb,NULL);
  
  while(bRun){
    if(chThdShouldTerminateX()){
      bRun = false;
    }
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_MS2I(10));
    if(evt & 0x01){
      while(canReceive(ip,CAN_ANY_MAILBOX,&rxMsg,TIME_IMMEDIATE) == MSG_OK){
        if(rxMsg.EID == 0x0020){ // heartbeat
          heartBeatHandler(NULL,NULL);
          commState.active = 1;
          continue;
        }
        eidActive = rxMsg.EID & 0xFFF;
        dest = (rxMsg.EID>>12)&0xff;
        uint8_t group = (dest >> 5);
        //dest &= 0x1F; // mask ID only
        if((dest == nvmBoardc.id) || (dest == 0x00)){
          if(findHandlerByID(eidActive, &rxMsg,&ptx)>0){
            ptx.EID = (nvmBoardc.id << 12) | eidActive;
            canTransmit(ip,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
          }
        }
        
      }
    }
    if(evt & 0x02){
      commState.heartBeatLost++;
      if(commState.heartBeatLost < 10){
        uint8_t sz;
        nvm_runtime_get_cellVoltageQueued(volt,&sz);
        for(uint8_t i=0;i<3;i++){
          ptx.EID = CAN_TX_SYS_STATE_0+i;
          ptx.EID |= (nvmBoardc.id << 12);
          memcpy(&ptx.data8[0],(uint8_t*)&volt[4*i],8);
          canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
        }
        nvm_runtime_get_temperature((int16_t*)volt,&sz);
        ptx.EID = CAN_TX_SYS_STATE_0+3;
        ptx.EID |= (nvmBoardc.id << 12);
        memcpy(&ptx.data8[0],(uint8_t*)&volt[0],8);
        canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
        memset(ptx.data8,0xFF,8);
        ptx.EID = CAN_TX_SYS_STATE_0+4;
        ptx.EID |= (nvmBoardc.id << 12);
        memcpy(&ptx.data8[0],(uint8_t*)&volt[4],2);
        canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));

        memset(ptx.data8,0xFF,8);
        ptx.EID = CAN_TX_SYS_STATE_0+5;
        ptx.EID |= (nvmBoardc.id << 12);
        nvm_runtime_get_balancingQueued(&ptx.data8[0]);
        nvm_runtime_get_openWireQueued(&ptx.data16[1]);
        ptx.data16[1] >>= 1;
        canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
      }
      else{
        commState.active = 0;
      }
    }
    
//    if(chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10))){
//      continue;
//    }
//    
  }
}

static THD_WORKING_AREA(waMGMT,512);
static THD_FUNCTION(procMGMT,p){
  (void)p;
  chThdCreateStatic(waCANRX, sizeof(waCANRX), NORMALPRIO, procCANRx, thisThd);
  
}

void cmuMgmtInit(void)
{
  canStart(&CAND1,&canCfg250K);

//  thisThd = chThdCreateStatic(waMGMT, sizeof(waMGMT), NORMALPRIO, procMGMT, NULL);
  thisThd = chThdCreateStatic(waCANRX, sizeof(waCANRX), NORMALPRIO, procCANRx, NULL);
}

int8_t config_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(prx->RTR == CAN_RTR_DATA){
    switch(prx->data8[0]){
    case 0x99: // save data
      //nvmWrite();
      //nvmParamPowerUp();
      break;
    default:break;
    }
  }
  return 0;
}
int8_t id_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(prx->RTR == CAN_RTR_REMOTE){
    ptx->RTR = CAN_RTR_DATA;
    ptx->DLC = 8;
    ptx->IDE = CAN_IDE_EXT;
    ptx->data32[0] = BOARD_ID;
    ptx->data32[1] = FW_VERSION;
    return 1;
  }
  else{
    uint8_t b1 = prx->data8[0];
    uint8_t b2 = ~prx->data8[1];
    if(b1 == b2){
      uint8_t gid = prx->data8[2] >> 5;
      uint8_t id = prx->data8[2] & 0x1f;
      if((id >0) && (gid > 0)){
        nvmBoardc.id = prx->data8[2];
        nvm_set_block(PG_BOARD,(uint8_t*)&nvmBoardc);
      }
    }
    else{
      bool valid = true;
      for(uint8_t i=0;i<4;i++){
        if(prx->data8[i*2] != 0x55)
          valid = false;
        if(prx->data8[i*2+1] != 0xAA)
          valid = false;
      }
      if(valid){
        nvm_set_default(0);
      }
    }
  }
  return 0;
}

// set report speed
int8_t report_config_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(prx->RTR == CAN_RTR_REMOTE){
    
  }
  else{
    uint16_t tmp = prx->data16[0];
    if((tmp >= 100) && (tmp <= 10000)){
      chSysLock();
      reportMS = tmp;
      chSysUnlock();
    }
  }
  return 0;
}

// set balancing volt, hystersis time and difference 
int8_t balancing_config_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  _nvm_balance_cfg_s bal_config;
  nvm_get_bal_config(&bal_config);
  uint16_t v;
  if(prx->RTR == CAN_RTR_REMOTE){
//    CANTxFrame txMsg;
//    txMsg.EID = eidActive | (nvmBoardc.id << 12);
    ptx->RTR = CAN_RTR_DATA;
    ptx->DLC = 8;
    ptx->IDE = CAN_IDE_EXT;
    ptx->data16[0] = bal_config.balanceVoltMv;
    ptx->data8[2] = bal_config.balanceHystersisMv;
    ptx->data8[3] = bal_config.enableBalance;
    ptx->data16[2] = bal_config.onTime;
    ptx->data16[3] = bal_config.offTime;
    return 1;
//    canTransmit(&CAND1,CAN_ANY_MAILBOX,&txMsg,TIME_MS2I(100));
  }
  else{
    bool valid = false;
    if(prx->DLC == 2){
      bal_config.balanceVoltMv = prx->data16[0];
      valid = true;
    }
    else if(prx->DLC == 3){
      bal_config.balanceVoltMv = prx->data16[0];
      bal_config.balanceHystersisMv = (uint8_t)prx->data8[2];
      valid = true;
    }
    else if(prx->DLC == 4){
      bal_config.balanceVoltMv = prx->data16[0];
      bal_config.balanceHystersisMv = (uint8_t)prx->data8[2];
      bal_config.enableBalance = prx->data8[3] == 0x0?0x0:0x1;
      valid = true;
    }
    else if(prx->DLC == 8){
      bal_config.balanceVoltMv = prx->data16[0];
      bal_config.balanceHystersisMv = (uint8_t)prx->data8[2];
      bal_config.enableBalance = prx->data8[3] == 0x0?0x0:0x1;
      bal_config.onTime = prx->data16[2];
      bal_config.offTime = prx->data16[3];
      if((bal_config.onTime < 40) && (bal_config.offTime >= bal_config.onTime)){
        valid = true;
      }
    }
    
    if(valid){
      nvm_set_bal_config(&bal_config);
    }
  }
  return 0;
}

// set resistance beta and beta temp of NTC
int8_t ntc_config_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  _nvm_ntc_cfg_s ntc;
  nvm_get_block(PG_CMU_TC,(uint8_t*)&ntc);
  if(prx->RTR == CAN_RTR_REMOTE){
//    CANTxFrame txMsg;
//    txMsg.EID = eidActive | (nvmBoardc.id << 12);
    ptx->RTR = CAN_RTR_DATA;
    ptx->DLC = 8;
    ptx->IDE = CAN_IDE_EXT;
    
    ptx->data16[0] = ntc.resistance[0];
    ptx->data16[1] = ntc.beta[0];
    ptx->data8[4] = ntc.betaTemp[0];
    return 1;
    //canTransmit(&CAND1,CAN_ANY_MAILBOX,&txMsg,TIME_MS2I(100));
  }
  else{
//    if(palReadPad(GPIOA,12) == PAL_LOW){
      for(uint8_t i=0;i<NOF_MAX_AUXIO;i++){
        ntc.resistance[i] = prx->data16[0];
      }
      for(uint8_t i=0;i<NOF_MAX_AUXIO;i++){
        ntc.beta[i] = prx->data16[1];
      }
      for(uint8_t i=0;i<NOF_MAX_AUXIO;i++){
        ntc.betaTemp[i] = prx->data8[4];
      }
      nvm_set_block(PG_CMU_TC,(uint8_t*)&ntc);
//    }
  }
  return 0;
}

// set/get cell queue
int8_t cell_queue_config_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(prx->RTR == CAN_RTR_REMOTE){
    //ptx->EID = eidActive | (nvmBoardc.id << 12);
    ptx->RTR = CAN_RTR_DATA;
    ptx->DLC = 8;
    ptx->IDE = CAN_IDE_EXT;
    
    nvm_get_cellQueueEx(&ptx->data16[0]);
    return 1;
    //canTransmit(&CAND1,CAN_ANY_MAILBOX,&txMsg,TIME_MS2I(100));
  }
  else{
    //if(palReadPad(GPIOA,12) == PAL_LOW){
      nvm_set_cellQueueEx(prx->data8);
    //}
  }
  return 0;
}

// set/get balancing voltage
int8_t cell_balance_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(prx->RTR == CAN_RTR_REMOTE){
    ptx->RTR = CAN_RTR_DATA;
    ptx->DLC = 4;
    ptx->IDE = CAN_IDE_EXT;
    ptx->data8[0] = commState.enableBalancing;
    ptx->data8[1] = commState.balancing_band;
    ptx->data16[1] = commState.balancing_voltage;
    return 1;
  }
  else{
    if(prx->DLC >=4){
      commState.enableBalancing = prx->data8[0];
      commState.balancing_band = prx->data8[1];
      commState.balancing_voltage = prx->data16[1];
      
      balSetState(commState.enableBalancing);
      balSetBalancingVoltage(commState.balancing_voltage,commState.balancing_band);
    }
  }
  return 0;
}

// set/get report state
int8_t report_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(prx->RTR == CAN_RTR_REMOTE){
    //ptx->EID = eidActive | (nvmBoardc.id << 12);
    ptx->RTR = CAN_RTR_DATA;
    ptx->DLC = 4;
    ptx->IDE = CAN_IDE_EXT;
    ptx->data8[0] = commState.active;
    ptx->data8[1] = commState.emgReport;
    ptx->data16[1] = commState.report_interval_ms;
    return 1;
  }
  else{
    if(prx->DLC >= 4){
      commState.active = prx->data8[0];
      commState.report_interval_ms = prx->data16[1];
    }
  }
  return 0;
}

int8_t heartBeatHandler(CANRxFrame *prx,CANTxFrame *ptx)
{
  commState.heartBeatLost = 0;
  return 0; 
}

