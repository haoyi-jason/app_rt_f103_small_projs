#include "ch.h"
#include "hal.h"
#include <string.h>
#include "cmu_mgmt.h"
#include "nvm.h"
#include "bal.h"
#include "bmu_ltc.h"
#include "ntc.h"

#define REPORT_INTERVAL_NORMAL  1000 // ms
#define REPORT_INTERVAL_FAST    100 // ms

static thread_t *thisThd;
static _nvm_board_s nvmBoard;

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
  chEvtSignalI(thisThd, EVENT_MASK(1));
  chVTSetI(&vtReport,TIME_MS2I(reportMS),report_cb,NULL);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waCANRX,1024);
static THD_FUNCTION(procCANRx,p){
 // thread_t *parent = (thread_t)p;
  CANDriver *ip = &CAND1;
  CANRxFrame rxMsg;
  event_listener_t el;
  CANTxFrame ptx;
  nvm_get_block(PG_BOARD,(uint8_t*)&nvmBoard);
  
  ptx.data64[0] = 0x1;
  ptx.IDE = CAN_IDE_EXT;
  ptx.RTR = CAN_RTR_DATA;
  ptx.DLC = 8;
  reportMS = 1000;
  chEvtRegister(&ip->rxfull_event,&el,0);
  chVTObjectInit(&vtReport);
  chVTSet(&vtReport,TIME_MS2I(reportMS),report_cb,NULL);
  uint8_t reportID = 0;
  uint16_t volt[12];
  bool bRun = true;
  _nvm_balance_cfg_s bal_config;
  nvmBoard.autoStart = 1;
  uint32_t eidActive;
  uint8_t target;
  if(nvmBoard.autoStart == 1){
    bmu_ltc_init();
    balInit();
  }
          
  while(bRun){
    if(chThdShouldTerminateX()){
      bRun = false;
    }
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_MS2I(10));
    if(evt & 0x01){
      while(canReceive(ip,CAN_ANY_MAILBOX,&rxMsg,TIME_IMMEDIATE) == MSG_OK){
        eidActive = rxMsg.EID & 0xFFF;
        target = (rxMsg.EID>>12)&0xff;
        if(eidActive == 0x131){ // set report speed
          uint16_t tmp = rxMsg.data16[0];
          if((tmp >= 100) && (tmp <= 10000)){
            chSysLock();
            reportMS = tmp;
            chSysUnlock();
          }
        }
        else if(eidActive == 0x132){ // set balancing volt, hystersis time and difference 
          nvm_get_bal_config(&bal_config);
          uint16_t v;
          if(rxMsg.RTR == CAN_RTR_REMOTE){
            CANTxFrame txMsg;
            txMsg.RTR = CAN_RTR_DATA;
            txMsg.EID = rxMsg.EID;
            txMsg.DLC = 8;
            txMsg.IDE = CAN_IDE_EXT;
            
            txMsg.data16[0] = bal_config.balanceVoltMv;
            txMsg.data8[2] = bal_config.balanceHystersisMv;
            txMsg.data8[3] = bal_config.enableBalance;
            txMsg.data16[2] = bal_config.onTime;
            txMsg.data16[3] = bal_config.offTime;
            canTransmit(&CAND1,CAN_ANY_MAILBOX,&txMsg,TIME_MS2I(100));
          }
          else{
            bal_config.balanceVoltMv = rxMsg.data16[0];
            bal_config.balanceHystersisMv = (uint8_t)rxMsg.data8[2];
            bal_config.enableBalance = rxMsg.data8[3] == 0x0?0x0:0x1;
            bal_config.onTime = rxMsg.data16[2];
            bal_config.offTime = rxMsg.data16[3];
            nvm_set_bal_config(&bal_config);
          }
        }
        else if(eidActive == 0x133){ // set resistance beta and beta temp of NTC
          _nvm_ntc_cfg_s ntc;
          nvm_get_block(PG_CMU_TC,(uint8_t*)&ntc);
          if(rxMsg.RTR == CAN_RTR_REMOTE){
            CANTxFrame txMsg;
            txMsg.RTR = CAN_RTR_DATA;
            txMsg.EID = rxMsg.EID;
            txMsg.DLC = 8;
            txMsg.IDE = CAN_IDE_EXT;
            
            txMsg.data16[0] = ntc.resistance[0];
            txMsg.data16[1] = ntc.beta[0];
            txMsg.data8[4] = ntc.betaTemp[0];
            canTransmit(&CAND1,CAN_ANY_MAILBOX,&txMsg,TIME_MS2I(100));
          }
          else{
            if(palReadPad(GPIOA,12) == PAL_LOW){
              for(uint8_t i=0;i<NOF_MAX_AUXIO;i++){
                ntc.resistance[i] = rxMsg.data16[0];
              }
              for(uint8_t i=0;i<NOF_MAX_AUXIO;i++){
                ntc.beta[i] = rxMsg.data16[1];
              }
              for(uint8_t i=0;i<NOF_MAX_AUXIO;i++){
                ntc.betaTemp[i] = rxMsg.data8[4];
              }
              nvm_set_block(PG_CMU_TC,(uint8_t*)&ntc);
            }
          }
        }
        else if(eidActive == 0x99){
          if(palReadPad(GPIOA,12) == PAL_LOW){
            nvmBoard.id = rxMsg.data8[0];
            nvm_set_block(PG_BOARD,(uint8_t*)&nvmBoard);
          }
        }
      }
    }
    if(evt & 0x02){
      uint8_t sz;
      nvm_runtime_get_cellVoltageQueued(volt,&sz);
      for(uint8_t i=0;i<3;i++){
        ptx.EID = CAN_TX_SYS_STATE_0+i;
        ptx.EID |= (nvmBoard.id << 12);
        memcpy(&ptx.data8[0],(uint8_t*)&volt[4*i],8);
        canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
      }
      nvm_runtime_get_temperature((int16_t*)volt,&sz);
      ptx.EID = CAN_TX_SYS_STATE_0+3;
      ptx.EID |= (nvmBoard.id << 12);
      memcpy(&ptx.data8[0],(uint8_t*)&volt[0],8);
      canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
      memset(ptx.data8,0xFF,8);
      ptx.EID = CAN_TX_SYS_STATE_0+4;
      ptx.EID |= (nvmBoard.id << 12);
      memcpy(&ptx.data8[0],(uint8_t*)&volt[4],2);
      canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));

      memset(ptx.data8,0xFF,8);
      ptx.EID = CAN_TX_SYS_STATE_0+5;
      ptx.EID |= (nvmBoard.id << 12);
      nvm_runtime_get_balancingQueued(&ptx.data8[0]);
      canTransmit(&CAND1,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
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