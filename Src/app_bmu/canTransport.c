#include "hal.h"
#include "ch.h"
#include "canTransport.h"




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

static THD_WORKING_AREA(waCANRX,256);
static THD_FUNCTION(procCANRx,p){
  (void)p;
  CANDriver *ip = p;
  CANRxFrame rxMsg;
  event_listener_t el;
  
  chEvtRegister(&ip->rxfull_event,&el,1);
  bool bRun = true;
  while(bRun){
    if(chThdShouldTerminateX()){
      bRun = false;
    }
    
    if(chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10))){
      continue;
    }
    
    while(canReceive(ip,CAN_ANY_MAILBOX,&rxMsg,TIME_IMMEDIATE) == MSG_OK){
      
    }
  }
}