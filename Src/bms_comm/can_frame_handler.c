#include "ch.h"
#include "hal.h"
#include "can_frame_handler.h"


#if !defined(NOF_CANFRAME_HANDLER) || (NOF_CANFRAME_HANDLER==0)
#error NOF_HANDLER is not defined or Zer0
#else
extern struct can_frame_handler PacketHandler[NOF_CANFRAME_HANDLER];
#endif

int8_t findHandlerByID(int32_t id, CANRxFrame *prx, CANTxFrame *ptx)
{
//  int8_t result = -1;
//  for(uint8_t i=0;i<NOF_CANFRAME_HANDLER;i++){
//    if(PacketHandler[i].eid == id){
//      result = PacketHandler[i].handler(prx,ptx);
//      break;
//    }
//  }
//  return result;
  int8_t result = -1;
  can_frame_handler handler;
  bool loop = true;
  uint8_t index = 0;
  while(loop){
    handler = PacketHandler[index++];
    if(handler.eid != -1 && handler.eid == id){
      result = handler.handler(prx,ptx);
    }
    if(handler.eid == -1){
      loop = false;
    }
  }
  return result;
}

int8_t findHandler(int32_t id, CANRxFrame *prx, CANTxFrame *ptx, can_frame_handler *handlers)
{
  int8_t result = -1;
  can_frame_handler handler;
  bool loop = true;
  uint8_t index = 0;
  while(loop){
    handler = handlers[index++];
    if(handler.eid != -1 && handler.eid == id){
      result = handler.handler(prx,ptx);
    }
    if(handler.eid == -1){
      loop = false;
    }
  }
  return result;
}


