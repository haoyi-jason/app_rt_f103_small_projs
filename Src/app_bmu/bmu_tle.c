#include "ch.h"
#include "hal.h"
#include "bmu_tle.h"
#include "tle9012.h"
#include "tle9012_def.h"
#include "nvm.h"
#include "std_types.h"
#include "ntc.h"

#define TLE_SAVELASTSTATES()   bmuState.lastState = bmuState.state; \
                               bmuState.lastSubState = bmuState.subState

#define TLE_ResetErrorTable()
static thread_t *thisThd;
static thread_reference_t thd_trp = NULL;

struct packet_handler{
  uint8_t szToSend;
  uint8_t szSent;
  uint8_t txBuffer[16];
  uint8_t rxBuffer[16];
  uint8_t szReceived;
} tle_buffer;


static int8_t registerRead(TLE9012Driver *dev, uint8_t *b, uint8_t n, uint32_t timeout);
static int8_t registerWrite(TLE9012Driver *dev, uint8_t *b, uint8_t n, uint32_t timeout);
static int8_t registerXfer(TLE9012Driver *dev, uint8_t *tx, uint8_t txn,uint8_t *rx, uint8_t rxn);
static const SerialConfig sercfg = {
  2000000,
};

const TLE9012Config tleConfig = {
  &SD2,
  &sercfg
};

static TLE9012Driver tle9012 = {
  TLE9012_UNINIT,
  &tleConfig,
  0x00,
  registerXfer,
  
};


static TLE_STATE_s bmuState = {
  .timer = 0,
  .statereq = TLE_STATE_NO_REQUEST,
  .state = TLE_STATEMACH_UNINITIALIZED,
  .subState = 0,
  .lastState = TLE_STATEMACH_UNINITIALIZED,
  .lastSubState = 0,
};
static void txread(UARTDriver *uartp) ;
static void txempty(UARTDriver *uartp) ;
static void rxErr(UARTDriver *uartp, uartflags_t e);
static void rxChar(UARTDriver *uartp, uint16_t c);
static void rxEnd(UARTDriver *uartp);

static uint8_t rxcnt = 0;

static  UARTConfig cfg0 = {
    txread,
    txempty,
    rxEnd,
    rxChar,
    rxErr,
    NULL,
    2000000,
    USART_CR1_RXNEIE,// | USART_CR1_TCIE ,
    USART_CR2_LINEN,
    0
  };

static  UARTConfig cfg = {
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    2000000,
    USART_CR1_RXNEIE,// | USART_CR1_TCIE ,
    USART_CR2_LINEN,
    0
  };

void uart_init(){
  UARTDriver *u = &UARTD1;
 // u->usart->CR1 |= USART_CR1_RXNEIE;
//  u->usart->CR1 |= USART_CR1_TCIE;
  
  
  uartStart(&UARTD1,&cfg);
  
}


static TLE_RETURN_TYPE_e TLE_CheckStateRequest(TLE_STATE_REQUEST_e statereq)
{
  if(bmuState.statereq == TLE_STATE_NO_REQUEST){
    if(statereq == TLE_STATE_INIT_REQUEST){
      if(bmuState.state == TLE_STATEMACH_UNINITIALIZED){
        return TLE_OK;
      }
      else{
        return TLE_ALREADY_INITIALIZED;
      }
    }
  }else{
    return TLE_REQUEST_PENDING;
  }
}

TLE_RETURN_TYPE_e TLE_SetStateRequest(TLE_STATE_REQUEST_e statereq)
{
  TLE_RETURN_TYPE_e retVal = TLE_STATE_NO_REQUEST;
  
  retVal = TLE_CheckStateRequest(statereq);
  if(retVal == TLE_OK || retVal == TLE_BUSY_OK || retVal == TLE_OK_FROM_ERROR){
    bmuState.statereq = statereq;
  }
  
  return retVal;
}

TLE_RETURN_TYPE_e TLE_TransferStateRequest(uint8_t *busIDptr, TLE_ADCMODE_e *adcModeptr, TLE_ADCMEAS_CHAN_e *adcMeasChptr) {
    TLE_RETURN_TYPE_e retval = TLE_STATE_NO_REQUEST;

    chSysLock();
    retval = bmuState.statereq;
    *adcModeptr = bmuState.adcModereq;
    *adcMeasChptr = bmuState.adcMeasChreq;
    bmuState.statereq = TLE_STATE_NO_REQUEST;
    chSysUnlock();

    return (retval);
}

static void TLE_SaveRXtoVoltageBuffer(uint8_t registerSet, uint8_t *rxBuffer)
{
  
  
}

static void TLE_SaveRXtoGPIOBuffer(uint8_t registerSet, uint8_t *rxBuffer)
{
  
}

static void TLE_SaveBalancingFeedback(uint8_t *rxBuffer)
{
  
}

static void TLE_StateTransition(TLE_STATEMACH_e state, uint8_t substate, uint16_t timer_ms)
{
  bmuState.state = state;
  bmuState.subState = substate;
  bmuState.timer = timer_ms;
}


static uint16_t TLE_Get_MeasurementTCycle(TLE_ADCMODE_e adcMode)
{
  
  return 0;
}
/*
  R_NTC = (RAW_ADC * VREF * 4^INTC)/(1024 * 320)
*/
static int16_t temp_transfer(uint16_t raw)
{
  int32_t ret;
  uint16_t raw_adc = raw & 0x3ff; // 10-bit value
  uint8_t p = (raw >> 10) & 0x3;
  uint8_t f = 1;
  while(p--){
    f<<=2;
  }
  
  ret = raw_adc * 2 * f * 100000;
  ret >>= 10;
  ret >>= 5;
  
  return (int16_t)ntcCalculateTempD10_R(ret,10000,3435,25);
}

static uint16_t cv_transfer(uint16_t raw)
{
  uint32_t ret;
  ret = raw*5000;
  ret >>= 16;
  return (uint16_t)ret;
}

static uint16_t bv_transfer(uint16_t raw)
{
  uint32_t ret;
  ret = raw*60000;
  ret >>= 16;
  return (uint16_t)ret;
}

static uint16_t it_transfer(uint16_t raw)
{
  uint32_t ret;
  uint16_t raw_adc = raw & 0x3ff; // 10-bit value
  ret = 5473 - raw_adc*6.624;
  return (uint16_t)ret;
}

static virtual_timer_t vtMeas;
static uint32_t measCntr; 
static void meas_cb(void *arg)
{
  chSysLockFromISR();
  measCntr++;
  if(measCntr == 5){
    measCntr = 0;
    TLE_SetStateRequest(TLE_STATE_OPENWIRE_CHECK_REQUEST);
  }else{
    TLE_SetStateRequest(TLE_STATE_ALLGPIOMEASUREMENT_REQUEST);
  }
  chVTSetI(&vtMeas,TIME_MS2I(100),meas_cb,NULL);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waBMU, 1024);
static THD_FUNCTION(procBMU, arg) 
{
  (void)arg;
  uint16_t cell_cfg = 0x0fff;
  STD_RETURN_TYPE_e retVal = E_OK;
  TLE_STATE_REQUEST_e statereq = TLE_STATE_NO_REQUEST;
  uint8_t tmpbusID = 0;
  TLE_ADCMODE_e tmpadcMode = TLE_ADCMODE_UNDEFINED;
  TLE_ADCMEAS_CHAN_e tmpadcMeasCh = TLE_ADCMEAS_UNDEFINED;
  
  measCntr = 0;
  chVTObjectInit(&vtMeas);
  chVTSet(&vtMeas,TIME_MS2I(100),meas_cb,NULL);
  
  nvm_get_cellQueue(bmuState.cells.activeCellQue);
  
  cell_cfg = 0x0;
  for(uint8_t i=0;i<sizeof(bmuState.cells.activeCellQue);i++){
    cell_cfg |= (bmuState.cells.activeCellQue[i]==0)?0:(1 << i);
  }

  TLE_SetStateRequest(TLE_STATE_INIT_REQUEST);
  
  chThdResume(&thd_trp,MSG_OK);
  
  // initial config
//  tle9012.part_config = 0x0FFF; // 12-cell
  tle9012.part_config = cell_cfg; // 8-cell
//  tle9012.ol_ov_thr = (0x19<<10) | 0x3AE; // 
//  tle9012.ol_uv_thr = (0x01 << 10) | 0x134; // 
  tle9012.ol_ov_thr = (0x3F<<10) | 0x3AE; // 
  tle9012.ol_uv_thr = (0x00 << 10) | 0x134; // 
  tle9012.temp_conf = (0x5 << 12); // 5-ntc, default
  tle9012.int_ot_warn_conf = 0x0;
  tle9012.rr_err_cnt = 0x0002; // default
  tle9012.rr_config = 0x8024; // default
  tle9012.fault_mask = 0x3400; // default
  tle9012.multi_read_cfg = (1 << 9) | (5<<5) | (1 << 4) | (0x0c);
//  tle9012.op_mode = 0x2;  // ext_wd enable
  tle9012.op_mode = 0x0;  // ext_wd enable
  tle9012.bal_curr_thr = 0x00FF;
  
  
  uint8_t buffer[128];
  tle9012.buffer = buffer;
  
  uart_init();
  TLE9012Driver *dev = &tle9012;
  bmuState.errorCount = 0;
  bool bRun = true;
  while(bRun){
    tle9012_feedWDT(dev,0x7f);
    switch(bmuState.state){
    case TLE_STATEMACH_UNINITIALIZED:
      statereq = TLE_TransferStateRequest(&tmpbusID,&tmpadcMode,&tmpadcMeasCh);
      if(statereq == TLE_STATE_INIT_REQUEST){
        TLE_StateTransition(TLE_STATEMACH_INITIALIZATION, TLE_START_INIT_INITIALIZATION,TLE_STATEMACH_SHORTTIME);
        bmuState.adcMode = tmpadcMode;
        bmuState.adcMeasCh = tmpadcMeasCh;
      }
      break;
    case TLE_STATEMACH_INITIALIZATION:
      if(bmuState.subState == TLE_START_INIT_INITIALIZATION){
        tle9012_init(&tle9012);
        TLE9012_READ_ICVID(dev);
        TLE9012_READ_GEN_DIAG(dev);
        //if(tle9012.gen_diag & 0x80){
          //TLE9012_READ_DIAG_OL(dev);
          tle9012.gen_diag = 0x0000;
          TLE9012_WRITE_GEN_DIAG(dev);
          TLE9012_READ_GEN_DIAG(dev);
       // }
        TLE9012_WRITE_OL_OV_THR(dev);
        TLE9012_WRITE_OL_UV_THR(dev);
        TLE9012_READ_OL_OV_THR(dev);
        TLE9012_READ_OL_UV_THR(dev);
        TLE9012_WRITE_BAL_CURR_THR(dev);
        bmuState.lastSubState = bmuState.subState;
       // bmuState.subState = TLE_READ_INITIALIZATION_REGISTER;
        TLE_StateTransition(TLE_STATEMACH_INITIALIZATION, TLE_READ_INITIALIZATION_REGISTER,bmuState.commonDataTransferTime);
      }
      else if(bmuState.subState == TLE_READ_INITIALIZATION_REGISTER){
        TLE9012_READ_PART_CONFIG(dev);
        if(tle9012.part_config == cell_cfg){
          bmuState.subState = TLE_EXIT_INITIALIZATION;
        }
        else{
          bmuState.subState = TLE_STATEMACH_UNINITIALIZED;
        }
      }
      else if(bmuState.subState == TLE_EXIT_INITIALIZATION){
        TLE_SAVELASTSTATES();
        TLE_ResetErrorTable();
        TLE_StateTransition(TLE_STATEMACH_INITIALIZED, TLE_START_INIT_INITIALIZATION, TLE_STATEMACH_SHORTTIME);  
      }
      break;
    case TLE_STATEMACH_INITIALIZED:
      TLE_SAVELASTSTATES();
      TLE_StateTransition(TLE_STATEMACH_STARTVMEAS, TLE_ENTRY, TLE_STATEMACH_SHORTTIME);  
      break;
    case TLE_STATEMACH_STARTVMEAS:
      tle9012.meas_ctrl = MEAS_CTRL_START | MEAS_CTRL_BITWIDTH(MWAS_CTRL_BIT16)| MEAS_CTRL_START_BVM | MEAS_CTRL_BVMBITWIDTH(MWAS_CTRL_BIT16) | MAAS_CTRL_PBOFF;
      tle9012.meas_ctrl |= 0xf; // timer
      TLE9012_WRITE_MEAS_CTRL(dev);
      TLE_StateTransition(TLE_STATEMACH_READVOLTAGE, TLE_ENTRY, TLE_STATEMACH_SHORTTIME);  
      break;
    case TLE_STATEMACH_READVOLTAGE:
      if(bmuState.subState == 0){ // check if conversions all done
        TLE9012_READ_MEAS_CTRL(dev);
        if((dev->meas_ctrl & 0x8880) == 0x0){
          TLE_StateTransition(TLE_STATEMACH_READVOLTAGE, 1, TLE_STATEMACH_SHORTTIME);  
        }
      }
      else if(bmuState.subState == 1){ // read all voltage
        TLE9012_READ_MULTI_READ(dev);
        TLE_StateTransition(TLE_STATEMACH_READVOLTAGE, 2, TLE_STATEMACH_SHORTTIME);  
      }
      else if(bmuState.subState == 2){ // feed data, data was packed in 5-byte order, id,reg_addr,dh,dl,crc
        uint8_t *dptr = tle9012.buffer;
        dptr += 4; // 4-byte offset for read command
        uint8_t crc;
        for(uint8_t i=0;i<19;i++){
          crc = crc8_msb(CRC_POLY,dptr,4);
          if(crc == dptr[4]){
            int16_t val = (dptr[2]<<8) | dptr[3];
            uint8_t regAddr = dptr[1];
            if((regAddr >= 0x19) && (regAddr <= 0x24)){
              tle9012.cv[regAddr - 0x19] = cv_transfer(val);
            }
            else if((regAddr == 0x28)){
              tle9012.bv = bv_transfer(val);
            }
            else if((regAddr == 0x30)){
              tle9012.int_temp = it_transfer(val);
            }
            else if((regAddr >= 0x29) && (regAddr <= 0x2d)){
              tle9012.et[regAddr - 0x29] = temp_transfer(val);
            }
          }
          dptr += 5;
        }
        nvm_runtime_set_cellVoltage(tle9012.cv);
        nvm_runtime_set_temperature(tle9012.et);
        TLE_StateTransition(TLE_STATEMACH_BALANCECONTROL, 0, TLE_STATEMACH_SHORTTIME);  
      }
      break;
    case TLE_STATEMACH_BALANCECONTROL:
      if(bmuState.subState == 0){
        nvm_runtime_get_balancing(tle9012.balancingControl);
        uint16_t bal_tmp = 0;
        for(uint8_t i=0;i<12;i++){
//          tle9012.bal_settings <<= 1;
          bal_tmp |= (tle9012.balancingControl[i] << i);
        }
        tle9012.bal_settings.write = (bal_tmp & 0xFFF);
        //tle9012.bal_settings = 0x0;
        TLE9012_WRITE_BAL_SETTINGS(dev);
        TLE_StateTransition(TLE_STATEMACH_BALANCECONTROL, 1, TLE_STATEMACH_SHORTTIME);  
      }
      else if(bmuState.subState == 1){
        TLE9012_READ_BAL_SETTINGS(dev);
        TLE_StateTransition(TLE_STATEMACH_OPENWIRE_CHECK, 0, TLE_STATEMACH_SHORTTIME);  
      }
      break;
      
    case TLE_STATEMACH_ALLGPIOMEASUREMENT:
      break;
      
    case TLE_STATEMACH_READALLGPIO:
      break;
      
    case TLE_STATEMACH_TEMPSEN_READ:
      break;
      
    case TLE_STATEMACH_OPENWIRE_CHECK:
      switch(bmuState.subState){
      case 0:
        TLE9012_READ_DIAG_OL(dev);
        if(dev->diag_ol != 0){
          // read gen_diag
          TLE9012_READ_GEN_DIAG(dev);
          dev->gen_diag &= ~0x80; // clear open load error
          TLE9012_WRITE_GEN_DIAG(dev);
          bmuState.errorCount++;
          TLE_StateTransition(TLE_STATEMACH_OPENWIRE_CHECK, 1, TLE_STATEMACH_SHORTTIME);  
        }
        else{
          bmuState.errorCount = 0;
          TLE_StateTransition(TLE_STATEMACH_STARTVMEAS, 0, TLE_STATEMACH_SHORTTIME);  
        }
        
        for(uint8_t i=0;i<sizeof(bmuState.cells.activeCellQue);i++){
          bmuState.cells.openWire[i] = (tle9012.cv[i] < 100)?1:0;
          if(bmuState.cells.activeCellQue[i] == 0){
            bmuState.cells.openWire[i] = 0;
          }
        }
        nvm_runtime_set_openWire(bmuState.cells.openWire);
        //nvm_runtime_set_openWireQueued(tle9012.diag_ol);
        break;
      case 1: 
        TLE_StateTransition(TLE_STATEMACH_OPENWIRE_CHECK, 0, TLE_STATEMACH_SHORTTIME);  
        break;        
      }
      break;
    default:
      break;
    }
    
    if(chThdShouldTerminateX()){
      bRun = false;
    }
    
    chThdSleepMilliseconds(50);
  }
}

msg_t bmu_tle_init()
{
  nvm_get_cellQueue(bmuState.cells.activeCellQue);
  bmuState.usedIndex = 0;
  chThdCreateStatic(waBMU, sizeof(waBMU), NORMALPRIO, procBMU, NULL);
  chSysLock();
  msg_t ret = chThdSuspendS(&thd_trp);
  chSysUnlock();
  return ret;
  
  
}

static int8_t registerRead(TLE9012Driver *dev, uint8_t *b, uint8_t n, uint32_t timeout)
{
  //if(tle_buffer.szReceived > 0){
    memcpy(b,tle_buffer.rxBuffer,16);
    //return tle_buffer.szReceived;
  //}
    return -1;
}
    
static int8_t registerWrite(TLE9012Driver *dev, uint8_t *b, uint8_t n, uint32_t timeout)
{
  // clear rx buffer first
  tle_buffer.szReceived = 0;
  rxcnt = 0;
  uint8_t rx[16];
  uartStartReceive(&UARTD1,16,tle_buffer.rxBuffer);
  uartSendTimeout(&UARTD1,(size_t*)&n,b,TIME_MS2I(10));
  uartStopReceive(&UARTD1);
//  if(tle_buffer.szToSend == 0){
//    tle_buffer.szToSend = n;
//    memcpy(tle_buffer.txBuffer,b,n);
//    tle_buffer.szSent = 1;
//    uartStartSend(&UARTD1,1,tle_buffer.txBuffer);
//  }
  return 0;
}
static int8_t registerXfer(TLE9012Driver *dev, uint8_t *tx, uint8_t txn,uint8_t *rx, uint8_t rxn)
{
  int8_t rxCnt = 0;
  if(rxn > 0){
    uartStartReceive(&UARTD1,rxn,rx);
  }
  uartSendTimeout(&UARTD1,(size_t*)&txn,tx,TIME_MS2I(10));

  if(rxn > 0){
    uint16_t n = rxn*12;
    while(--n) __NOP();
    rxCnt = rxn - uartStopReceive(&UARTD1);
  }
  
  return rxCnt;
}
  
static void txread(UARTDriver *uartp)
{
  (void)uartp;
}
static void txempty(UARTDriver *uartp)
{
  chSysLockFromISR();
  // check for pending buffer
  if(tle_buffer.szToSend > tle_buffer.szSent){
    uartStartSendI(&UARTD1,1,&tle_buffer.txBuffer[tle_buffer.szSent]);
    tle_buffer.szSent++;
  }
  else{
    UARTD1.usart->CR1 &= ~USART_CR1_TCIE;
    tle_buffer.szToSend = 0;
  }
  chSysUnlockFromISR();
}
static void rxErr(UARTDriver *uartp, uartflags_t e)
{
  
}
static void rxChar(UARTDriver *uartp, uint16_t c)
{
  chSysLockFromISR();
  rxcnt++;
//  tle_buffer.rxBuffer[(tle_buffer.szReceived++)&0xF] = c;
//  if(tle_buffer.szReceived == 16){
//    tle_buffer.szReceived = 0;
//  }
  chSysLockFromISR();
}
static void rxEnd(UARTDriver *uartp)
{
    (void)uartp;
}














