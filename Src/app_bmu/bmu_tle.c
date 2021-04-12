#include "ch.h"
#include "hal.h"
#include "bmu_tle.h"
#include "tle9012.h"
#include "tle9012_def.h"
#include "nvm.h"
#include "std_types.h"

#define TLE_SAVELASTSTATES()   bmuState.lastState = bmuState.state; \
                               bmuState.lastSubState = bmuState.subState

#define TLE_ResetErrorTable()
static thread_t *thisThd;
static thread_reference_t thd_trp = NULL;



static int8_t registerRead(TLE9012Driver *dev, uint8_t *b, uint8_t n, uint32_t timeout);
static int8_t registerWrite(TLE9012Driver *dev, uint8_t *b, uint8_t n, uint32_t timeout);

static const SerialConfig sercfg = {
  2000000,
};

const TLE9012Config tleConfig = {
  &SD1,
  &sercfg
};

TLE9012Driver tle9012 = {
  TLE9012_UNINIT,
  &tleConfig,
  0x01,
  registerRead,
  registerWrite
};


static TLE_STATE_s bmuState = {
  .timer = 0,
  .statereq = TLE_STATE_NO_REQUEST,
  .state = TLE_STATEMACH_UNINITIALIZED,
  .subState = 0,
  .lastState = TLE_STATEMACH_UNINITIALIZED,
  .lastSubState = 0,
};


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
  STD_RETURN_TYPE_e retVal = E_OK;
  TLE_STATE_REQUEST_e statereq = TLE_STATE_NO_REQUEST;
  uint8_t tmpbusID = 0;
  TLE_ADCMODE_e tmpadcMode = TLE_ADCMODE_UNDEFINED;
  TLE_ADCMEAS_CHAN_e tmpadcMeasCh = TLE_ADCMEAS_UNDEFINED;
  
  measCntr = 0;
  chVTObjectInit(&vtMeas);
  chVTSet(&vtMeas,TIME_MS2I(100),meas_cb,NULL);
  
  TLE_SetStateRequest(TLE_STATE_INIT_REQUEST);
  
  chThdResume(&thd_trp,MSG_OK);
  
  bool bRun = true;
  while(bRun){
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
        bmuState.lastSubState = bmuState.subState;
        TLE_StateTransition(TLE_STATEMACH_INITIALIZATION, TLE_EXIT_INITIALIZATION,bmuState.commonDataTransferTime);
      }
      else if(bmuState.subState == TLE_EXIT_INITIALIZATION){
        TLE_SAVELASTSTATES();
        TLE_ResetErrorTable();
        TLE_StateTransition(TLE_STATEMACH_INITIALIZED, TLE_ENTRY_INITIALIZATION, TLE_STATEMACH_SHORTTIME);  
      }
      break;
    case TLE_STATEMACH_INITIALIZED:
      TLE_SAVELASTSTATES();
      TLE_StateTransition(TLE_STATEMACH_STARTVMEAS, TLE_ENTRY, TLE_STATEMACH_SHORTTIME);  
      break;
    case TLE_STATEMACH_STARTVMEAS:
      bmuState.measCtrl = MEAS_CTRL_START | MEAS_CTRL_BITWIDTH(MWAS_BTRL_BIT16) | MAAS_CTRL_PBOFF;
      tle9012_startVMeas(&tle9012, bmuState.measCtrl);
      TLE_StateTransition(TLE_STATEMACH_READVOLTAGE, TLE_ENTRY, TLE_STATEMACH_SHORTTIME);  
      break;
    case TLE_STATEMACH_READVOLTAGE:
      if(bmuState.subState == 0){ // set multi-read mask to read all cells
        tle9012_configMultiRead(&tle9012,0xc0);
        TLE_StateTransition(TLE_STATEMACH_READVOLTAGE, 1, TLE_STATEMACH_SHORTTIME);  
      }
      else if(bmuState.subState == 1){ // read all voltage
        tle9012_readVoltage(&tle9012,&bmuState.cells.voltage);
        TLE_StateTransition(TLE_STATEMACH_READVOLTAGE, 2, TLE_STATEMACH_SHORTTIME);  
      }
      else if(bmuState.subState == 2){
        TLE_StateTransition(TLE_STATEMACH_STARTVMEAS, 0, TLE_STATEMACH_SHORTTIME);  
      }
      break;
    case TLE_STATEMACH_BALANCECONTROL:
      
      break;
      
    case TLE_STATEMACH_ALLGPIOMEASUREMENT:
      break;
      
    case TLE_STATEMACH_READALLGPIO:
      break;
      
    case TLE_STATEMACH_TEMPSEN_READ:
      break;
      
    case TLE_STATEMACH_OPENWIRE_CHECK:
      break;
    default:
      break;
    }
    
    if(chThdShouldTerminateX()){
      bRun = false;
    }
    
    chThdSleepMilliseconds(100);
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
  if(dev->config->devp){
    sdReadTimeout(dev->config->devp,b,n,timeout);
    return 0;
  }
  return -1;
}
static int8_t registerWrite(TLE9012Driver *dev, uint8_t *b, uint8_t n, uint32_t timeout)
{
  if(dev->config->devp){
    sdWriteTimeout(dev->config->devp,b,n,timeout);
    return 0;
  }
  return -1;
}
  















