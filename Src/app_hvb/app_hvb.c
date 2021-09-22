#include "ch.h"
#include "hal.h"
#include "at24_eep.h"
#include <string.h>
#include "ads1015.h"
#include "at32_flash.h"
#include "can_frame_handler.h"
#include "iir.h"
#include <math.h>

#include "bootConfig.h"

#define NVM_FLAG        0x16    // 2021/06


int8_t config_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t id_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t digital_input_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t digital_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t analog_input_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t analog_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t power_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t report_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t heartBeatHandler(CANRxFrame *prx,CANTxFrame *ptx);

can_frame_handler PacketHandler[] = {
  {0x01,config_handler},
  {0x20,heartBeatHandler},
  {0x81,report_handler},
  {0x99,id_handler},
//  {0x140,digital_output_handler},
//  {0x141,digital_input_handler},
//  {0x150,analog_output_handler},
  {0x160,analog_input_handler},  
//  {0x180,power_output_handler},  
  {-1,NULL},
};

static struct{
  uint8_t active;
  uint8_t emgReport;
  uint16_t report_interval_ms;
  uint8_t heartBeatLost;
  uint8_t zero_calib;
  int32_t raw_calib;
  uint8_t band_calib;
  uint8_t calib_cntr;
  uint8_t calib_ch;
  float calib_eng_pt;
}commState;


_float_filter_t filters[2] = {
  {16,1,0,0},
  {16,1,0,0},
};

static boot_info bootInfo;

static SPIConfig spicfg = {
  false,
  NULL,
  GPIOB,
  12,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

//static SerialConfig serialcfg = {
//  115200,
//};

static CANConfig canCfg500K = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_BRP(3) | CAN_BTR_TS1(14) | CAN_BTR_TS2(1) | CAN_BTR_SJW(0)
};

// can config for 250K baud
static CANConfig canCfg250K = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_BRP(7) | CAN_BTR_TS1(14) | CAN_BTR_TS2(1) | CAN_BTR_SJW(0)
};

static const I2CConfig i2ccfg = {
  OPMODE_I2C,
  100000,
  STD_DUTY_CYCLE
};

static ads1x15_config_t ads1115_config = {
  &I2CD2,
  &i2ccfg,
  DEV_ADS1115
};

ADS1x15Driver ads1115 = {
  &ads1115_config,
  0x8583, // 128 sps
  0x0,
  0x1,
  ADS1015_ADDR_GND
};

#define NOF_ADC_CH      4
#define NOF_AVERAGE     8

struct{
  uint8_t id;
  uint16_t timer;
  uint16_t voltage;
  uint16_t current;
  thread_t *canThread;
  thread_t *adsThread;
  int32_t analogAccum[8];       // accumulated raw data 
  int16_t analogIn[8]; //  ads1115 raw data
  int16_t analogResult[8]; // physical value, adc_value * analog_transfer
  thread_reference_t canRef;
  mutex_t mutex;
}runTime;

struct{
  uint8_t flag;
  uint8_t id;
  uint32_t boardId;
  struct analog_transfer adc_transfer[8];
}nvmParam;

void eep_test()
{
  at24eep_init(&I2CD2,32,1024,0x50,2);
  eepTest();  
}
void nvmRead()
{
  flash_Read(FLASH_START_ADDRESS,(uint16_t*)&nvmParam,sizeof(nvmParam)/2);
}

void nvmWrite()
{
  //chMtxLock(&runTime.mutex);
  chSysLock();
  flash_Write(FLASH_START_ADDRESS,(uint16_t*)&nvmParam,sizeof(nvmParam)/2);
  chSysUnlock();
  //chMtxUnlock(&runTime.mutex);
}

void nvmLoadDefault()
{
  nvmParam.flag = NVM_FLAG;
  nvmParam.boardId = BOARD_ID;
  nvmParam.id = 0x3F;  // for HVB, ID always = 0x1f (31d) for group 1 to 6 (3-bit MSB)
  for(uint8_t i=0;i<8;i++){
    nvmParam.adc_transfer[i].raw_low = 0;
    nvmParam.adc_transfer[i].raw_high = 32767;
    nvmParam.adc_transfer[i].eng_low = 0.0;
    nvmParam.adc_transfer[i].eng_high = 100.0;
    nvmParam.adc_transfer[i].factor = 100;
  }
  // ch.2 for current
    nvmParam.adc_transfer[2].raw_low = -20000;
    nvmParam.adc_transfer[2].raw_high = 20000;
    nvmParam.adc_transfer[2].eng_low = -4000.0;
    nvmParam.adc_transfer[2].eng_high = 4000.0;

    //ch.3 for voltage
    nvmParam.adc_transfer[3].raw_low = 0;
    nvmParam.adc_transfer[3].raw_high = 32767;
    nvmParam.adc_transfer[3].eng_low = 0.0;
    nvmParam.adc_transfer[3].eng_high = 10300.0;
}

msg_t nvm_get_block(uint8_t id, uint8_t *dptr)
{
  if(id == 0x99){
    flash_Read(BL_CONFIG_ADDRESS,(uint16_t*)dptr,sizeof(boot_info)/2);
  }
  return MSG_OK;
}

msg_t nvm_set_block(uint8_t id, uint8_t *dptr)
{
  if(id == 0x99){
    flash_Write(BL_CONFIG_ADDRESS,(uint16_t*)dptr,sizeof(boot_info)/2);
  }
  return MSG_OK;
}


void nvmFlashInit()
{
  nvmRead(); // load nvm
  if(nvmParam.flag != NVM_FLAG){
    // load default
    nvmLoadDefault();
    nvmWrite();
  }
}

static thread_t *canThread;
static thread_t *adThread;

static virtual_timer_t vtReport;
static uint16_t reportMS=1000;
static void report_cb(void *arg)
{
  chSysLockFromISR();
  if(commState.active == 1){
    chEvtSignalI(runTime.canThread, EVENT_MASK(1));
  }
  chVTSetI(&vtReport,TIME_MS2I(commState.report_interval_ms),report_cb,NULL);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waCANBUS,2048);
static THD_FUNCTION(procCANBUS,p){
  CANDriver *ip = &CAND1;
  CANRxFrame rxMsg;
  CANTxFrame ptx;

  event_listener_t el;

  chEvtRegister(&ip->rxfull_event,&el,0);
  chVTObjectInit(&vtReport);
  chVTSet(&vtReport,TIME_MS2I(1000),report_cb,NULL);
  
  canStart(ip,&canCfg250K);

  boot_info bootInfo;
  nvm_get_block(0x99,(uint8_t*)&bootInfo);
  if(bootInfo.product_id != 0x20063000){
    bootInfo.product_id = 0x20063000;
    nvm_set_block(0x99,(uint8_t*)&bootInfo);
  }
  

  chThdResume(&runTime.canRef,MSG_OK);
  bool bRun = true;
  while(bRun){
    
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));
    if(evt & 0x01){
      while(canReceive(ip,CAN_ANY_MAILBOX,&rxMsg,TIME_IMMEDIATE) == MSG_OK){
        if(rxMsg.EID == 0x0020){ // heartbeat
          heartBeatHandler(NULL,NULL);
          commState.active = 1;
//          continue;
        }
        else{
          uint16_t eidActive = rxMsg.EID & 0xFFF;
          uint8_t dest = (rxMsg.EID >> 12) & 0xFF;
          if(dest == nvmParam.id){
            if(findHandlerByID(eidActive, &rxMsg,&ptx)>0){
              ptx.EID = (nvmParam.id << 12) | eidActive;
              canTransmit(ip,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
            }
            else{ // not suupported function
              // do nothing
            }
          }
        }
      }
    }
    if(evt & 0x02){
      // report A/D data, stack voltage & stack current
      // voltage: 16-bit unsigned, in unit of 0.1 v
      // current: 16-bit unsigned, in unit of 0.1 A
      // other 2 16-bit data for temperature
      commState.heartBeatLost++;
      if(commState.heartBeatLost < 10){
        ptx.EID = nvmParam.id << 12;
        ptx.EID |= 0x120; // todo: define message id
        ptx.IDE = CAN_IDE_EXT;
        ptx.DLC = 8;
        ptx.RTR = CAN_RTR_DATA;
        for(uint8_t i=0;i<4;i++){
          ptx.data16[i] = runTime.analogResult[i];
        }
        canTransmit(ip, CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
      }
      else{
        commState.active = 0;
      }
    }
    
    
    
    if(chThdShouldTerminateX()){
      bRun = false;
    }
    
    
  }
  
};

float engTransfer(int id, int16_t raw)
{
  float ret = 0.;
  struct analog_transfer *t = &nvmParam.adc_transfer[id];
  //int16_t raw = runTime.analogIn[id];
  if(raw < t->raw_low){
    ret = t->eng_low;
  }
  else if(raw >= t->raw_high){
    ret = t->eng_high;
  }
  else{
    ret = t->eng_low + (raw - t->raw_low)*(t->eng_high - t->eng_low)/(t->raw_high - t->raw_low);
  }
  return ret;
}

static THD_WORKING_AREA(waADS1115,1024);
static THD_FUNCTION(procADS1115,p){
  (void)p;
  uint16_t volts[8];
  uint16_t amps[8];
  ads1015_set_mode(&ads1115,MODE_SINGLE);
//  ads1015_set_thresLow(&ads1115);
//  ads1015_set_thresHigh(&ads1115);
  //PGA_1_024
  uint8_t voltIndex = 0, ampIndex = 0;
  bool bRun = true;
  uint8_t state = 0;
  int32_t voltSum,ampSum;
  ads1015_set_mux(&ads1115,MUX_AIN0_AIN1);
  ads1015_set_pga(&ads1115,PGA_1_024);
  int32_t sum = 0;
  int16_t raw;
  
  while(bRun){
    switch(state){
    case 0:
      ads1015_start_conversion(&ads1115);
      state++;
      break;
    case 1:
      raw = ads1015_read_data(&ads1115);
      runTime.analogIn[2] = raw;
      ads1015_set_mux(&ads1115,MUX_AIN3_GND);
      iir_insert_f(&filters[0],engTransfer(2,raw));
      if(commState.calib_ch == 2){
        if(commState.zero_calib == 1){
          commState.raw_calib += raw;
        }
        else if(commState.band_calib == 1){
          commState.raw_calib += raw;
        }
        commState.calib_cntr++;
        if(commState.calib_cntr == 16){
          commState.raw_calib >>= 4;
          if(commState.zero_calib == 1){
            nvmParam.adc_transfer[2].raw_high = 20000 + commState.raw_calib;
            nvmParam.adc_transfer[2].raw_low = -20000 + commState.raw_calib;
            nvmWrite();
            commState.calib_ch = 0xff;
            commState.zero_calib = 0;
          }
          else if(commState.band_calib == 1){
            float ratio = nvmParam.adc_transfer[2].eng_high - nvmParam.adc_transfer[2].eng_low;
            ratio /=2;
            ratio /= commState.calib_eng_pt;
            //if(ratio < 0) ratio *= -1;
            int16_t offset = nvmParam.adc_transfer[2].raw_high + nvmParam.adc_transfer[2].raw_low;
            offset >>= 1;
            commState.raw_calib -= offset;
            nvmParam.adc_transfer[2].raw_high = commState.raw_calib*ratio +offset;
            nvmParam.adc_transfer[2].raw_low  = -commState.raw_calib*ratio + offset;
            nvmWrite();
            commState.calib_ch = 0xff;
            commState.band_calib = 0;
          }
        }
      }
      state++;
     // state = 0;
    case 2:
      ads1015_start_conversion(&ads1115);
      state++;
      break;
    case 3:
      raw = ads1015_read_data(&ads1115);
      runTime.analogIn[3] = raw;
      ads1015_set_mux(&ads1115,MUX_AIN0_AIN1);
      iir_insert_f(&filters[1],engTransfer(3,raw));
      runTime.analogResult[2] = (int16_t)lround(filters[0].last);
      runTime.analogResult[3] = (int16_t)lround(filters[1].last);
      if(commState.calib_ch == 3){
        commState.raw_calib += raw;
        commState.calib_cntr++;
        if(commState.calib_cntr == 16){
          commState.raw_calib >>= 4;
          if(commState.zero_calib == 1){
            nvmParam.adc_transfer[3].raw_low += commState.raw_calib;
            nvmWrite();
            commState.calib_ch = 0xff;
            commState.zero_calib = 0;
          }
          else if(commState.band_calib == 1){
            float ratio = nvmParam.adc_transfer[2].eng_high - nvmParam.adc_transfer[2].eng_low;
            ratio /= commState.calib_eng_pt;
            nvmParam.adc_transfer[2].raw_high *= ratio;
            nvmParam.adc_transfer[2].raw_low *= ratio;
            nvmWrite();
            commState.calib_ch = 0xff;
            commState.band_calib = 0;
          }
        }
      }
      state = 0;
      break;
    default:break;
    }
    
    chThdSleepMilliseconds(100);
    if(chThdShouldTerminateX()){
      bRun = false;
    }
  }
  
}


void main(void)
{
  halInit();
  chSysInit();
  
  // remapI2C2
  //AFIO->MAP5 |= AFIO_MAP5_I2C2_GRMP_10;
  //AFIO->MAP6 |= AFIO_MAP6_CAN1_GRMP;
  //AFIO->MAP6 |= AFIO_MAP6_CAN2_GRMP;
  
  //nvmInit(&I2CD2);
  nvmFlashInit();
  
  
  commState.active = 1;
  commState.report_interval_ms = 1000;
  commState.zero_calib = 0;
  commState.band_calib = 0;
  commState.raw_calib = 0;
  commState.calib_ch = 0xff;
  
  runTime.canThread = chThdCreateStatic(waCANBUS, sizeof(waCANBUS), NORMALPRIO, procCANBUS, NULL);
  chThdSuspendS(&runTime.canRef);
  runTime.adsThread = chThdCreateStatic(waADS1115, sizeof(waADS1115), NORMALPRIO, procADS1115, NULL);

  while(1){
    chThdSleepMilliseconds(500);
  }
}

int8_t id_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  // only group can be changed
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
      if((id == 0x1F) && (gid > 0)){
        nvmParam.id = prx->data8[2];
        nvmWrite();
      }
    }
    else if(prx->DLC == 8){
      bool valid = true;
      for(uint8_t i=0;i<4;i++){
        if(prx->data8[i*2] != 0x55)
          valid = false;
        if(prx->data8[i*2+1] != 0xAA)
          valid = false;
      }
      if(valid){
        nvmLoadDefault();
      }
    }
  }
  return 0;
 
}

int8_t config_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  boot_info bootInfo;
  flash_Read(BL_CONFIG_ADDRESS,(uint16_t*)&bootInfo,sizeof(boot_info)/2);
//  nvm_get_block(0x99,(uint8_t*)&bootInfo);
  if(prx->RTR == CAN_RTR_REMOTE){
    ptx->RTR = CAN_RTR_DATA;
    ptx->DLC = 8;
    ptx->IDE = CAN_IDE_EXT;
    ptx->data32[0] = bootInfo.fw_version;
    ptx->data32[1] = bootInfo.product_id;
    return 1;
  }
  else if(prx->RTR == CAN_RTR_DATA){
    if(prx->data32[0] == BOOT_KEY){
        bootInfo.bootOption = BOOT_KEY;
        flash_Write(BL_CONFIG_ADDRESS,(uint16_t*)&bootInfo,sizeof(boot_info)/2);
        chSysDisable();
        NVIC_SystemReset();
    }
  }
  return 0;
}
int8_t digital_input_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  return 0;
}
int8_t digital_output_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  return 0;
}
int8_t analog_input_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(ptx->RTR == CAN_RTR_REMOTE){
    uint8_t idx = prx->EID & 0xFF;
    uint8_t ch = prx->data8[0];
    uint8_t opt = prx->data8[1];
    switch(opt){
    case 0: // raw low/high
      ptx->data16[0] = nvmParam.adc_transfer[ch].raw_low;
      ptx->data16[1] = nvmParam.adc_transfer[ch].raw_high;
      ptx->DLC = 4;
      break;
    case 1: // eng low/high
      memcpy(&ptx->data8[0],&nvmParam.adc_transfer[ch].eng_low,4);
      memcpy(&ptx->data8[4],&nvmParam.adc_transfer[ch].eng_high,4);
      ptx->DLC = 8;       
      break;
    case 0xE:     // adc raw data, ch 0~3
      ptx->data16[0] = runTime.analogIn[0];
      ptx->data16[1] = runTime.analogIn[1];
      ptx->data16[2] = runTime.analogIn[2];
      ptx->data16[3] = runTime.analogIn[3];
      ptx->DLC = 8;
      break;
    case 0xF:     // adc raw data, ch 4~7
      ptx->data16[0] = runTime.analogIn[4];
      ptx->data16[1] = runTime.analogIn[5];
      ptx->data16[2] = runTime.analogIn[6];
      ptx->data16[3] = runTime.analogIn[7];
      ptx->DLC = 8;
      break;
    }
    return 1;
  }
  else{ // adc channel config
    uint8_t ch = prx->data8[0];
    uint8_t opt = prx->data8[1];  
    int16_t tmp;
    if(ch < 4){
      switch(opt){
      case 0: // raw low;
        tmp = (int16_t)prx->data16[1];
        nvmParam.adc_transfer[ch].raw_low = (int32_t)(tmp);
        break;
      case 1: // raw_high
        tmp = (int16_t)prx->data16[1];
        nvmParam.adc_transfer[ch].raw_high = (int32_t)(tmp);
        break;
      case 2: // eng_low, float
        memcpy(&nvmParam.adc_transfer[ch].eng_low,(void*)&prx->data8[2],4);
        break;
      case 3: // eng high, float
        memcpy(&nvmParam.adc_transfer[ch].eng_high,(void*)&prx->data8[2],4);
        break;
      case 4: // offset calibration
        if(commState.band_calib == 0){
          commState.zero_calib = 1;
          commState.raw_calib = 0;
          commState.calib_cntr = 0;
          commState.calib_ch = ch;
        }
        break;
      case 5: // band calib
        if(commState.zero_calib == 0){
          commState.band_calib = 1;
          commState.raw_calib = 0;
          commState.calib_cntr = 0;
          commState.calib_ch = ch;
          memcpy(&commState.calib_eng_pt,(void*)&prx->data8[2],4);
        }
      default:break;
      }
    }
    else if(ch == 0x99){ // save
      nvmWrite();
    }
  }
  return 0;
}
int8_t analog_output_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  return 0; // not supported
}
int8_t power_output_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  return 0; 
}

// set/get report state
int8_t report_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(prx->RTR == CAN_RTR_REMOTE){
    //ptx->EID = eidActive | (nvmBoard.id << 12);
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
