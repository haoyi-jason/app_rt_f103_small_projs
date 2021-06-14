#include "ch.h"
#include "hal.h"
#include "at24_eep.h"
#include <string.h>
#include "ads1015.h"
#include "at32_flash.h"
#include "can_frame_handler.h"

#define NVM_FLAG        0x16    // 2021/06


int8_t config_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t id_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t digital_input_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t digital_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t analog_input_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t analog_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t power_output_handler(CANRxFrame *prx,CANTxFrame *ptx);

struct can_frame_handler PacketHandler[] = {
  {0x99,id_handler},
//  {0x140,digital_output_handler},
//  {0x141,digital_input_handler},
//  {0x150,analog_output_handler},
  {0x160,analog_input_handler},  
//  {0x180,power_output_handler},  
};


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
  0x8583,
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
  chSysLock();
  flash_Write(FLASH_START_ADDRESS,(uint16_t*)&nvmParam,sizeof(nvmParam)/2);
  chSysUnlock();
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
  chEvtSignalI(runTime.canThread, EVENT_MASK(1));
  chVTSetI(&vtReport,TIME_MS2I(reportMS),report_cb,NULL);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waCANBUS,1024);
static THD_FUNCTION(procCANBUS,p){
  CANDriver *ip = &CAND1;
  CANRxFrame rxMsg;
  CANTxFrame ptx;

  event_listener_t el;

  chEvtRegister(&ip->rxfull_event,&el,0);
  chVTObjectInit(&vtReport);
  chVTSet(&vtReport,TIME_MS2I(1000),report_cb,NULL);
  
  canStart(ip,&canCfg250K);
  bool bRun = true;
  while(bRun){
    
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));
    if(evt & 0x01){
      while(canReceive(ip,CAN_ANY_MAILBOX,&rxMsg,TIME_IMMEDIATE) == MSG_OK){
        uint16_t eidActive = rxMsg.EID & 0xFFF;
        uint8_t dest = (rxMsg.EID >> 12) & 0xFF;
        if(dest != nvmParam.id) continue;
        if(findHandlerByID(eidActive, &rxMsg,&ptx)>0){
          ptx.EID = (nvmParam.id << 12) | eidActive;
          canTransmit(ip,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
        }
        else{ // not suupported function
          // do nothing
        }
      }
    }
    if(evt & 0x02){
      // report A/D data, stack voltage & stack current
      // voltage: 16-bit unsigned, in unit of 0.1 v
      // current: 16-bit unsigned, in unit of 0.1 A
      // other 2 16-bit data for temperature
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
    
    
    
    if(chThdShouldTerminateX()){
      bRun = false;
    }
    
    
  }
  
};

float engTransfer(int id)
{
  float ret = 0.;
  struct analog_transfer *t = &nvmParam.adc_transfer[id];
  int16_t raw = runTime.analogIn[id];
  if(raw < t->raw_low){
    ret = t->eng_low;
  }
  else if(raw >= t->raw_high){
    ret = t->eng_high;
  }
  else{
    ret = t->eng_low + (runTime.analogIn[id] - t->raw_low)*(t->eng_high - t->eng_low)/(t->raw_high - t->raw_low);
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
  ads1015_set_pga(&ads1115,PGA_1_024);
  int32_t sum = 0;
  while(bRun){
    switch(state){
    case 0:
    case 2:
    case 4:
    case 6:
//      ads1015_set_mux(&ads1115,MUX_AIN0_AIN1);
      ads1015_start_conversion(&ads1115);
      state++;
      break;
    case 1:
    case 3:
    case 5:
    case 7:
      sum += ads1015_read_data(&ads1115);
      state++;
      break;
    case 8:
      runTime.analogIn[2] = sum >> 2;
      state = 10;
      sum = 0;
      ads1015_set_mux(&ads1115,MUX_AIN3_GND);
      break;
    case 10:
    case 12:
    case 14:
    case 16:
      ads1015_start_conversion(&ads1115);
      state++;
      break;
    case 11:
    case 13:
    case 15:
    case 17:
      sum += ads1015_read_data(&ads1115);
      state++;
      break;
    case 18:
      runTime.analogIn[3] = sum >> 2;
      sum = 0;
      ads1015_set_mux(&ads1115,MUX_AIN0_AIN1);
      state = 19;
    case 19:
      for(uint8_t i=0;i< NOF_ADC_CH;i++){
//        runTime.analogResult[i] = (int16_t)(nvmParam.adc_transfer[i].eng_high - nvmParam.adc_transfer[i].eng_low)*(runTime.analogIn[i] - nvmParam.adc_transfer[i].raw_low)/(nvmParam.adc_transfer[i].raw_high - nvmParam.adc_transfer[i].raw_low);
        runTime.analogResult[i] = (int16_t)engTransfer(i);
      }
      runTime.voltage = runTime.analogResult[3];
      runTime.current = runTime.analogResult[2];
      state = 0;
      break;
    }
    
    chThdSleepMilliseconds(10);
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
  
//  palClearPad(GPIOB,12);
//  palSetPad(GPIOB,12);
//  palClearPad(GPIOB,12);
//  palSetPad(GPIOB,12);
  //spiTest();
//  serialTest();
  //eep_test();
//  canTest();
  runTime.canThread = chThdCreateStatic(waCANBUS, sizeof(waCANBUS), NORMALPRIO, procCANBUS, NULL);
  runTime.adsThread = chThdCreateStatic(waADS1115, sizeof(waADS1115), NORMALPRIO, procADS1115, NULL);
  //cmuMgmtInit();
  
//  bmu_ltc_init();
//  balInit();

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
    switch(idx){
    case 0:     // adc raw data, ch 0~3
      ptx->data16[0] = runTime.analogIn[0];
      ptx->data16[1] = runTime.analogIn[1];
      ptx->data16[2] = runTime.analogIn[2];
      ptx->data16[3] = runTime.analogIn[3];
      ptx->DLC = 8;
      break;
    case 1:     // adc raw data, ch 4~7
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