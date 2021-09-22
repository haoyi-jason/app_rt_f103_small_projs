#include "ch.h"
#include "hal.h"
#include "at24_eep.h"
#include <string.h>
#include "ad5593r.h"
#include "at32_flash.h"
#include "can_frame_handler.h"
#include "bootConfig.h"

#define NVM_FLAG        0x16    // 2021/06

// BTT6050 current shunt scaling factor
#define BTT_KFACTOR     1500
#define ADC_LSB         4096 
#define ADC_VREF        5    // V
#define SHUNT_R         1200    // ohmn
//#define VSOURCE_MA      BTT_KFACTOR*ADC_VREF/ADC_LSB/SHUNT_R*1000
#define VSOURCE_MA      1.50

#define BCU_BOARD_ID    0x01

typedef int8_t(can_pack_handler)(CANRxFrame *prx,CANTxFrame *ptx);


int8_t config_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t id_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t digital_input_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t digital_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t analog_input_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t analog_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t power_output_handler(CANRxFrame *prx,CANTxFrame *ptx);
int8_t heartBeatHandler(CANRxFrame *prx,CANTxFrame *ptx);


struct can_frame_handler PacketHandler[] = {
  {0x01,config_handler},
  {0x20,heartBeatHandler},
  {0x99,id_handler},
  {0x140,digital_output_handler},
  {0x141,digital_input_handler},
  {0x150,analog_output_handler},
  {0x160,analog_input_handler},  
  {0x180,power_output_handler},  
  {-1,NULL},
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
static void icuwidthcb(ICUDriver *icup);
static void icuperiodcb(ICUDriver *icup);

static ICUConfig icucfg_t2 = {
  ICU_INPUT_ACTIVE_HIGH,
  10000,
  icuwidthcb,
  icuperiodcb,
  NULL,
  ICU_CHANNEL_1,
  0
};

static ICUConfig icucfg_t3 = {
  ICU_INPUT_ACTIVE_LOW,
  100000,
  icuwidthcb,
  icuperiodcb,
  NULL,
  ICU_CHANNEL_1,
  0
};

static AD5593RConfig adc_cfg = {
  &I2CD2,
  &i2ccfg,
  AD5593_DEV1,
  0x0320,
  0x0ff,
  0x00,
  NULL,
  VIO_2VREF
};

AD5593RDriver ad5593 = {
  AD5593R_UNINIT,
  &adc_cfg,
  AD5593R_ADDR_BASE | AD5593_DEV0,
};

//static ads1x15_config_t ads1115_config = {
//  &I2CD2,
//  &i2ccfg,
//  DEV_ADS1115
//};
//
//ADS1x15Driver ads1115 = {
//  &ads1115_config,
//  0x8583,
//  0x0,
//  ADS1015_ADDR_GND
//};



struct vsource{
  uint8_t enable;
  uint8_t state;
  uint16_t current_limit;
  uint16_t working_current;
  uint16_t max_current;
  uint16_t counter;
  uint8_t switchDelay;
  uint8_t id;
};


struct icuState{
  uint16_t valid;
  icucnt_t period;
  icucnt_t width;
};

static struct{
  uint8_t id;
  uint16_t timer;
  uint16_t voltage;
  uint16_t current;
  thread_t *canThread;
  thread_t *canThread2;
  thread_t *adsThread;
  uint8_t digitIn[2]; // isolated di
  uint8_t digitOut[2];// relayout out
  uint16_t pwmInDuty[2];     // 
  uint16_t analogIn[9]; // ad5593 raw
  struct vsource source[2];
  struct icuState icuState[2];
  uint8_t heartBeatLost;
  uint8_t active;
  uint16_t report_ms;
  uint8_t restartSource;
}runTime;

struct{
  uint8_t flag;
  uint8_t id;
  uint32_t boardId;
  uint8_t default_digit_out;
  struct vsource vsource[2];
  struct analog_transfer adc_transfer[8];
}nvmParam;



void spiTest()
{
  SPIDriver *dev = &SPID2;
  uint8_t tx[32];
  uint8_t reg = 0x1;
  for(uint8_t i=0;i<32;i++)
    tx[i] = i*4;
  spiAcquireBus(dev);
  spiStart(dev,&spicfg);
  spiSelect(dev);
  spiSend(dev,32,tx);
//  spiReceive(dev->config->devp,n,b);
  spiUnselect(dev);
  spiStop(dev);
  spiReleaseBus(dev);
  
}

void serialTest()
{
  
//  sdStart(&SD1,&serialcfg);
//  for(uint8_t i=0;i<10;i++){
//    sdWrite(&SD1,"Hello\n",5);
//    chThdSleepMilliseconds(100);
//  }

}

typedef struct{
  uint32_t pgn;
  uint8_t buffer[8];
  uint8_t len;
  uint8_t dst;
  uint8_t src;
  uint8_t prio;
}j1939_msg_t;

static msg_t canSend(CANDriver *p, j1939_msg_t *m)
{
  CANTxFrame ptx;
  uint8_t i;
  
  ptx.EID = (m->prio << 26) | (m->pgn << 8) | m->src;
  ptx.IDE = CAN_IDE_EXT;
  ptx.RTR = CAN_RTR_DATA;
  ptx.DLC = m->len;
  memcpy(ptx.data8,m->buffer,m->len);
  
  return canTransmit(p,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
}

void canTest()
{
  canStart(&CAND1,&canCfg250K);
  //canStart(&CAND2,&canCfg500K);

  j1939_msg_t msg;
//  msg.dst = 0x1234;
//  msg.pgn = 100;
//  msg.src =  0x5678;
//  msg.buffer[0] = 0x11;
//  msg.buffer[1] = 0x22;
//  msg.buffer[2] = 0x33;
//  msg.buffer[3] = 0x44;
//  msg.buffer[4] = 0x55;
//  msg.buffer[5] = 0x66;
//  msg.buffer[6] = 0x77;
//  msg.buffer[7] = 0x88;
//  msg.len = 8;
  
  for(uint8_t i=0;i<10;i++){
    canSend(&CAND1,&msg);
    msg.buffer[0]++;
  }
  //canSend(&CAND2,&msg);
}

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
  nvmParam.id = BCU_BOARD_ID; // always 0x01 form comm board, cannot be modified
  nvmParam.default_digit_out = 0x0;
  nvmParam.vsource[0].enable = 0;
  nvmParam.vsource[0].current_limit = 100;
  nvmParam.vsource[1].enable = 0;
  nvmParam.vsource[1].current_limit = 100;
  
  for(uint8_t i=0;i<6;i++){
    nvmParam.adc_transfer[i].raw_low = 0;
    nvmParam.adc_transfer[i].raw_high = 4095;
    nvmParam.adc_transfer[i].eng_low = 0.0;
    nvmParam.adc_transfer[i].eng_high = 100.0;
    nvmParam.adc_transfer[i].factor = 100;
  }
  for(uint8_t i=6;i<8;i++){
    nvmParam.adc_transfer[i].raw_low = 0;
    nvmParam.adc_transfer[i].raw_high = 4095;
    nvmParam.adc_transfer[i].eng_low = 0.0;
    nvmParam.adc_transfer[i].eng_high = 6250.0;
    nvmParam.adc_transfer[i].factor = 1.523;
  }
}

void nvmParamPowerUp()
{
  runTime.source[0].enable = nvmParam.vsource[0].enable;
  runTime.source[1].enable = nvmParam.vsource[1].enable;
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
  nvmParamPowerUp();
}

void set_relay_output(uint8_t ch, uint8_t state)
{
  switch(ch){
  case 0:
    state==0?palSetPad(GPIOB,15):palClearPad(GPIOB,15);
    break;
  case 1:
    state==0?palSetPad(GPIOA,8):palClearPad(GPIOA,8);
    break;
  }
}

void set_dc_output(uint8_t ch, uint8_t state)
{
  switch(ch){
  case 0:
    state==0?palSetPad(GPIOA,15):palClearPad(GPIOA,15);
    break;
  case 1:
    state==0?palSetPad(GPIOB,3):palClearPad(GPIOB,3);
    break;
  }
}

void ad5593_rst_toggle()
{
  palSetPad(GPIOA,9);
  chThdSleepMilliseconds(1);
  palClearPad(GPIOA,9);
  chThdSleepMilliseconds(1);
}


static thread_t *canThread;
static thread_t *adThread;

static virtual_timer_t vtReport;
static uint16_t reportMS=500;
static void report_cb(void *arg)
{
  chSysLockFromISR();
  if(runTime.active){
    chEvtSignalI(runTime.canThread, EVENT_MASK(1));
  }
  chVTSetI(&vtReport,TIME_MS2I(reportMS),report_cb,NULL);
  chSysUnlockFromISR();
}

static void icuwidthcb(ICUDriver *icup)
{
  if(icup == &ICUD2){
//    runTime.icuState[0].valid = 0;
    runTime.icuState[0].width = icuGetWidthX(icup);
  }
  else if(icup == &ICUD3){
//    runTime.icuState[1].valid = 0;
    runTime.icuState[1].width = icuGetWidthX(icup);
  }
}
static void icuperiodcb(ICUDriver *icup)
{
  if(icup == &ICUD2){
    runTime.icuState[0].period = icuGetPeriodX(icup);
    runTime.icuState[0].valid = 1;
  }
  else if(icup == &ICUD3){
    runTime.icuState[1].period = icuGetPeriodX(icup);
    runTime.icuState[1].valid = 1;
  }
}

static THD_WORKING_AREA(waCANBUS,2048);
//static THD_WORKING_AREA(waCANBUS2,1024);
static THD_FUNCTION(procCANBUS,p){
//  CANDriver *ip = &CAND1;
  CANDriver *ip = (CANDriver*)p;
  CANRxFrame rxMsg;
  CANTxFrame ptx;

  event_listener_t el;

  chEvtRegister(&ip->rxfull_event,&el,0);
  chVTObjectInit(&vtReport);
  chVTSet(&vtReport,TIME_MS2I(runTime.report_ms),report_cb,NULL);
  bool bRun = true;
  
  canStart(ip,&canCfg250K);
  canStart(&CAND2,&canCfg250K); // for communication purpose only, not used in current project
  static uint8_t reportState = 0;
  
  boot_info bootInfo;
  nvm_get_block(0x99,(uint8_t*)&bootInfo);
  bootInfo.product_id = 0x20066000;
  nvm_set_block(0x99,(uint8_t*)&bootInfo);
  
  while(bRun){
    
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));
    if(evt & 0x01){
      while(canReceive(ip,CAN_ANY_MAILBOX,&rxMsg,TIME_IMMEDIATE) == MSG_OK){
        if(rxMsg.EID == 0x0020){ // heartbeat
          heartBeatHandler(NULL,NULL);
          continue;
        }
        uint16_t eidActive = rxMsg.EID & 0xFFF;
        uint8_t dest = (rxMsg.EID >> 12) & 0xFF;
        if((dest == nvmParam.id) || (dest == 0x00)){
          heartBeatHandler(NULL,NULL);
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
    if(evt & 0x02){
        runTime.heartBeatLost++;
        if(runTime.heartBeatLost < 10){
          // report A/D data, stack voltage & stack current
          // voltage: 16-bit unsigned, in unit of 0.1 v
          // current: 16-bit unsigned, in unit of 0.1 A
          // other 2 16-bit data for temperature
          switch(reportState){
          case 0:
            ptx.EID = nvmParam.id << 12;
            ptx.EID |= 0x121; // todo: define message id
            ptx.DLC = 8;
            ptx.RTR = CAN_RTR_DATA;
            ptx.IDE = 1;
    //        chSysLock();
            ptx.data16[0] = runTime.pwmInDuty[0];
            ptx.data16[1] = runTime.pwmInDuty[1];
            ptx.data16[2] = runTime.source[0].working_current;
            ptx.data16[3] = runTime.source[1].working_current;
            canTransmit(ip, CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
      //      canTransmit(&CAND2, CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
            reportState++;
            break;
          case 1:
            ptx.EID = nvmParam.id << 12;
            ptx.EID |= 0x122; // todo: define message id
            ptx.DLC = 8;
            ptx.RTR = CAN_RTR_DATA;
            ptx.IDE = 1;
    //        chSysLock();
    //        ptx.data8[0] = (board_input_state(0)) | (board_input_state(1)<<1);
    //        ptx.data8[1] = (board_output_state(0)) | (board_output_state(1)<<1);        
            ptx.data8[0] = (runTime.digitIn[0]) | (runTime.digitIn[1] << 1);
            ptx.data8[1] = (runTime.digitOut[0]) | (runTime.digitOut[1] << 1);
            canTransmit(ip, CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
            reportState = 0;
            break;
            
          }
        }
        else{
          // no signal form BMS, close vsource and set digital outout to 0
//          board_relay_clr(0);
//          board_relay_clr(1);
//          board_power_clr(0);
//          board_power_clr(1);
          runTime.source[0].enable = 0;
          runTime.source[1].enable = 0;
          runTime.active = 0;
        }
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
  uint16_t raw = runTime.analogIn[id];
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

static THD_WORKING_AREA(waVS1,1024);
static THD_WORKING_AREA(waVS2,1024);
static THD_FUNCTION(procVS,p)
{
  struct vsource *vs = (struct vsource*)p;
  
  bool bRun = true;
  uint8_t cycleCount = 20;
  vs->switchDelay = 0;
  while(bRun){
    if(chThdShouldTerminateX()){
      bRun = false;
    }
    
    switch(vs->state){
    case 0:// init
      if(vs->switchDelay > 0){
        vs->switchDelay--;
      }
      else if(vs->switchDelay == 0){
        if(vs->enable){
          board_power_set(vs->id);
          vs->state++;
          vs->switchDelay = 0;
        }
      }
      break;
    case 1:
      if(vs->switchDelay > 0){
        vs->switchDelay--;
      }
      else{ // togglo off
        board_power_clr(vs->id);
        vs->switchDelay = 0;
        if(cycleCount == 0){
          vs->state = 4;
        }else{
          cycleCount--;
          vs->state = 0;
        }
      }
      break;
    case 4: // final turn on
      if(vs->switchDelay > 0){
        vs->switchDelay--;
      }
      else{
        board_power_set(vs->id);
        vs->state = 100;
      }
      break;
    case 99: // turn off 
      if(vs->enable == 1){
        if(vs->counter == 0){
          vs->state = 0; // restart
          cycleCount = 20;
          vs->switchDelay = 100;
        }
        else{
          vs->counter--;
        }
      }
      break;
    case 100: // normal running
      if(vs->enable == 0){
        vs->state = 99;
        board_power_clr(vs->id);
      }
      else{
        if(vs->working_current > vs->current_limit){
          vs->counter++;
          if(vs->counter > 100){ // over 1 second over current
            vs->state = 99;
            board_power_clr(vs->id);
          }
        }
        else{
          vs->counter = 0;
        }
        
        if(vs->working_current > vs->max_current){
          vs->max_current = vs->working_current;
        }
        
      }
      break;
    default:break;
    }
    
    chThdSleepMilliseconds(8);
  }
  
}

static THD_WORKING_AREA(waADC,1024);
static THD_FUNCTION(procADC,p){
  (void)p;
  uint16_t volts[8];
  uint16_t amps[8];
//  ads1015_set_pga(&ads1115,PGA_4_096);
//  ads1015_set_mode(&ads1115,MODE_SINGLE);
  uint8_t voltIndex = 0, ampIndex = 0;
  bool bRun = true;
  uint8_t state = 0;
  uint16_t voltSum,ampSum;
  ad5593rObjectInit(&ad5593);
  palClearPad(GPIOA,9);
  chThdSleepMilliseconds(10);
  palSetPad(GPIOA,9);
  ad5593rConfig(&ad5593);
  uint16_t adResult[10];
  uint8_t rxBuf[18];
  // check for initial state
  if(nvmParam.default_digit_out & 0x01){
    board_relay_set(0);
  }
  else{
    board_relay_clr(0);
  }
  if(nvmParam.default_digit_out & 0x02){
    board_relay_set(1);
  }
  else{
    board_relay_clr(1);
  }
  
  runTime.digitOut[0] = board_output_state(0);
  runTime.digitOut[0] = board_output_state(1);
  runTime.digitIn[0] = board_input_state(0);
  runTime.digitIn[1] = board_input_state(1);
  
  runTime.source[0].switchDelay = 0;
  runTime.source[1].switchDelay = 0;
  runTime.restartSource = 1;
  
    
  board_power_clr(0);
  board_power_clr(1);
  while(bRun){
    switch(state){
    case 0:
      ad5593rADCRead(&ad5593,rxBuf);
      for(uint8_t i=0;i<9;i++){
        runTime.analogIn[i] = (rxBuf[i*2+1]) | ((rxBuf[i*2] & 0xF)<<8);
      }
      state++;
      break;
    case 1:
//      runTime.source[0].working_current = (uint16_t)(runTime.analogIn[6]*VSOURCE_MA);
//      runTime.source[1].working_current = (uint16_t)(runTime.analogIn[7]*VSOURCE_MA);
      runTime.source[0].working_current = (uint16_t)engTransfer(6);
      runTime.source[1].working_current = (uint16_t)engTransfer(7);
      state++;
      break;
    case 2:
      // check if over current
//      for(uint8_t i=0;i<2;i++){
//        if(runTime.source[i].working_current > nvmParam.vsource[i].current_limit){
//          runTime.source[i].counter++;
//          if(runTime.source[i].counter > 100){
////            board_power_clr(i);
////            runTime.source[i].enable = 0;
////            runTime.source[i].state = 99; // turn power off
//            
//          }
//        }
//        else{
//          runTime.source[i].counter = 0;
////          if((runTime.source[i].enable == 1) && (board_vout_state(i) == 0)){
////              board_power_set(i);
////          }
//        }
//        // current_limit field in runTime represent for maximum current in history
//        if(runTime.source[i].working_current > runTime.source[i].current_limit){
//          runTime.source[i].current_limit = runTime.source[1].working_current;
//        }
//      }
      state++;
      break;
    case 3:
      // read di/do input state
      for(uint8_t i=0;i<2;i++){
        runTime.digitIn[i] = board_input_state(i);
        runTime.digitOut[i] = board_output_state(i);
      }
      state++;
      break;
    case 4:
      // read pwm in
      for(uint8_t i=0;i<2;i++){
        if(runTime.icuState[i].valid){
          runTime.icuState[i].valid = 0;
          runTime.pwmInDuty[i] = 100- runTime.icuState[i].width*100/runTime.icuState[i].period;
        }
        else{
          runTime.pwmInDuty[i] = 100;
        }
      }
      state = 0;
      break;
    }
    
    if(runTime.restartSource){
      runTime.restartSource = 0;
//      runTime.source[0].enable = 1;
//      runTime.source[1].enable = 1;
      runTime.source[0].state = 0;
      runTime.source[1].state = 0;
      runTime.source[0].current_limit = 0;
      runTime.source[1].current_limit = 0;
      
      //runTime.source[0].switchDelay = 100;
      //runTime.source[1].switchDelay = 120;
    }
    // issue power control
//    for(uint8_t i=0;i<2;i++){
      
//      if(runTime.source[i].enable == 0){ // disable
//        if(board_vout_state(i) == 1){
//          board_power_clr(i);
//          runTime.source[i].switchDelay = 100 + i*20; 
//        }
//      }
//      else{ // enable
//        if(board_vout_state(i) == 0){
//          if(runTime.source[i].switchDelay == 0){
//            board_power_set(i);
//            //chThdSleepMilliseconds(100);
//          }
//          else{
//            runTime.source[i].switchDelay--;
//          }
//        }
//      }
//    }
    
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
  AFIO->MAP6 |= AFIO_MAP6_CAN1_GRMP;
  AFIO->MAP6 |= AFIO_MAP6_CAN2_GRMP;
  
  AFIO->MAP7 |= AFIO_MAP7_SWJTAG_GRMP_010;
  
  //nvmInit(&I2CD2);
  nvmFlashInit();

  // start icu
  icuStart(&ICUD2,&icucfg_t2);
  icuStart(&ICUD3,&icucfg_t3);
  
  icuStartCapture(&ICUD2);
  icuStartCapture(&ICUD3);
  
  icuEnableNotifications(&ICUD2);
  icuEnableNotifications(&ICUD3);
  
//  board_power_set(0);
//  board_power_set(1);
//  board_power_clr(0);
//  board_power_clr(1);
//
//  board_power_set(0);
//  board_power_set(1);
//  board_power_clr(0);
//  board_power_clr(1);
  runTime.active = 0;
  runTime.heartBeatLost = 0;
  runTime.report_ms = 1000;
  
  runTime.source[0].id = 0;
  runTime.source[1].id = 1;
  runTime.source[0].enable = nvmParam.vsource[0].enable;
  runTime.source[1].enable = nvmParam.vsource[1].enable;
  runTime.source[0].current_limit = nvmParam.vsource[0].current_limit;
  runTime.source[1].current_limit = nvmParam.vsource[1].current_limit;

  runTime.canThread = chThdCreateStatic(waCANBUS, sizeof(waCANBUS), NORMALPRIO, procCANBUS, &CAND1);
  //runTime.canThread2 = chThdCreateStatic(waCANBUS2, sizeof(waCANBUS2), NORMALPRIO, procCANBUS, &CAND2);
  runTime.adsThread = chThdCreateStatic(waADC, sizeof(waADC), NORMALPRIO, procADC, NULL);
  
  chThdCreateStatic(waVS1, sizeof(waVS1), NORMALPRIO, procVS, &runTime.source[0]);
  //chThdCreateStatic(waVS2, sizeof(waVS2), NORMALPRIO, procVS, &runTime.source[1]);
  //cmuMgmtInit();
  
//  bmu_ltc_init();
//  balInit();

  while(1){
    chThdSleepMilliseconds(500);
  }
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
    if(palReadPad(GPIOA,12) == PAL_LOW){
      //nvmParam.boardID = rxMsg.data8[0];
      //nvm_set_block(PG_BOARD,(uint8_t*)&nvmBoard);
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
        nvmLoadDefault();
      }
    }
  }
  return 0;
}
int8_t digital_input_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(prx->RTR == CAN_RTR_REMOTE){ // read digital input
    ptx->data8[0] = 0;
    for(uint8_t i=0;i<2;i++){
      ptx->data8[0] |= (board_input_state(i) << i);
    }
    ptx->RTR = CAN_RTR_DATA;
    ptx->DLC = 1;
    return 1;
  }
  else{
    
  }
  return 0;
}
int8_t digital_output_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(ptx->RTR == CAN_RTR_REMOTE){
    ptx->data8[0] = 0;
    for(uint8_t i=0;i<2;i++){
      ptx->data8[0] |= (board_output_state(i) << i);
    }
    ptx->RTR = CAN_RTR_DATA;
    ptx->DLC = 1;
    return 1;
  }
  else{
    uint8_t cmd = prx->data8[0];
    uint8_t d = prx->data8[1];
    switch(cmd){
      case 0: // output control
        if(d & 0x01){
          board_relay_set(0);
        }
        else{
          board_relay_clr(0);
        }
        if(d & 0x02){
          board_relay_set(1);
        }
        else{
          board_relay_clr(1);
        }
        break;
      case 1: // set default value
        nvmParam.default_digit_out = prx->data8[1];
        break;
      default:break;
    }
  }
  return 0;
}
int8_t analog_input_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(prx->RTR == CAN_RTR_REMOTE){
    uint8_t idx = prx->EID & 0xFF;
    //if(prx->DLC == 0){
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
      case 2:     // pwm duty
        ptx->data16[0] = runTime.pwmInDuty[0];
        ptx->data16[1] = runTime.pwmInDuty[1];
        ptx->DLC = 4;
        break;
      }
//    }
//    else if(prx->DLC == 1){
//      
//    }
    return 1;
  }
  else{ // adc channel config
    uint8_t ch = prx->data8[0];
    uint8_t opt = prx->data8[1];  
    if(ch < 8){
      switch(opt){
      case 0: // raw low;
        nvmParam.adc_transfer[ch].raw_low = prx->data16[1];
        break;
      case 1: // raw_high
        nvmParam.adc_transfer[ch].raw_high = prx->data16[1];
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
  return 0; 
}
int8_t power_output_handler(CANRxFrame *prx,CANTxFrame *ptx)
{
  if(ptx->RTR == CAN_RTR_REMOTE){
    // read maximum current
    uint8_t ch = prx->data8[0];
    if(ch < 2){
      ptx->DLC = 4;
      ptx->data16[0] = ch;
      ptx->data16[1] = runTime.source[ch].current_limit;
    }
  }
  else{
    uint8_t ch = prx->data8[0];
    if(ch < 2){
      nvmParam.vsource[ch].enable = prx->data8[1]==0?0:1;
      nvmParam.vsource[ch].current_limit = prx->data16[1];
      runTime.source[ch].enable = nvmParam.vsource[ch].enable;
      runTime.source[ch].current_limit = nvmParam.vsource[ch].current_limit;
      
     // runTime.source[ch].enable = nvmParam.vsource[ch].enable;
//      if(runTime.source[ch].enable == 1){
//        board_power_set(ch);
//      }
//      else{
//        board_power_clr(ch);
//      }
    }
  }
}

int8_t heartBeatHandler(CANRxFrame *prx,CANTxFrame *ptx)
{
  runTime.heartBeatLost = 0;
  if(runTime.active == 0){
    runTime.active = 1;
    runTime.restartSource = 1;
  }
  return 0; 
}

