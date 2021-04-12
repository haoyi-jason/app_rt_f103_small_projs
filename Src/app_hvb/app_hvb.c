#include "ch.h"
#include "hal.h"
#include "at24_eep.h"
#include <string.h>
#include "ads1015.h"
#include "at32_flash.h"


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
  ADS1015_ADDR_GND
};

struct{
  uint8_t id;
  uint16_t timer;
  uint16_t voltage;
  uint16_t current;
  thread_t *canThread;
  thread_t *adsThread;
}runTime;

struct{
  uint32_t flag;
  uint8_t devID;
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
  flash_Write(FLASH_START_ADDRESS,(uint16_t*)&nvmParam,sizeof(nvmParam)/2);
}

void nvmFlashInit()
{
  nvmRead(); // load nvm
  if(nvmParam.flag != 0x53290921){
    // load default
    nvmParam.flag = 0x53290921;
    nvmParam.devID = 0x01;
    
    nvmWrite();
  }
}

static thread_t *canThread;
static thread_t *adThread;

static virtual_timer_t vtReport;
static uint16_t reportMS;
static void report_cb(void *arg)
{
  chSysLockFromISR();
  chEvtSignalI(canThread, EVENT_MASK(1));
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
  
  bool bRun = true;
  while(bRun){
    
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));
    if(evt & 0x01){
      while(canReceive(ip,CAN_ANY_MAILBOX,&rxMsg,TIME_IMMEDIATE) == MSG_OK){
        uint16_t eidActive = rxMsg.EID & 0xFFF;
        uint8_t dest = (rxMsg.EID >> 12) & 0xFF;
        if(eidActive == 0x131){ // report speed
          
        }
        else if(eidActive == 0x99){ // set id
          
        }
        else if(eidActive == 0x140){ // set params
          
        }
      }
    }
    if(evt & 0x02){
      // report A/D data, stack voltage & stack current
      // voltage: 16-bit unsigned, in unit of 0.1 v
      // current: 16-bit unsigned, in unit of 0.1 A
      // other 2 16-bit data for temperature
      ptx.EID = runTime.id << 12;
      ptx.EID |= 0x111; // todo: define message id
      chSysLock();
      ptx.data16[0] = runTime.voltage;
      ptx.data16[1] = runTime.current;
      canTransmit(ip, CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
    }
    
    
    
    if(chThdShouldTerminateX()){
      bRun = false;
    }
    
    
  }
  
};

static THD_WORKING_AREA(waADS1115,1024);
static THD_FUNCTION(procADS1115,p){
  (void)p;
  uint16_t volts[8];
  uint16_t amps[8];
  ads1015_set_pga(&ads1115,PGA_4_096);
  ads1015_set_mode(&ads1115,MODE_SINGLE);
  uint8_t voltIndex = 0, ampIndex = 0;
  bool bRun = true;
  uint8_t state = 0;
  uint16_t voltSum,ampSum;
  while(bRun){
    switch(state){
    case 0:
      ads1015_set_mux(&ads1115,MUX_AIN0_GND);
      ads1015_start_conversion(&ads1115);
      state++;
      break;
    case 1:
      volts[voltIndex++] = ads1015_read_data(&ads1115);
      if(voltIndex == 8) voltIndex = 0;
      state++;
      break;
    case 2:
      ads1015_set_mux(&ads1115,MUX_AIN1_GND);
      ads1015_start_conversion(&ads1115);
      state++;
      break;
    case 3:
      amps[ampIndex++] = ads1015_read_data(&ads1115);
      if(ampIndex == 8) ampIndex = 0;
      state++;
      break;
    case 4:
      voltSum = ampSum = 0;
      for(uint8_t i=0;i<8;i++){
        voltSum += volts[i];
        ampSum += amps[i];
      }
      runTime.voltage = voltSum / 8;
      runTime.current = ampSum / 8;
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
  AFIO->MAP5 |= AFIO_MAP5_I2C2_GRMP_10;
  AFIO->MAP6 |= AFIO_MAP6_CAN1_GRMP;
  AFIO->MAP6 |= AFIO_MAP6_CAN2_GRMP;
  
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