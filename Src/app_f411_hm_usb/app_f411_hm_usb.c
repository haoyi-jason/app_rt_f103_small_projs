#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "shell.h"
#include "chprintf.h"
#include "adcmxl3021.h"


#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

static SPIConfig spicfg = {
  false,
  NULL,
  NULL,
  0,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

static ADXLSPIConfig adcmxl_config = {
  &SPID2,
  &spicfg,
  GPIOB,
  12
};

static ADCMXL3021Driver adcmxl;

/*
  write register, usage:
  write xx yy
  xx: register address, use 0x for hex format
  yy: register value to write, use 0x for hex format
*/
static void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 2){
    uint8_t addr;
    uint16_t value;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint8_t)strtol(argv[0],16);
    }
    else{
      addr = (uint8_t)strtol(argv[0],10);
    }
    
    if(strncmp(argv[1],"0x",2) == 0){
      value = (uint16_t)strtol(argv[1],16);
    }
    else{
      value = (uint16_t)strtol(argv[1],10);
    }
    
    // todo write register
    adcmxWriteRegister(&adcmxl,addr,&value);
  }
  //chprintf(chp,"\r\nWrite Command");
}

uint16_t swap_16(uint8_t *in)
{
  uint16_t ret;
  uint8_t *s = (uint8_t *)&ret;
  s[1] = in[0];
  s[0] = in[1];
  return ret;
}

/*
  read: read register, usage
  read aa,
  aa: register address
*/
static void cmd_read(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  
  if(argc == 2){
    uint8_t addr;
    uint16_t n;
    uint16_t value;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint8_t)strtol(argv[0],16);
    }
    else{
      addr = (uint8_t)strtol(argv[0],10);
    }
        
    if(strncmp(argv[1],"0x",2) == 0){
      n = (uint16_t)strtol(argv[1],16);
    }
    else{
      n = (uint16_t)strtol(argv[1],10);
    }

    // todo read register
    uint8_t tx[2];
    tx[0] = addr;
    uint16_t *rx = chHeapAlloc(NULL,n*2);
    addr = (addr >> 1) << 1;
    
    if(rx != NULL){
      adcmxReadRegister(&adcmxl,addr,n,rx);
      
      for(uint8_t i=0;i<n;i++){
        chprintf(chp,"%02x = %04x\r\n",(addr+i)<<1,swap_16((uint8_t*)&rx[i]));
      }
    }
    chHeapFree(rx);
    
    
  }
}

static void cmd_page(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  
  if(argc == 1){
    uint8_t page;
    if(strncmp(argv[0],"0x",2) == 0){
      page = (uint8_t)strtol(argv[0],16);
    }
    else{
      page = (uint8_t)strtol(argv[0],10);
    }    
    adcmxSetPage(&adcmxl,page);
    
//    chprintf(chp,"%x = %x-%x\r\n",addr,rx[0],rx[1]);
    
  }
}

static void cmd_run(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  
  chprintf(chp,"\r\nRun Command");
}

static const ShellCommand commands[] = {
  {"write", cmd_write},
  {"read", cmd_read},
  {"page", cmd_page},
  {"run", cmd_run},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};




  

//static SerialConfig serialcfg = {
//  115200,
//};
//
//static CANConfig canCfg500K = {
//  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
//  CAN_BTR_BRP(6) | CAN_BTR_TS1(8) | CAN_BTR_TS2(1) | CAN_BTR_SJW(0)
//};
//
//// can config for 250K baud
//static CANConfig canCfg250K = {
//  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
//  CAN_BTR_BRP(20) | CAN_BTR_TS1(5) | CAN_BTR_TS2(0) | CAN_BTR_SJW(0)
//};


static THD_WORKING_AREA(waBlink, 1024);
static THD_FUNCTION(procBlink, arg) 
{
  (void)arg;
  
  while(1){
    palSetPad(GPIOC,2);
    chThdSleepMilliseconds(500);
    palClearPad(GPIOC,2);
    chThdSleepMilliseconds(500);
  }
}

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

//void serialTest()
//{
//  
//  sdStart(&SD1,&serialcfg);
//  for(uint8_t i=0;i<10;i++){
//    sdWrite(&SD1,"Hello\n",5);
//    chThdSleepMilliseconds(100);
//  }
//
//}

//typedef struct{
//  uint32_t pgn;
//  uint8_t buffer[8];
//  uint8_t len;
//  uint8_t dst;
//  uint8_t src;
//  uint8_t prio;
//}j1939_msg_t;
//
//static msg_t canSend(CANDriver *p, j1939_msg_t *m)
//{
//  CANTxFrame ptx;
//  uint8_t i;
//  
//  ptx.EID = (m->prio << 26) | (m->pgn << 8) | m->src;
//  ptx.IDE = CAN_IDE_EXT;
//  ptx.RTR = CAN_RTR_DATA;
//  ptx.DLC = m->len;
//  memcpy(ptx.data8,m->buffer,m->len);
//  
//  return canTransmit(p,CAN_ANY_MAILBOX,&ptx,TIME_MS2I(100));
//}

//void canTest()
//{
//  canStart(&CAND1,&canCfg500K);
//  //canStart(&CAND2,&canCfg500K);
//
//  j1939_msg_t msg;
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
//  
//  canSend(&CAND1,&msg);
//  //canSend(&CAND2,&msg);
//}

static THD_WORKING_AREA(waShell,2048);
static THD_WORKING_AREA(waReader,256);
static THD_FUNCTION(procUSBReader,arg)
{
  (void)arg;
  while(true){
    
  }
}


void main(void)
{
  thread_t *shelltp1 = NULL;
  halInit();
  chSysInit();
  
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  
  
  usbDisconnectBus(serusbcfg.usbp);
  palClearPad(GPIOA,15);
  chThdSleepMilliseconds(1000);
  palSetPad(GPIOA,15);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);  
//  palClearPad(GPIOB,12);
//  palSetPad(GPIOB,12);
//  palClearPad(GPIOB,12);
//  palSetPad(GPIOB,12);
  //spiTest();
//  serialTest();
  //canTest();
//  chThdCreateStatic(waBlink, sizeof(waBlink), NORMALPRIO, procBlink, NULL);
 shellInit();
 
 adcmxlObjectInit(&adcmxl);
 adcmxlStart(&adcmxl,&adcmxl_config);
 
// uint8_t tx[4];
// uint16_t rx[2];
// adcmxSetPage(&adcmxl,1);
// for(uint8_t i=0;i<0x7f;i+=2){
//   tx[0] = i;
//   tx[1] = 0x00;
//   adcmxRegisterXfer(&adcmxl,tx,2,(uint8_t*)rx,2);
//   chThdSleepMilliseconds(50);
// }
  
  while(1){
    if (SDU1.config->usbp->state == USB_ACTIVE) {
      /* Starting shells.*/
      if (shelltp1 == NULL) {
        shelltp1 = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                       "shell1", NORMALPRIO + 1,
                                       shellThread, (void *)&shell_cfg1);
//        shelltp1 = chThdCreateStatic(waShell, sizeof(waShell),NORMALPRIO+1,shellThread,(void*)&shell_cfg1);
      }

      /* Waiting for an exit event then freeing terminated shells.*/
      chEvtWaitAny(EVENT_MASK(0));
      if (chThdTerminatedX(shelltp1)) {
        chThdRelease(shelltp1);
        shelltp1 = NULL;
      }
    }
    else {
      chThdSleepMilliseconds(200);
    }
  }
}