#include "ch.h"
#include "hal.h"

static SPIConfig spicfg = {
  false,
  NULL,
  GPIOB,
  12,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

static SerialConfig serialcfg = {
  115200,
};

static CANConfig canCfg500K = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_BRP(6) | CAN_BTR_TS1(8) | CAN_BTR_TS2(1) | CAN_BTR_SJW(0)
};

// can config for 250K baud
static CANConfig canCfg250K = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_BRP(20) | CAN_BTR_TS1(5) | CAN_BTR_TS2(0) | CAN_BTR_SJW(0)
};


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

void serialTest()
{
  
  sdStart(&SD1,&serialcfg);
  for(uint8_t i=0;i<10;i++){
    sdWrite(&SD1,"Hello\n",5);
    chThdSleepMilliseconds(100);
  }

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
  canStart(&CAND1,&canCfg500K);
  //canStart(&CAND2,&canCfg500K);

  j1939_msg_t msg;
  msg.dst = 0x1234;
  msg.pgn = 100;
  msg.src =  0x5678;
  msg.buffer[0] = 0x11;
  msg.buffer[1] = 0x22;
  msg.buffer[2] = 0x33;
  msg.buffer[3] = 0x44;
  msg.buffer[4] = 0x55;
  msg.buffer[5] = 0x66;
  msg.buffer[6] = 0x77;
  msg.buffer[7] = 0x88;
  msg.len = 8;
  
  canSend(&CAND1,&msg);
  //canSend(&CAND2,&msg);
}



void main(void)
{
  halInit();
  chSysInit();
  
//  palClearPad(GPIOB,12);
//  palSetPad(GPIOB,12);
//  palClearPad(GPIOB,12);
//  palSetPad(GPIOB,12);
  //spiTest();
//  serialTest();
  canTest();
  chThdCreateStatic(waBlink, sizeof(waBlink), NORMALPRIO, procBlink, NULL);
 
  
  while(1){
    chThdSleepMilliseconds(500);
  }
}