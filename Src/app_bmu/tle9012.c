#include "hal.h"
#include "tle9012.h"
#include "tle9012_def.h"

#define CRC_POLY        0x2F
#define CRC_INIT        0x00
#define CRC_XOROUT      0x00

static const SerialConfig sercfg = {
  2000000,
};

static const SerialConfig sercfg_wkup = {
  55000,
};

static const uint8_t tle_meas_cell_bd[6] = {0x1E,0xBF,0x18,0xE0,0x21,0xB2};
static const uint8_t tle_meas_block_bd[6] = {0x1E,0xBF,0x18,0xE0,0x21,0xB2};

static uint8_t crc8_msb(uint8_t poly,uint8_t *data, uint8_t sz)
{
  uint8_t crc = CRC_INIT;
  uint8_t bit;
  
  while(sz--){
    crc ^= *data++;
    for(bit = 0; bit < 8; bit++){
      if(crc & 0x80)
        crc = (crc << 1) ^poly;
      else
        crc <<= 1;
    }
  }
  return crc;
}

//static uint8_t crc8_lsb(uint8_t poly, uint8_t *data, uint8_t sz)
//{
//  uint8_t crc = CRC_INIT;
//  uint8_t bit;
//  while(sz--){
//    crc ^= *data++;
//    for(bit = 0; bit < 8; bit++){
//      if(crc & 0x01)
//        crc = (crc >> 1)^poly;
//      else
//        crc >>= 1;
//    }
//  }
//  return crc;
//}


static TLE9012_ERR tle9012_writeRegister(TLE9012Driver *dev, uint8_t reg, uint16_t value)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t txBuf[6]={0x1E,0x80,0x36,0x00,0x01,0x78};
  txBuf[1] |= dev->id;
  txBuf[1] |= 63;
  txBuf[2] = reg;
  txBuf[3] = (value >> 8) & 0xff;
  txBuf[4] = value & 0xff;
  txBuf[5] = crc8_msb(CRC_POLY,txBuf,5);
  
  dev->registerWrite(dev,txBuf,6,10);
  chThdSleepMilliseconds(100);
  uint8_t rxBuf[12];
  dev->registerRead(dev,rxBuf,7,10);
  
  return err;
}

static TLE9012_ERR tle9012_readRegister(TLE9012Driver *dev, uint8_t reg, uint16_t *value)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t txBuf[6]={0x1E,0x00,0x36,0x00,0x01,0x78};
  txBuf[1] |= dev->id;
  txBuf[1] |= 63;
  txBuf[2] = reg;
  txBuf[3] = crc8_msb(CRC_POLY,txBuf,3);
  
  dev->registerWrite(dev,txBuf,4,10);
  chThdSleepMilliseconds(100);
  uint8_t rxBuf[12];
  dev->registerRead(dev,rxBuf,12,10);
  
  return err;
}

TLE9012_ERR tle9012_setID(TLE9012Driver *dev, uint8_t id)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t txBuf[6]={0x1E,0x80,0x36,0x00,0x01,0x78};
  txBuf[3] = 0x08; // final node
  txBuf[4] = id;
  txBuf[5] = crc8_msb(CRC_POLY,txBuf,5);
  
  dev->registerWrite(dev,txBuf,6,10);
  chThdSleepMilliseconds(100);
  //uint8_t rxBuf[12];
  //dev->registerRead(dev,rxBuf,6,10);
  
  return err;
}

TLE9012_ERR tle9012_enableCellMonitor(TLE9012Driver *dev, uint16_t mask)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t txBuf[6]={0x1E,0x81,0x00,0x00,0x00,0x00};
  txBuf[3] = (mask >> 8) & 0xff;
  txBuf[4] = mask & 0xff;
  txBuf[5] = crc8_msb(CRC_POLY,txBuf,5);
  
  dev->registerWrite(dev,txBuf,6,10);
  return err;
}

TLE9012_ERR tle9012_enableNTC(TLE9012Driver *dev, uint8_t nofNTC)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t txBuf[6]={0x1E,0x81,0x04,0x50,0x00,0x00};
//  txBuf[3] = (mask >> 8) & 0xff;
//  txBuf[4] = mask & 0xff;
  txBuf[5] = crc8_msb(CRC_POLY,txBuf,5);
  
  dev->registerWrite(dev,txBuf,6,10);
  return err;
}

TLE9012_ERR tle9012_balanceControl(TLE9012Driver *dev, uint16_t mask)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t txBuf[6]={0x1E,0x81,REG_BAL_SETTING,0x00,0x00,0x00};
  txBuf[3] = (mask >> 8) & 0x0f;
  txBuf[4] = mask & 0xff;
  txBuf[5] = crc8_msb(CRC_POLY,txBuf,5);
  
  dev->registerWrite(dev,txBuf,6,10);

  return err;
}

TLE9012_ERR tle9012_startVMeas(TLE9012Driver *dev, uint16_t meas_ctrl)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t txBuf[6]={TLC_FRAME_SYNC,TLC_ID_WR_MASK | 0x01,REG_MEAS_CTRL,0xE0,0x21,0xDC};
  txBuf[1] = TLC_ID_WR_MASK | dev->id;
  txBuf[2] = REG_MEAS_CTRL;
  txBuf[3] = (meas_ctrl >> 8) & 0xff;
  txBuf[4] = meas_ctrl & 0xff;
  txBuf[5] = crc8_msb(CRC_POLY,txBuf,5);
  
  dev->registerWrite(dev,txBuf,6,10);
  return err;
}

TLE9012_ERR tle9012_startGPIOVmeas(TLE9012Driver *dev)
{
  TLE9012_ERR err = TLE9012_NOERR;

  return err;
}

TLE9012_ERR tle9012_startOpenWireMeas(TLE9012Driver *dev)
{
  TLE9012_ERR err = TLE9012_NOERR;

  return err;
}

TLE9012_ERR tle9012_RX(TLE9012Driver *dev, uint8_t addr, uint8_t *d)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t txBuf[4]={0x1E,0x01,0x00,0x00};
  uint8_t rxBuf[5];
  txBuf[1] = dev->id;
  txBuf[2] = addr;
  txBuf[3] = crc8_msb(CRC_POLY,txBuf,3);
  
  dev->registerWrite(dev,txBuf,4,10);
  dev->registerRead(dev,rxBuf,5,10);
  
  // crc check

  return err;
}

TLE9012_ERR tle9012_readVoltage(TLE9012Driver *dev, uint16_t *volt)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t rxBuf[5];
  err = tle9012_RX(dev,REG_MULTI_READ,rxBuf);
  
  return err;
}

TLE9012_ERR tle9012_readGPIOVoltage(TLE9012Driver *dev,uint16_t *volt)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t rxBuf[5];
  err = tle9012_RX(dev,REG_MULTI_READ,rxBuf);
  
  return err;
}

TLE9012_ERR tle9012_configMultiRead(TLE9012Driver *dev, uint16_t mask)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t txBuf[6]={0x1E,0x81,REG_MULTI_READ_CFG,0x00,0x00,0x00};
  txBuf[1] = TLC_ID_WR_MASK | dev->id;
  txBuf[3] = (mask >> 8) & 0x0f;
  txBuf[4] = mask & 0xff;
  txBuf[5] = crc8_msb(CRC_POLY,txBuf,5);
  
  dev->registerWrite(dev,txBuf,6,10);
  
  return err;
}


void tle9012_wakeup(TLE9012Driver *dev)
{
  // config TX to output mode
  palSetPadMode(GPIOA,9,PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(GPIOA,9);
  //chThdSleepMilliseconds(1);
  
  for(uint8_t i=0;i<16;i++){
    uint8_t n=80;
    while(--n)
      __NOP();
    palTogglePad(GPIOA,9);
  }

  uint8_t n=40;
  
  while(--n){
    uint8_t m=200;
    while(--m)
      __NOP();
  }
  palSetPadMode(GPIOA,9,PAL_MODE_STM32_ALTERNATE_PUSHPULL);
}


TLE9012_ERR tle9012_init(TLE9012Driver *dev)
{
  TLE9012_ERR err = TLE9012_NOERR;
  if(dev->state != TLE9012_UNINIT){
    return TLE9021_ERR;
  }
  
  
  tle9012_wakeup(dev);
  
  
  sdStart(dev->config->devp,dev->config->config);

  tle9012_setID(dev,dev->id);

  uint16_t tmp16;
  tle9012_writeRegister(dev,REG_PART_CONFIG,0xfff);
  tle9012_readRegister(dev,REG_PART_CONFIG,&tmp16);
  
  return err;
}
