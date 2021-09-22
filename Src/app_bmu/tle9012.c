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

uint8_t crc8_msb(uint8_t poly,uint8_t *data, uint8_t sz)
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

uint8_t u8_reverse(uint8_t v)
{
  //return v;
  uint8_t tmp = v;
  uint8_t vv = v;
  vv >>= 1;
  uint8_t cnt = 7;
  while(vv){
    tmp <<= 1;
    tmp |= (vv & 0x1);
    vv >>= 1;
    cnt--;
  }
  tmp <<= cnt;
  return tmp;
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


uint8_t tle9012_writeRegister(TLE9012Driver *dev, uint8_t id, uint8_t reg, uint16_t value)
{
  uint8_t ret = 0xff;
  uint8_t txBuf[6]={0x1E,0x80,0x36,0x00,0x01,0x78};
  txBuf[1] = id | 0x80;
  txBuf[2] = reg;
  txBuf[3] = (value >> 8) & 0xff;
  txBuf[4] = value & 0xff;
  txBuf[5] = crc8_msb(CRC_POLY,txBuf,5);
  
  for(uint8_t i=0;i<6;i++){
    txBuf[i] = u8_reverse(txBuf[i]);
  }
  
  uint8_t rx[16];
  int8_t nrx = dev->registerXfer(dev,txBuf,6,rx,16);
  for(uint8_t i=0;i<nrx;i++){
    rx[i] = u8_reverse(rx[i]);
  }
  if(nrx == 7){
    ret = rx[6];
  }
  
  return ret;
}

//static TLE9012_ERR tle9012_readRegister(TLE9012Driver *dev,uint8_t id, uint8_t reg,uint8_t sz, uint8_t *value, uint8_t *rxCnt)
TLE9012_ERR tle9012_readRegister(TLE9012Driver *dev,uint8_t id, uint8_t reg,uint8_t sz, uint8_t *value)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t txBuf[6]={0x1E,0x00,0x36,0x00,0x01,0x78};
  txBuf[1] = id;
  txBuf[2] = reg;
  txBuf[3] = crc8_msb(CRC_POLY,txBuf,3);

  // LSB->MSB
  for(uint8_t i=0;i<4;i++){
    txBuf[i] = u8_reverse(txBuf[i]);
  }
  uint8_t rxBuf[16];
  int8_t nrx = dev->registerXfer(dev,txBuf,4,rxBuf,16)-4;
//  for(uint8_t i=0;i<nrx;i++){
//    value[i] = u8_reverse(rxBuf[i]);
//  }
  if(nrx != sz){ // wrong read size
    err = TLE9012_ERR_COMM; 
  }
  else{
    value[1] = u8_reverse(rxBuf[6]);
    value[0] = u8_reverse(rxBuf[7]);
  }
  
  return err;
}

TLE9012_ERR tle9012_readRegisterMulti(TLE9012Driver *dev,uint8_t id, uint8_t reg,uint8_t sz, uint8_t *value)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t txBuf[6]={0x1E,0x00,0x36,0x00,0x01,0x78};
  txBuf[1] = id;
  txBuf[2] = reg;
  txBuf[3] = crc8_msb(CRC_POLY,txBuf,3);

  // LSB->MSB
  for(uint8_t i=0;i<4;i++){
    txBuf[i] = u8_reverse(txBuf[i]);
  }
  //uint8_t rxBuf[128];
  int8_t nrx = dev->registerXfer(dev,txBuf,4,value,128);
//  for(uint8_t i=0;i<nrx;i++){
//    value[i] = u8_reverse(rxBuf[i]);
//  }
  for(uint8_t i=0;i<nrx;i++){
    
    value[i] = u8_reverse(value[i]);
  }
  if(nrx != sz){ // wrong read size
    err = TLE9012_ERR_COMM; 
  }
  
  return err;
}



TLE9012_ERR tle9012_setID(TLE9012Driver *dev, uint8_t id)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint16_t reg;
  err = tle9012_writeRegister(dev,0x80,0x36,id | 0x0800);
  return err;
}

TLE9012_ERR tle9012_enableCellMonitor(TLE9012Driver *dev, uint16_t mask)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t ret;
  uint8_t rxCnt;
  uint16_t reg;
  uint8_t rx[16];
  ret = tle9012_writeRegister(dev,0x0,0x01,mask);
  tle9012_readRegister(dev,0x0,0x01,16,rx);
  if(ret == 0xff){
    err = TLE9012_ERR_COMM;
  }
  else{
    if((ret & 0x08) == 0x08){
      uint16_t diag;
      //ret = tle9012_ReadDiag(dev);
      err = TLE9012_ERR_WDIAG;
    }
    if((ret & 0x10) == 0x10){
      err = TLE9012_ERR_WADDR;
    }
    if((ret & 0x20) == 0x20){
      err = TLE9012_ERR_WCMD;
    }
  }
  return err;

}

TLE9012_ERR tle9012_enableNTC(TLE9012Driver *dev, uint8_t nofNTC)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t txBuf[6]={0x1E,0x81,0x04,0x50,0x00,0x00};
//  txBuf[3] = (mask >> 8) & 0xff;
//  txBuf[4] = mask & 0xff;
  txBuf[5] = crc8_msb(CRC_POLY,txBuf,5);
  
 // dev->registerWrite(dev,txBuf,6,10);
  return err;
}

TLE9012_ERR tle9012_balanceControl(TLE9012Driver *dev, uint16_t mask)
{
  TLE9012_ERR err = TLE9012_NOERR;
  uint8_t txBuf[6]={0x1E,0x81,REG_BAL_SETTING,0x00,0x00,0x00};
  txBuf[3] = (mask >> 8) & 0x0f;
  txBuf[4] = mask & 0xff;
  txBuf[5] = crc8_msb(CRC_POLY,txBuf,5);
  
  //dev->registerWrite(dev,txBuf,6,10);

  return err;
}

TLE9012_ERR tle9012_startVMeas(TLE9012Driver *dev)
{
  dev->meas_ctrl = MEAS_CTRL_START | MEAS_CTRL_BITWIDTH(MWAS_CTRL_BIT16) | MEAS_CTRL_START_BVM | MEAS_CTRL_BVMBITWIDTH(MWAS_CTRL_BIT16);
  return TLE9012_WRITE_MEAS_CTRL(dev);
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
  
  //dev->registerWrite(dev,txBuf,4,10);
  //dev->registerRead(dev,rxBuf,5,10);
  
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
  
  //dev->registerWrite(dev,txBuf,6,10);
  
  return err;
}

void tle9012_wakeup(TLE9012Driver *dev)
{
 
  uint8_t txBuf[] = {0x55,0x55};
  dev->registerXfer(dev,txBuf,2,NULL,0);
  // 350 us delay
  uint16_t n=3000;
  while(--n)
    __NOP();
}

void tle9012_clean(TLE9012Driver *dev)
{

}

void tle9012_feedWDT(TLE9012Driver *dev, uint8_t count)
{
  TLE9012_READ_WDG_CNT(dev);
  dev->wdg_cnt.write = dev->wdg_cnt.read | 0xFFE0;
  dev->wdg_cnt.write |= count;
  TLE9012_WRITE_WDG_CNT(dev);
}


TLE9012_ERR tle9012_init(TLE9012Driver *dev)
{
  TLE9012_ERR err = TLE9012_NOERR;
  if(dev->state != TLE9012_UNINIT){
    return TLE9012_ERR_INIT;
  }
  
  // wakeup 
  tle9012_wakeup(dev);
  // initial config
  TLE9012_WRITE_PART_CONFIG(dev);
  TLE9012_WRITE_TEMP_CONF(dev);
  TLE9012_WRITE_MULTI_READ_CFG(dev);
  TLE9012_WRITE_OPMODE(dev);
  
  
  return err;
}
