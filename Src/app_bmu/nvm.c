#include "ch.h"
#include "hal.h"
#include <string.h>
#include "nvm.h"
#include "at24_eep.h"
#include "ntc.h"

static I2CDriver *dev = &I2CD2;

const _nvm_board_s nvmBoardDefault = {
  .id = 0x01,
  .boardId = BOARD_ID,
  .flag = FG_NORMAL,
  .cellQueue = {1,1,1,1,0,0,1,1,1,1,0,0},     // 8-cell BMU
  .cellVoltReportNormal = 1000,  
  .cellVoltReportFast = 100,
  .cellTempReportNormal = 5000,
  .cellTempReportFast= 1000,
};

const _nvm_balance_cfg_s nvmBalanceDefault = {
  .enableBalance = 1,
  .balanceVoltMv = 3350,
  .onTime = 60,
  .offTime = 60,
  .balanceHystersisMv = 8,
};

const _nvm_ntc_cfg_s nvmNtcDefault = {
  .beta = {3950,3950,3950,3950,3950},
  .resistance = {10,10,10,10,10}, // in unit of k
  .betaTemp = {25,25,25,25,25},
};


static _nvm_board_s nvmBoard;
static _nvm_balance_cfg_s nvmBalance;
static _nvm_ntc_cfg_s nvmNtc;



_runtime_info_s runTime;

/********** header **********/
static _block_header_s nvmHeaderDefault[] = {
  {(void*)&nvmBoardDefault,sizeof(_nvm_board_s)},
  {(void*)&nvmBalanceDefault,sizeof(_nvm_balance_cfg_s)},
  {(void*)&nvmNtcDefault,sizeof(_nvm_ntc_cfg_s)},
};

static _block_header_s nvmHeader[] = {
  {(void*)&nvmBoard,sizeof(_nvm_board_s)},
  {(void*)&nvmBalance,sizeof(_nvm_balance_cfg_s)},
  {(void*)&nvmNtc,sizeof(_nvm_ntc_cfg_s)},
};


msg_t nvmReadPage(uint8_t page, uint8_t *d)
{
  return eepromRead(page * NVM_PAGE_SIZE, NVM_PAGE_SIZE, d);
}

msg_t nvmWritePage(uint8_t page, uint8_t *d)
{
  return eepromWrite(page * NVM_PAGE_SIZE, NVM_PAGE_SIZE, d);
}

msg_t nvm_read_block(uint8_t id, uint8_t *dptr, uint16_t sz)
{
  uint8_t pg[NVM_PAGE_SIZE];
  if(id < PG_MAX){
    nvmReadPage(id,pg);
    memcpy(dptr,pg,sz);
    return MSG_OK;
  }
  return MSG_RESET;
}

msg_t nvm_write_block(uint8_t id, uint8_t *dptr, uint16_t sz)
{
  uint8_t pg[NVM_PAGE_SIZE];
  if(id < PG_MAX){
    memset(pg,0xff,NVM_PAGE_SIZE);
    memcpy(pg,dptr,sz);
    nvmWritePage(id,pg);
    return MSG_OK;
  }
  return MSG_RESET;
}

msg_t nvm_runtime_set_cellVoltage(uint16_t *volts)
{
  uint8_t used = 0;
  for(uint8_t i=0;i<NOF_MAX_CELL_PER_MODULE;i++){
    runTime.cells.voltages[i] = volts[i];
  }
  
  return MSG_OK;
}

msg_t nvm_runtime_get_cellVoltage(uint16_t *volts, uint8_t *n)
{
  uint8_t used = 0;
  for(uint8_t i=0;i<runTime.activeCells;i++){
    volts[i]=runTime.cells.voltages[i];
  }
  *n = runTime.activeCells;
  return MSG_OK;
}

msg_t nvm_runtime_get_cellVoltageQueued(uint16_t *volts, uint8_t *n)
{
  uint8_t used = 0;
  for(uint8_t i=0;i<NOF_MAX_CELL_PER_MODULE;i++){
    if(nvmBoard.cellQueue[i] == 1){
      volts[used++]=runTime.cells.voltages[i];
    }
  }
  *n = used;
  return MSG_OK;
}

msg_t nvm_runtime_set_gpioVoltage(uint16_t *volts)
{
  uint8_t used = 0;
  for(uint8_t i=0;i<NOF_MAX_AUXIO;i++){
    runTime.ntcTemp.gpioVolt[i] = volts[i];
    // calculate temperature 
    runTime.ntcTemp.temperature[i] = ntcCalculateTempD10(volts[i],3000,10000,nvmNtc.resistance[i],nvmNtc.beta[i],nvmNtc.betaTemp[i]);
  }
  return MSG_OK;
}
    
msg_t nvm_runtime_get_gpioVoltage(uint16_t *volts, uint8_t *n)
{
  for(uint8_t i=0;i<NOF_MAX_AUXIO;i++){
    volts[i] = runTime.ntcTemp.gpioVolt[i];
  }
  return MSG_OK;
}

msg_t nvm_runtime_get_temperature(int16_t *value, uint8_t *n)
{
  for(uint8_t i=0;i<NOF_MAX_AUXIO;i++){
    value[i] = runTime.ntcTemp.temperature[i];
  }
  return MSG_OK;
}

msg_t nvm_get_cellQueue(uint8_t *queue)
{
  for(uint8_t i=0;i<NOF_MAX_CELL_PER_MODULE;i++){
    queue[i] = nvmBoard.cellQueue[i];
  }
  return MSG_OK;
}

msg_t nvm_set_cellQueue(uint8_t *queue)
{
  uint8_t active = 0;
  for(uint8_t i=0;i<NOF_MAX_CELL_PER_MODULE;i++){
    nvmBoard.cellQueue[i] = queue[i];
    if(nvmBoard.cellQueue[i] == 1){
      active++;
    }
  }
  runTime.activeCells = active;
  nvm_write_block(PG_BOARD,nvmHeader[PG_BOARD].block,nvmHeader[PG_BOARD].sz);
  
  return MSG_OK;
}

msg_t nvm_get_bal_config(_nvm_balance_cfg_s *p)
{
  memcpy((void*)p,nvmHeader[PG_CMU_BC].block,nvmHeader[PG_CMU_BC].sz);
  return MSG_OK;
}

msg_t nvm_set_bal_config(_nvm_balance_cfg_s *p)
{
  memcpy(nvmHeader[PG_CMU_BC].block,p, nvmHeader[PG_CMU_BC].sz);
  // todo : write fo eeprom
  return MSG_OK;
}

msg_t nvm_get_block(uint8_t id, uint8_t *dptr)
{
  if(id < PG_MAX){
    memcpy(dptr,nvmHeader[id].block, nvmHeader[id].sz);
  }
  return MSG_OK;
}

msg_t nvm_set_block(uint8_t id, uint8_t *dptr)
{
  if(id < PG_MAX){
    memcpy(nvmHeader[id].block, dptr, nvmHeader[id].sz);
    nvm_write_block(id,nvmHeader[id].block,nvmHeader[id].sz);
  }
  return MSG_OK;
}

msg_t nvm_runtime_get_balancing(uint8_t *p)
{
  memcpy(p,runTime.cells.balancing,12);
  return MSG_OK;
}

msg_t nvm_runtime_set_balancing(uint8_t *p)
{
  memcpy(runTime.cells.balancing,p,12);
  chSysLock();
  runTime.cells.balMask[0] = runTime.cells.balMask[1] = 0x0;
  uint8_t i;
  for(i=0;i<8;i++){
    if(runTime.cells.balancing[i] == 1){
      runTime.cells.balMask[0] |= (1 << i);
    }
  }
  for(i=8;i<NOF_MAX_CELL_PER_MODULE;i++){
    if(runTime.cells.balancing[i] == 1){
      runTime.cells.balMask[1] |= (1 << (i-8));
    }
  }
  chSysUnlock();
  return MSG_OK;
}

msg_t nvm_runtime_get_balancingQueued(uint8_t *p)
{
  p[0] = p[1] = 0x0;
  uint8_t i;
  for(i=0;i<8;i++){
    if(runTime.cells.balancing[i] == 1){
      p[0] |= (1 << i);
    }
  }
  for(i=8;i<NOF_MAX_CELL_PER_MODULE;i++){
    if(runTime.cells.balancing[i] == 1){
      p[1] |= (1 << (i-8));
    }
  }
  return MSG_OK;
}

void nvmInit(I2CDriver *devp)
{
  dev = devp;
  at24eep_init(dev,32,1024,0x50,2);
  for(uint8_t i=0;i<PG_MAX;i++){
    nvm_read_block(i,nvmHeader[i].block,nvmHeader[i].sz);
  }
  
  if((nvmBoard.boardId != BOARD_ID) || (nvmBoard.flag == FG_DEFAULT)){
    for(uint8_t i=0;i<PG_MAX;i++){
      memcpy(nvmHeader[i].block,nvmHeaderDefault[i].block,nvmHeader[i].sz);
      nvm_write_block(i,nvmHeader[i].block,nvmHeader[i].sz);
    }
  }
  
  runTime.activeCells = 0;
  for(uint8_t i=0;i<NOF_MAX_CELL_PER_MODULE;i++){
    if(nvmBoard.cellQueue[i] == 1)
      runTime.activeCells++;
  }
  runTime.id = nvmBoard.id;
  
}


