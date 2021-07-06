#include "ch.h"
#include "hal.h"
#include <string.h>
#include "nvm.h"
#include "at24_eep.h"
#include "ntc.h"
#include "at32_flash.h"

static I2CDriver *dev = &I2CD2;
static thread_t *thisThd;
const _nvm_board_s nvmBoardDefault = {
  .id = 0x21,
  .boardId = BOARD_ID,
  .flag = NVM_FLAG,
  .cellQueue = {1,1,1,1,1,1,1,1,1,1,1,1},     // 12-cell BMU
  .cellVoltReportNormal = 1000,  
  .cellVoltReportFast = 100,
  .cellTempReportNormal = 5000,
  .cellTempReportFast= 1000,
};

const _nvm_balance_cfg_s nvmBalanceDefault = {
  .enableBalance = 1,
  .balanceVoltMv = 5000,
  .onTime = 30,
  .offTime = 30,
  .balanceHystersisMv = 8,
};

const _nvm_ntc_cfg_s nvmNtcDefault = {
  .beta = {3435,3435,3435,3435,3435},
  .resistance = {10,10,10,10,10}, // in unit of k
  .betaTemp = {25,25,25,25,25},
};

static struct{
  uint8_t State;
  uint8_t id;
}pendTask;

static _nvm_board_s nvmBoard;
static _nvm_balance_cfg_s nvmBalance;
static _nvm_ntc_cfg_s nvmNtc;

static _nvmParam nvmParam;

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

void nvmFlashReadPage(uint8_t page, uint8_t *d)
{
  flash_Read(FLASH_START_ADDRESS + page*NVM_PAGE_SIZE,(uint16_t*)d,NVM_PAGE_SIZE/2);
}

void nvmFlashWritePage(uint8_t page, uint8_t *d)
{
  chSysLock();
  flash_Write(FLASH_START_ADDRESS + page * NVM_PAGE_SIZE,(uint16_t*)d,NVM_PAGE_SIZE/2);
  chSysUnlock();
}


static msg_t nvmReadPage(uint8_t page, uint8_t *d)
{
  return eepromRead(page * NVM_PAGE_SIZE, NVM_PAGE_SIZE, d);
}

static msg_t nvmWritePage(uint8_t page, uint8_t *d)
{
  return eepromWrite(page * NVM_PAGE_SIZE, NVM_PAGE_SIZE, d);
}

static msg_t nvm_read_block(uint8_t id, uint8_t *dptr, uint16_t sz)
{
  uint8_t pg[NVM_PAGE_SIZE];
  if(id < PG_MAX){
    //chSysLock();
    nvmFlashReadPage(id,pg);
    //chSysUnlock();
    memcpy(dptr,pg,sz);
    return MSG_OK;
  }
  return MSG_RESET;
}

static msg_t nvm_flash_read_block(uint8_t id, uint8_t *dptr, uint16_t sz)
{
  uint8_t pg[NVM_PAGE_SIZE];
  if(id < PG_MAX){
    nvmFlashReadPage(id,pg);
    memcpy(dptr,pg,sz);
    return MSG_OK;
  }
  return MSG_RESET;
}

static msg_t nvm_write_block(uint8_t id, uint8_t *dptr, uint16_t sz)
{
  uint8_t pg[NVM_PAGE_SIZE];
  if(id < PG_MAX){
    memset(pg,0xff,NVM_PAGE_SIZE);
    memcpy(pg,dptr,sz);
    nvmFlashWritePage(id,pg);
    return MSG_OK;
  }
  return MSG_RESET;
}

static msg_t nvm_flash_write_block(uint8_t id, uint8_t *dptr, uint16_t sz)
{
  uint8_t pg[NVM_PAGE_SIZE];
  if(id < PG_MAX){
    memset(pg,0xff,NVM_PAGE_SIZE);
    memcpy(pg,dptr,sz);
    nvmFlashWritePage(id,pg);
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
  for(uint8_t i=0;i<NOF_MAX_CELL_PER_MODULE;i++){
    volts[i]=runTime.cells.voltages[i];
  }
  *n = NOF_MAX_CELL_PER_MODULE;
  return MSG_OK;
}

msg_t nvm_runtime_get_cellVoltageQueued(uint16_t *volts, uint8_t *n)
{
//  uint8_t used = 0;
//  for(uint8_t i=0;i<NOF_MAX_CELL_PER_MODULE;i++){
//    if(nvmBoard.cellQueue[i] == 1){
//      volts[used++]=runTime.cells.voltages[i];
//    }
//  }
//  *n = used;
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
//    queue[i] = nvmBoard.cellQueue[i];
    queue[i] = nvmBoard.cellQueue[i];
  }
  return MSG_OK;
}

msg_t nvm_get_cellQueueEx(uint16_t *queue)
{
  uint16_t q = 0x0;
  for(uint8_t i=0;i<NOF_MAX_CELL_PER_MODULE;i++){
//    if(nvmBoard.cellQueue[i]==1){
    if(nvmBoard.cellQueue[i]==1){
      q |= (1 << i);
    }
  }
  *queue = q;
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
  nvm_flash_write_block(PG_BOARD,nvmHeader[PG_BOARD].block,nvmHeader[PG_BOARD].sz);
//  if(pendTask.State == 0){
//    pendTask.State = 2;
//    pendTask.id = PG_BOARD;
//    chEvtSignal(thisThd, EVENT_MASK(0));
//  }
  return MSG_OK;
}

msg_t nvm_set_default(uint8_t id)
{
  if(pendTask.State == 0){
    pendTask.State = 4;
    chEvtSignal(thisThd, EVENT_MASK(0));
  }
  return MSG_OK;
}

msg_t nvm_set_cellQueueEx(uint8_t *queue)
{
  uint8_t active = 0;
  for(uint8_t i=0;i<8;i++){
    if((queue[0] & (1 << i)) == (1 << i)){
      nvmBoard.cellQueue[i] = 1;
      active++;
    }
    else{
      nvmBoard.cellQueue[i] = 0;
    }
  }
  for(uint8_t i=8;i<NOF_MAX_CELL_PER_MODULE;i++){
    if((queue[1] & (1 << (i-8))) == (1 << (i-8))){
      nvmBoard.cellQueue[i] = 1;
      active++;
    }
    else{
      nvmBoard.cellQueue[i] = 0;
    }
  }

  runTime.activeCells = active;
  nvm_flash_write_block(PG_BOARD,nvmHeader[PG_BOARD].block,nvmHeader[PG_BOARD].sz);
//  if(pendTask.State == 0){
//    pendTask.State = 2;
//    pendTask.id = PG_BOARD;
//    chEvtSignal(thisThd, EVENT_MASK(0));
//  }
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
  nvm_write_block(PG_CMU_BC,nvmHeader[PG_CMU_BC].block,nvmHeader[PG_CMU_BC].sz);
//  if(pendTask.State == 0){
//    pendTask.State = 2;
//    pendTask.id = PG_CMU_BC;
//    chEvtSignal(thisThd, EVENT_MASK(0));
//  }
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
//    if(pendTask.State == 0){
//      pendTask.State = 2;
//      pendTask.id = id;
//      chEvtSignal(thisThd, EVENT_MASK(0));
//    }
    
  }
  return MSG_OK;
}

msg_t nvm_runtime_get_balancingMasked(uint8_t *p)
{
  for(uint8_t i=0;i<NOF_MAX_CELL_PER_MODULE;i++){
    p[i] = runTime.cells.balancing[i];
    if(runTime.cells.state[i] == 1){
      p[i] = 0;
    }
  }
  return MSG_OK;
}

msg_t nvm_runtime_get_balancing(uint8_t *p)
{
  //chSysLock();
  memcpy(p,runTime.cells.balancing,NOF_MAX_CELL_PER_MODULE);
  //chSysUnlock();
  return MSG_OK;
}

msg_t nvm_runtime_set_balancing(uint8_t *p)
{
  memcpy(runTime.cells.balancing,p,NOF_MAX_CELL_PER_MODULE);
  //chSysLock();
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
  //chSysUnlock();
  return MSG_OK;
}

msg_t nvm_runtime_get_balancingQueued(uint8_t *p)
{
  p[0] = p[1] = 0x0;
  uint8_t i;
  uint8_t used = 0;
  for(i=0;i<NOF_MAX_CELL_PER_MODULE;i++){
    if((nvmBoard.cellQueue[i] == 1)){
      if((runTime.cells.balancing[i] == 1)){
        if(used < 8)
          p[0] |= (1 << used);
        else
          p[1] |= (1 << (used-8));
      }
      used++;
    }
  }
  return MSG_OK;
}

msg_t nvm_runtime_set_openWire(uint8_t *p)
{
  uint16_t tmp = 0x0;
  // set state to 1
  for(uint8_t i=0;i<NOF_MAX_CELL_PER_MODULE+1;i++){
    tmp |= (p[i]==1)?(1 << i):0;
  }
  runTime.cells.openWire = tmp;
}

msg_t nvm_runtime_get_openWire(uint8_t *p)
{
  for(uint8_t i=0;i<NOF_MAX_CELL_PER_MODULE;i++){
    p[i] = runTime.cells.state[i] + runTime.cells.lastState[i];
  }
}

msg_t nvm_runtime_get_openWireQueued(uint16_t *p)
{
  *p = runTime.cells.openWire;
}

void nvm_load_default()
{
  for(uint8_t i=0;i<PG_MAX;i++){
    memcpy(nvmHeader[i].block,nvmHeaderDefault[i].block,nvmHeader[i].sz);
    nvm_write_block(i,nvmHeader[i].block,nvmHeader[i].sz);
  }
}

void nvm_flash_load_default()
{
  for(uint8_t i=0;i<PG_MAX;i++){
    memcpy(nvmHeader[i].block,nvmHeaderDefault[i].block,nvmHeader[i].sz);
    nvm_flash_write_block(i,nvmHeader[i].block,nvmHeader[i].sz);
  }
}

static THD_WORKING_AREA(waNVM,512);
static THD_FUNCTION(procNVM,p){

  while(1){
   eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
   if(pendTask.State == 1){ // read
    nvm_read_block(pendTask.id,nvmHeader[pendTask.id].block,nvmHeader[pendTask.id].sz);
   }
   else if(pendTask.State == 2){ // write
     //chSysLock();
    nvm_write_block(pendTask.id,nvmHeader[pendTask.id].block,nvmHeader[pendTask.id].sz);
    NVIC_SystemReset();
    //chSysUnlock();
   }
   else if(pendTask.State == 4){
     nvm_load_default();
   }
   pendTask.State = 0;
    
  }
}


void nvmInit(I2CDriver *devp)
{
  //dev = devp;
  //at24eep_init(dev,32,1024,0x50,2);
  for(uint8_t i=0;i<PG_MAX;i++){
    nvm_read_block(i,nvmHeader[i].block,nvmHeader[i].sz);
  }
  
  if((nvmBoard.boardId != BOARD_ID) || (nvmBoard.flag == FG_DEFAULT)){
    nvm_load_default();
  }
  
  runTime.activeCells = 0;
  for(uint8_t i=0;i<NOF_MAX_CELL_PER_MODULE;i++){
    if(nvmBoard.cellQueue[i] == 1)
      runTime.activeCells++;
  }
  runTime.id = nvmBoard.id;
  //thisThd = chThdCreateStatic(waNVM, sizeof(waNVM), NORMALPRIO-1, procNVM, NULL);
}


void nvmFlashInit()
{
  nvmFlashRead(); // load nvm
  if(nvmBoard.flag != NVM_FLAG){
    // load default
    nvm_load_default();
    nvmFlashWrite();
  }
  
  
}

