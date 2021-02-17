#ifndef _NVM_
#define _NVM_

#include "board.h"

#define NVM_PAGE_SIZE   64 // bytes
#define SECTION_INITIALIZED     0xAB    // flag to indicate section initialized


enum flags{
  FG_NORMAL,
  FG_DEFAULT
};

/** NVM data block **/
enum nvm_page_e
{
  PG_BOARD,        // board identify
  PG_CMU_BC,    // balance config
  PG_CMU_TC,    // temperature config
  PG_MAX
};

typedef struct{
  void* block;
  uint16_t sz;
}_block_header_s;

typedef struct{
  uint8_t id;
  uint8_t flag;
  uint32_t boardId;
  uint8_t cellQueue[NOF_MAX_CELL_PER_MODULE];
  uint16_t cellVoltReportNormal; 
  uint16_t cellVoltReportFast;
  uint16_t cellTempReportNormal;
  uint16_t cellTempReportFast;
  uint8_t autoStart;
}_nvm_board_s;

typedef struct
{
  uint8_t enableBalance;
  uint16_t balanceVoltMv;
  uint8_t balanceHystersisMv;
  uint16_t onTime;
  uint16_t offTime;
}_nvm_balance_cfg_s;

typedef struct{
  uint16_t beta[NOF_MAX_AUXIO];
  uint16_t resistance[NOF_MAX_AUXIO];
  uint8_t betaTemp[NOF_MAX_AUXIO];
}_nvm_ntc_cfg_s;


/** Runtime Datablock **/
enum cell_state{
  CELL_OK,
  CELL_UV,
  CELL_OV,
  CELL_OPEN
};

typedef struct{
  uint16_t voltages[NOF_MAX_CELL_PER_MODULE];
  uint8_t balancing[NOF_MAX_CELL_PER_MODULE];
  uint8_t state[NOF_MAX_CELL_PER_MODULE]; // normal or abnormal
  uint8_t balMask[2];
}_run_cell_info;

typedef struct{
  uint16_t gpioVolt[NOF_MAX_AUXIO];
  int16_t temperature[NOF_MAX_AUXIO];
}_run_ntc_info;

typedef struct{
  uint8_t id;
  uint8_t activeCells;
  _run_cell_info cells;
  _run_ntc_info ntcTemp;
}_runtime_info_s;


extern _runtime_info_s runTime;

msg_t nvmReadPage(uint8_t page, uint8_t *d);
msg_t nvmWritePage(uint8_t page, uint8_t *d);
void nvmInit(I2CDriver *devp);


#endif