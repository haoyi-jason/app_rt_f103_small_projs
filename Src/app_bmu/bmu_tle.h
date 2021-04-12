#ifndef _BMU_TLE_
#define _BMU_TLE_

#include "hal.h"
//#include "tle9012.h"
//#include "tle9012_def.h"

#define TLE_MAXCELL_PER_MODULE  12

#define TLE_STATEMACH_SHORTTIME 5
#define TLE_STATEMACH_LONGTIME  10

//
//typedef struct{
//  uint8_t flag;
//  uint8_t id;   // board id in canbus 
//  uint8_t type;
//  uint8_t cellEnableMask[TLE_MAXCELL_PER_MODULE];
//  uint8_t enableAutoBalance;
//  uint8_t balancingCriteria;
//  uint16_t normapReportInterval;
//  uint16_t fastReportInterval;
//  uint16_t lowVoltageThres;
//  uint16_t highVoltageThres;
//  uint16_t lowTempThres;
//  uint16_t highTempThres;
//}_cmu_nvm_s;

msg_t bmu_tle_init();

#endif