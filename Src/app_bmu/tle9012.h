#ifndef _TLE9012_
#define _TLE9012_
#include "tle9012_def.h"

#define TLE_SHORT_DELAY         100
typedef enum{
  TLE9012_NOERR = 0,
  TLE9021_ERR = -1,
}TLE9012_ERR;

typedef struct TLE9012Driver_s TLE9012Driver;

typedef struct{
  SerialDriver *devp;
  const SerialConfig *config;
}TLE9012Config;

typedef enum{
  TLE9012_UNINIT = 0,
  TLE9012_STOP,
  TLE9012_READY,
  TLE9012_MEASURE,
  TLE9012_SLEEP
}tle9012_state_t;
  
//typedef struct{
//  uint32_t timeStamp;
//  uint32_t prevTimeStamp;
//  uint16_t voltage[12]; 
//  uint16_t validVolt; // bit mask
//  uint32_t sumOfCells; // mv
//  uint8_t openWire[12];
//  uint8_t activeCellQue[12];
//  uint8_t activeCells;
//  uint8_t state;
//  uint16_t gpioVoltage[5];
//  uint16_t validGpioVolt;
//  uint16_t balanceFeedback[12];
//  uint16_t pulldnFeedback[12];
//  uint16_t pullupFeedback[12];
//}_cell_def;

#define _tle9012_data \
  tle9012_state_t state; \
  const TLE9012Config *config; \
  uint8_t id; \
  int8_t(*registerRead)(TLE9012Driver *dev, uint8_t *b, uint8_t n, uint32_t timeout); \
  int8_t(*registerWrite)(TLE9012Driver *dev, uint8_t *b, uint8_t n, uint32_t timeout);    
struct TLE9012Driver_s{
  _tle9012_data
};

#endif
