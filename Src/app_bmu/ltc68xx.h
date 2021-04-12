#ifndef _LTC68XX_
#define _LTC68XX_

#include "ltc_defs.h"

typedef enum{
  LTC68XX_NOERROR = 0,
  LTC68XX_ERR_STATE = -1,
}LTC68XX_ERR;

typedef struct LTC68XXDriver_s LTC68XXDriver;

typedef struct{
  SPIDriver *devp;
  const SPIConfig *config;
}LTC68xxConfig;

typedef enum{
  LTC68XX_UNINIT = 0,
  LTC68XX_STOP,
  LTC68XX_READY,
  LTC68XX_MEASURE,
  LTC68XX_SLEEP
}ltc68xx_state_t;

    
typedef struct{
  uint32_t timeStamp;
  uint32_t prevTimeStamp;
  uint16_t voltage[12]; 
  uint16_t validVolt; // bit mask
  uint32_t sumOfCells; // mv
  uint8_t openWire[13];
  uint8_t activeCellQue[12];
  uint8_t activeCells;
  uint8_t state;
  uint16_t gpioVoltage[5];
  uint16_t validGpioVolt;
  uint16_t balanceFeedback[12];
  uint16_t pulldnFeedback[12];
  uint16_t pullupFeedback[12];
}_cell_def;

typedef struct{
  uint32_t timeStamp;
  uint32_t prevTimeStamp;
  float soc_mean;
  float soc_min;
  float soc_max;
  uint8_t state;
}_soc_def;

typedef struct{
  uint32_t timeStamp;
  uint32_t prevTimeStamp;
  uint8_t balancing_state[12];
  uint32_t deltaCharge[12];
  uint8_t enableBalancing;
  uint8_t threshold;
  uint8_t request;
  uint8_t state;
}_balancing_control_s;


typedef struct{
  uint8_t a;
  
}_ltc_module_t;

#define _ltc68xx_data \
  ltc68xx_state_t state; \
  const LTC68xxConfig *config; \
  _balancing_control_s *balance; \
  int8_t(*registerRead)(LTC68XXDriver *dev, uint8_t *b, uint16_t n, uint8_t cs); \
  int8_t(*registerWrite)(LTC68XXDriver *dev, uint8_t *b, uint16_t n, uint8_t cs); \
  int8_t(*exchangeData)(LTC68XXDriver *dev, uint8_t *tx, uint16_t ntx, uint8_t *rx, uint16_t nrx);
    
struct LTC68XXDriver_s{
  _ltc68xx_data
};


#endif