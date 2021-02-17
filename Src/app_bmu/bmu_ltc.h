#ifndef _BMU_LTC_
#define _BMU_LTC_
#include "hal.h"
#include "ltc68xx.h"

/**
 * LTC statemachine short time definition in ms
 */
#define LTC_STATEMACH_SHORTTIME     1


typedef struct {
    uint8_t PEC_valid;  /*!<    */
    uint8_t mux0;       /*!<    */
    uint8_t mux1;       /*!<    */
    uint8_t mux2;       /*!<    */
    uint8_t mux3;       /*!<    */
} LTC_ERRORTABLE_s;

typedef struct{
  uint8_t flag;
  uint8_t id;   // board id in canbus 
  uint8_t type;
  uint8_t cellEnableMask[12];
  uint8_t enableAutoBalance;
  uint8_t balancingCriteria;
  uint16_t normapReportInterval;
  uint16_t fastReportInterval;
  uint16_t lowVoltageThres;
  uint16_t highVoltageThres;
  uint16_t lowTempThres;
  uint16_t highTempThres;
}_cmu_nvm_s;

typedef struct{
  uint16_t timer;
  LTC_TASK_TYPE_e taskMode;
  LTC_STATE_REQUEST_e statereq;
  LTC_STATEMACH_e state;
  LTC_STATEMACH_e lastState;
  uint8_t subState;
  uint8_t lastSubState;
  uint8_t configuration[6];
  LTC_ADCMODE_e adcMode;
  LTC_ADCMODE_e voltMeasMode;
  LTC_ADCMODE_e gpioMeasMode;
  LTC_ADCMODE_e adcModereq;
  LTC_ADCMEAS_CHAN_e adcMeasCh;
  LTC_ADCMEAS_CHAN_e adcMeasChreq;
  uint32_t errPECCounter;
  uint8_t errRetryCounter;
  uint32_t errRequestCounter;
  uint8_t triggerentry;
  uint32_t commandDataTransferTime;
  uint32_t commandTransferTime;
  uint32_t gpioClocksTransferTime;
  uint32_t voltageSampleTime;
  uint32_t muxSampleTime;
  uint8_t nofCellPerModule;
  LTC_ERROR_s errStatus;
  LTC_CONFIG_s ltcConfig;
  uint8_t firstMeasMade;
  uint8_t resendCommandCounter;
  STD_RETURN_TYPE_e balance_control_done;
  LTC_REUSE_MODE_e reusageMeasurementMode;
  LTC_ERRORTABLE_s errTable;
  _cell_def cells;
  uint8_t usedIndex;
}_bmu_state_s;

#endif