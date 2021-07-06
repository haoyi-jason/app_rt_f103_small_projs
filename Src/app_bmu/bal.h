#ifndef _BAL_
#define _BAL_

// balancing 

#include "hal.h"
#include "ltc_cfg.h"


#define BS_NR_OF_BAT_CELLS 12
/**
 * States of the BAL state machine
 */
typedef enum {
    /* Init-Sequence */
    BAL_STATEMACH_UNINITIALIZED             = 0,    /*!<    */
    BAL_STATEMACH_INITIALIZATION            = 1,    /*!<    */
    BAL_STATEMACH_INITIALIZED               = 2,    /*!<    */
    BAL_STATEMACH_CHECK_BALANCING           = 3,    /*!<    */
    BAL_STATEMACH_BALANCE                   = 4,    /*!<    */
    BAL_STATEMACH_NOBALANCING               = 5,    /*!<    */
    BAL_STATEMACH_ALLOWBALANCING            = 6,    /*!<    */
    BAL_STATEMACH_GLOBALDISABLE             = 7,    /*!<    */
    BAL_STATEMACH_GLOBALENABLE              = 8,    /*!<    */
    BAL_STATEMACH_UNDEFINED                 = 20,   /*!< undefined state                                */
    BAL_STATEMACH_RESERVED1                 = 0x80, /*!< reserved state                                 */
    BAL_STATEMACH_ERROR                     = 0xF0, /*!< Error-State:  */
} BAL_STATEMACH_e;


/**
 * Substates of the BAL state machine
 */
typedef enum {
    BAL_ENTRY                                    = 0,    /*!< Substate entry state       */
    BAL_CHECK_IMBALANCES                         = 1,  /*!< Check if balancing has been initialized       */
    BAL_COMPUTE_IMBALANCES                       = 2,  /*!< Compute imbalances */
    BAL_ACTIVATE_BALANCING                       = 3,  /*!< Activated balancing resistors */
    BAL_CHECK_LOWEST_VOLTAGE                     = 4,  /*!< Check if lowest voltage is still  above limit */
    BAL_CHECK_CURRENT                            = 5,  /*!< Check if current is still  under limit */
} BAL_STATEMACH_SUB_e;

/**
 * State requests for the BAL statemachine
 */
typedef enum {
    BAL_STATE_INIT_REQUEST                     = BAL_STATEMACH_INITIALIZATION,           /*!<    */
    BAL_STATE_ERROR_REQUEST                    = BAL_STATEMACH_ERROR,   /*!<    */
    BAL_STATE_NOBALANCING_REQUEST              = BAL_STATEMACH_NOBALANCING,     /*!<    */
    BAL_STATE_ALLOWBALANCING_REQUEST           = BAL_STATEMACH_ALLOWBALANCING,     /*!<    */
    BAL_STATE_GLOBAL_DISABLE_REQUEST           = BAL_STATEMACH_GLOBALDISABLE,     /*!<    */
    BAL_STATE_GLOBAL_ENABLE_REQUEST            = BAL_STATEMACH_GLOBALENABLE,     /*!<    */
    BAL_STATE_NO_REQUEST                       = BAL_STATEMACH_RESERVED1,                /*!<    */
} BAL_STATE_REQUEST_e;


/**
 * Possible return values when state requests are made to the BAL statemachine
 */
typedef enum {
    BAL_OK                                 = 0,    /*!< BAL --> ok                             */
    BAL_BUSY_OK                            = 1,    /*!< BAL busy                               */
    BAL_REQUEST_PENDING                    = 2,    /*!< requested to be executed               */
    BAL_ILLEGAL_REQUEST                    = 3,    /*!< Request can not be executed            */
    BAL_INIT_ERROR                         = 7,    /*!< Error state: Source: Initialization    */
    BAL_OK_FROM_ERROR                      = 8,    /*!< Return from error --> ok               */
    BAL_ERROR                              = 20,   /*!< General error state                    */
    BAL_ALREADY_INITIALIZED                = 30,   /*!< Initialization of BAL already finished */
    BAL_ILLEGAL_TASK_TYPE                  = 99,   /*!< Illegal                                */
} BAL_RETURN_TYPE_e;

/**
 * This structure contains all the variables relevant for the BAL state machine.
 * The user can get the current state of the BAL state machine with this variable
 */
typedef struct {
    uint16_t timer;                         /*!< time in ms before the state machine processes the next state, e.g. in counts of 1ms */
    BAL_STATE_REQUEST_e statereq;           /*!< current state request made to the state machine                                     */
    BAL_STATEMACH_e state;                  /*!< state of Driver State Machine                                                       */
    BAL_STATEMACH_SUB_e substate;           /*!< current substate of the state machine                                               */
    BAL_STATEMACH_e laststate;              /*!< previous state of the state machine                                                 */
    uint8_t lastsubstate;                   /*!< previous substate of the state machine                                              */
    uint8_t triggerentry;                   /*!< counter for re-entrance protection (function running flag) */
    uint32_t ErrRequestCounter;             /*!< counts the number of illegal requests to the BAL state machine */
    STD_RETURN_TYPE_e initFinished;         /*!< E_OK if statemachine initialized, otherwise E_NOT_OK */
    uint8_t active;                         /*!< indicate if balancing active or not */
    uint32_t balancing_threshold;           /*!< effective balancing threshold */
    uint8_t balancing_allowed;              /*!< flag to disable balancing */
    uint8_t balancing_global_allowed;       /*!< flag to globally disable balancing */
} BAL_STATE_s;

/**
 * data structure declaration of DATA_BLOCK_BALANCING_CONTROL
 */
//typedef struct {
//    /* Timestamp info needs to be at the beginning. Automatically written on DB_WriteBlock */
//    uint32_t timestamp;                         /*!< timestamp of database entry                */
//    uint32_t previous_timestamp;                /*!< timestamp of last database entry           */
//    uint8_t balancing_state[BS_NR_OF_BAT_CELLS];    /*!< 0 means balancing is active, 0 means balancing is inactive*/
//    uint32_t delta_charge[BS_NR_OF_BAT_CELLS];    /*!< Difference in Depth-of-Discharge in mAs*/
//    uint8_t enable_balancing;           /*!< Switch for enabling/disabling balancing    */
//    uint8_t threshold;                  /*!< balancing threshold in mV                  */
//    uint8_t request;                     /*!< balancing request per CAN                 */
//    uint8_t state;                      /*!< for future use                             */
//} DATA_BLOCK_BALANCING_CONTROL_s;

/**
 * data block struct of LTC minimum and maximum values
 */
//typedef struct {
//    /* Timestamp info needs to be at the beginning. Automatically written on DB_WriteBlock */
//    uint32_t timestamp;                         /*!< timestamp of database entry                */
//    uint32_t previous_timestamp;                /*!< timestamp of last database entry           */
//    uint32_t voltage_mean;
//    uint16_t voltage_min;
//    uint16_t voltage_module_number_min;
//    uint16_t voltage_cell_number_min;
//    uint16_t previous_voltage_min;
//    uint16_t voltage_max;
//    uint16_t voltage_module_number_max;
//    uint16_t voltage_cell_number_max;
//    uint16_t previous_voltage_max;
//    float temperature_mean;
//    int16_t temperature_min;
//    uint16_t temperature_module_number_min;
//    uint16_t temperature_sensor_number_min;
//    int16_t temperature_max;
//    uint16_t temperature_module_number_max;
//    uint16_t temperature_sensor_number_max;
//    uint8_t state;
//} DATA_BLOCK_MINMAX_s;

typedef struct{
  uint16_t timer;  // time remaining to balancing
  uint8_t dir;          // 0: idle, 1: active
  uint8_t balancing;    //0x00: no, 0x01: yes
}_cell_bal_state_s;

typedef struct{
  uint32_t timeStamp;
  uint16_t totalWatt; // how many milli-wattage discharging
  _cell_bal_state_s cells[BS_NR_OF_BAT_CELLS_PER_MODULE];
  uint8_t enableBalancing;
  uint8_t shuntResistor;
  uint16_t balancingVolt;
  uint8_t balancingHystersis;
  uint16_t volt[12];
  uint8_t enabledCells;
  uint16_t OnTimeSecond;
  uint16_t OffTimeSecond;
}bal_state_s;

void balSetBalancingVoltage(uint16_t volt, uint8_t band);
void balSetState(uint8_t enable);

#endif