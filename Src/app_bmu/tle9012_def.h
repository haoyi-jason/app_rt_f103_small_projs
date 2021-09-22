#ifndef _TLE9012_DEF_H
#define _TLE9012_DEF_H

#define TLE9012_MAX_CELLS       12

#define TLC_FRAME_SYNC  0x1E
#define TLC_ID_WR_MASK  0x80

#define WR_REPLY_FAULT_MASK     0x04
#define WR_REPLY_ADDRV_MASK     0x10
#define WR_REPLY_CMD_OK         0x20

#define REG_PART_CONFIG         0x01
#define REG_OL_OV_THR           0x02
#define REG_OL_UV_THR           0x03
#define REG_TEMP_CONF           0x04
#define REG_INT_OT_WARN_CONF    0x05
#define REG_RR_ERR_CNT          0x08
#define REG_RR_CONFIG           0x09
#define REG_FAULT_MASK          0x0A
#define REG_GEN_DIAG            0x0B
#define REG_CELL_UV             0x0C
#define REG_CELL_OV             0x0D
#define REG_EXT_TEMP_DIAG       0x0E
#define REG_DIAG_OL             0x10
#define REG_CRC_ERR             0x11
#define REG_OP_MODE             0x14
#define REG_BAL_CURR_THR        0x15
#define REG_BAL_SETTING         0x16
#define REG_AVM_CONFIG          0x17
#define REG_MEAS_CTRL           0x18
#define REG_CVM_0               0x19
#define REG_CVM_1               0x1A
#define REG_CVM_2               0x1B
#define REG_CVM_3               0x1C
#define REG_CVM_4               0x1D
#define REG_CVM_5               0x1E
#define REG_CVM_6               0x1F
#define REG_CVM_7               0x20
#define REG_CVM_8               0x21
#define REG_CVM_9               0x22
#define REG_CVM_10              0x23
#define REG_CVM_11              0x24
#define REG_BVM                 0x28
#define REG_EXT_TMP_0           0x29
#define REG_EXT_TMP_1           0x2A
#define REG_EXT_TMP_2           0x2B
#define REG_EXT_TMP_3           0x2C
#define REG_EXT_TMP_4           0x2D
#define REG_EXT_TMP_R_DIAG      0x2F
#define REG_INT_TEMP            0x30
#define REG_MULTI_READ          0x31
#define REG_MULTI_READ_CFG      0x32
#define REG_BAL_DIAG_OC         0x33
#define REG_BAL_DIAG_UC         0x34
#define REG_CONFIG              0x36
#define REG_GPIO                0x37
#define REG_GPIO_PWM            0x38
#define REG_ICVID               0x39
#define REG_MAILBOX             0x3A
#define REG_CUSTOM_ID_0         0x3B
#define REG_CUSTOM_ID_1         0x3C
#define WDG_CNT                 0x3D

//- MEAS_CTRL
#define MEAS_CTRL_START         (1 << 15)
#define MEAS_CTRL_BITWIDTH(x)   (x << 12)
typedef enum {
  MEAS_CTRL_BIT10,
  MWAS_CTRL_BIT11,
  MWAS_CTRL_BIT16=6
}MEAS_CTRL_BIT_e;

#define MEAS_CTRL_START_BVM     (1 << 11)
#define MEAS_CTRL_BVMBITWIDTH(x)   (x << 8)
#define MEAS_CTRL_START_AVM     (1 << 7)
#define MAAS_CTRL_PBOFF         (1 << 5)


typedef enum{
  PBOFF_KEEP_BALANCING,
  PBOFF_BALANCING_OFF
}TLE_PBOFF_e;

typedef enum{
  TLE_ADCMODE_UNDEFINED         =0,
  TLE_ADCMODE_11BIT,
  TLE_ADCMODE_12BIT,
  TLE_ADCMODE_13BIT,
  TLE_ADCMODE_14BIT,
  TLE_ADCMODE_15BIT,
  TLE_ADCMODE_16BIT,
}TLE_ADCMODE_e;

typedef enum{
  TLE_ADCMEAS_UNDEFINED         =0,
  TLE_ADCMEAS_ALLCHANNEL        
}TLE_ADCMEAS_CHAN_e;

typedef enum{
  TLE_STATEMACH_UNINITIALIZED   =0,
  TLE_STATEMACH_INITIALIZATION,
  TLE_STATEMACH_INITIALIZED,
  TLE_STATEMACH_STARTVMEAS,
  TLE_STATEMACH_READVOLTAGE,
  TLE_STATEMACH_BALANCECONTROL,
  TLE_STATEMACH_ALLGPIOMEASUREMENT,
  TLE_STATEMACH_READALLGPIO,
  TLE_STATEMACH_TEMPSEN_READ,
  TLE_STATEMACH_OPENWIRE_CHECK
}TLE_STATEMACH_e;

typedef enum{
  TLE_STATE_INIT_REQUEST                = TLE_STATEMACH_INITIALIZATION,
  TLE_STATE_VOLTAGEMEASUREMENT_REQUEST  = TLE_STATEMACH_STARTVMEAS,
  TLE_STATE_READVOLTAGE_REQUEST         = TLE_STATEMACH_READVOLTAGE,
  TLE_STATE_OPENWIRE_CHECK_REQUEST      = TLE_STATEMACH_OPENWIRE_CHECK,
  TLE_STATE_BALANCECONTROL_REQUEST      = TLE_STATEMACH_BALANCECONTROL,
  TLE_STATE_ALLGPIOMEASUREMENT_REQUEST  = TLE_STATEMACH_ALLGPIOMEASUREMENT,
  TLE_STATE_NO_REQUEST
}TLE_STATE_REQUEST_e;

typedef enum{
  TLE_ENTRY             =0,
  
}TLE_STATEMACH_SUB_e;

/**
 * Substates for the initialization state
 */
typedef enum {
    /* Init-Sequence */
    TLE_ENTRY_INITIALIZATION            = 0,    /*!<    */
    TLE_START_INIT_INITIALIZATION       = 1,    /*!<    */
    TLE_RE_ENTRY_INITIALIZATION         = 2,    /*!<    */
    TLE_READ_INITIALIZATION_REGISTER    = 3,    /*!<    */
    TLE_EXIT_INITIALIZATION             = 4,    /*!<    */
} TLE_STATEMACH_INITIALIZATION_SUB_e;

typedef enum{
  TLE_READ_VOLTAGE_             =0,
  TLE_EXIT_READVOLTAGE
}TLE_STATEMACH_READVOLTAGE_SUB_e;

typedef enum{
  TLE_CONFIG_BALANCECONTROL     =0,
  
}TLE_STATEMACH_BALANCCONTROL_SUB_e;

typedef enum{
  TLE_REQUEST_OPENWIRE_CHECK    = 0,
}TLE_STATEMACH_OPENWIRECHECK_SUB_e;

typedef enum{
  TLE_REQUEST_DEVICE_PARAMETER  =0,
  
}TLE_STATEMACH_DIAGNOSIS_SUB_e;


typedef enum{
  TLE_OK,
  TLE_BUSY_OK,
  TLE_REQUEST_PENDING,
  TLE_ILLEGAL_REQUEST,
  TLE_UART_ERROR,
  TLE_INIT_ERROR,
  TLE_OK_FROM_ERROR,
  TLE_ERROR = 20,
  TLE_ALREADY_INITIALIZED,
  TLE_ILLEGAL_TASK_TYPE
}TLE_RETURN_TYPE_e;

typedef struct{
  uint32_t timeStamp;
  uint32_t prevTimeStamp;
  uint16_t voltage[TLE9012_MAX_CELLS]; 
  uint16_t validVolt; // bit mask
  uint32_t sumOfCells; // mv
  uint8_t openWire[TLE9012_MAX_CELLS];
  uint8_t activeCellQue[TLE9012_MAX_CELLS];
  uint8_t activeCells;
  uint8_t state;
  uint16_t gpioVoltage[5];
  uint16_t validGpioVolt;
  uint16_t balanceFeedback[TLE9012_MAX_CELLS];
  uint16_t pulldnFeedback[TLE9012_MAX_CELLS];
  uint16_t pullupFeedback[TLE9012_MAX_CELLS];
}_cell_def;

typedef struct{
  uint16_t timer;
  TLE_STATEMACH_e state;
  TLE_STATEMACH_e statereq;
  uint8_t subState;
  TLE_STATEMACH_e lastState;
  uint8_t lastSubState;
  TLE_ADCMODE_e adcMode;
  TLE_ADCMODE_e adcModereq;
  TLE_ADCMEAS_CHAN_e adcMeasCh;
  TLE_ADCMEAS_CHAN_e adcMeasChreq;
  _cell_def cells;
  uint32_t commonDataTransferTime;
  uint32_t commonTransferTime;
  uint8_t usedIndex;
  uint16_t measCtrl;
  uint8_t errorCount;
}TLE_STATE_s;


#endif