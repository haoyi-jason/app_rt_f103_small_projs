#ifndef _TLE9012_
#define _TLE9012_
#include "tle9012_def.h"


#define CRC_POLY        0x2F
#define CRC_INIT        0x00
#define CRC_XOROUT      0x00

#define TLE_SHORT_DELAY         100
typedef enum{
  TLE9012_NOERR = 0,
  TLE9012_ERR_INIT,
  TLE9012_ERR_COMM,
  TLE9012_ERR_WDIAG,
  TLE9012_ERR_WADDR,
  TLE9012_ERR_WCMD,
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

typedef struct{
  uint16_t read;
  uint16_t write;
}_u16_rw;
  
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
  int8_t(*registerXfer)(TLE9012Driver *dev, uint8_t *tx, uint8_t txnn, uint8_t *rx,uint8_t rxn); \
  uint16_t part_config; \
  uint16_t ol_ov_thr; \
  uint16_t ol_uv_thr; \
  uint16_t temp_conf; \
  uint16_t int_ot_warn_conf; \
  uint16_t rr_err_cnt; \
  uint16_t rr_config; \
  uint16_t fault_mask; \
  uint16_t gen_diag; \
  uint16_t cell_uv; \
  uint16_t cell_ov; \
  uint16_t ext_temp_diag; \
  uint16_t diag_ol; \
  uint16_t reg_crc_err; \
  uint16_t op_mode; \
  uint16_t bal_curr_thr; \
  _u16_rw bal_settings; \
  uint16_t avm_config; \
  uint16_t meas_ctrl; \
  uint16_t cv[12]; \
  uint16_t bv; \
  uint16_t et[5]; \
  uint16_t et_r_diag; \
  uint16_t int_temp; \
  uint16_t multi_read; \
  uint16_t multi_read_cfg; \
  uint16_t bal_dial_oc; \
  uint16_t bal_dial_uc; \
  uint16_t cfg; \
  uint16_t gpio; \
  uint16_t gpio_pwm; \
  uint16_t icvid; \
  uint16_t mailbox; \
  uint16_t cid_0; \
  uint16_t cid_1; \
  _u16_rw wdg_cnt; \
  uint8_t *buffer; \
  uint8_t balancingControl[12];
    
struct TLE9012Driver_s{
  _tle9012_data
};

#define TLE9012_READ_PART_CONFIG(dev) tle9012_readRegister(dev,dev->id,REG_PART_CONFIG,5,(uint8_t*)&dev->part_config)
#define TLE9012_READ_OL_OV_THR(dev) tle9012_readRegister(dev,dev->id,REG_OL_OV_THR,5,(uint8_t*)&dev->ol_ov_thr)
#define TLE9012_READ_OL_UV_THR(dev) tle9012_readRegister(dev,dev->id,REG_OL_UV_THR,5,(uint8_t*)&dev->ol_uv_thr)
#define TLE9012_READ_TEMP_CONF(dev) tle9012_readRegister(dev,dev->id,REG_TEMP_CONF,5,(uint8_t*)&dev->temp_conf)
#define TLE9012_READ_INT_OT_WARN_CONF(dev) tle9012_readRegister(dev,dev->id,REG_INT_OT_WARN_CONF,5,(uint8_t*)&dev->int_ot_warn_conf)
#define TLE9012_READ_RR_ERR_CNT(dev) tle9012_readRegister(dev,dev->id,REG_RR_ERR_CNT,5,(uint8_t*)&dev->rr_err_cnt)
#define TLE9012_READ_RR_CONFIG(dev) tle9012_readRegister(dev,dev->id,REG_RR_CONFIG,5,(uint8_t*)&dev->rr_config)
#define TLE9012_READ_FAULT_MASK(dev) tle9012_readRegister(dev,dev->id,REG_FAULT_MASK,5,(uint8_t*)&dev->fault_mask)
#define TLE9012_READ_GEN_DIAG(dev) tle9012_readRegister(dev,dev->id,REG_GEN_DIAG,5,(uint8_t*)&dev->gen_diag)
#define TLE9012_READ_CELL_UV(dev) tle9012_readRegister(dev,dev->id,REG_CELL_UV,5,(uint8_t*)&dev->cell_uv)
#define TLE9012_READ_CELL_OV(dev) tle9012_readRegister(dev,dev->id,REG_CELL_OV,5,(uint8_t*)&dev->cell_ov)
#define TLE9012_READ_EXT_TEMP_DIAG(dev) tle9012_readRegister(dev,dev->id,REG_EXT_TEMP_DIAG,5,(uint8_t*)&dev->ext_temp_diag)
#define TLE9012_READ_DIAG_OL(dev) tle9012_readRegister(dev,dev->id,REG_DIAG_OL,5,(uint8_t*)&dev->diag_ol)
#define TLE9012_READ_REG_CRC_ERR(dev) tle9012_readRegister(dev,dev->id,REG_CRC_ERR,5,(uint8_t*)&dev->reg_crc_err)
#define TLE9012_READ_OPMODE(dev) tle9012_readRegister(dev,dev->id,REG_OP_MODE,5,(uint8_t*)&dev->op_mode)
#define TLE9012_READ_BAL_CURR_THR(dev) tle9012_readRegister(dev,dev->id,REG_BAL_CURR_THR,5,(uint8_t*)&dev->bal_curr_thr)
#define TLE9012_READ_BAL_SETTINGS(dev) tle9012_readRegister(dev,dev->id,REG_BAL_SETTING,5,(uint8_t*)&dev->bal_settings.read)
#define TLE9012_READ_AVM_CONFIG(dev) tle9012_readRegister(dev,dev->id,REG_AVM_CONFIG,5,(uint8_t*)&dev->avm_config)
#define TLE9012_READ_MEAS_CTRL(dev) tle9012_readRegister(dev,dev->id,REG_MEAS_CTRL,5,(uint8_t*)&dev->meas_ctrl)

#define TLE9012_READ_EXT_TEMP_R_DIAG(dev) tle9012_readRegister(dev,dev->id,REG_EXT_TMP_R_DIAG,5,(uint8_t*)&dev->et_r_diag)
#define TLE9012_READ_INT_TEMP(dev) tle9012_readRegister(dev,dev->id,REG_INT_TEMP,5,(uint8_t*)&dev->int_temp)

#define TLE9012_READ_MULTI_READ(dev) tle9012_readRegisterMulti(dev,dev->id,REG_MULTI_READ,99,(uint8_t*)dev->buffer)

#define TLE9012_READ_MULTI_READ_CFG(dev) tle9012_readRegister(dev,dev->id,REG_MULTI_READ_CFG,5,(uint8_t*)&dev->multi_read_cfg)
#define TLE9012_READ_BAL_DIAL_OC(dev) tle9012_readRegister(dev,dev->id,REG_BAL_DIAG_OC,5,(uint8_t*)&dev->bal_dial_oc)
#define TLE9012_READ_BAL_DIAL_UC(dev) tle9012_readRegister(dev,dev->id,REG_BAL_DIAG_UC,5,(uint8_t*)&dev->bal_dial_uc)
#define TLE9012_READ_CONFIG(dev) tle9012_readRegister(dev,dev->id,REG_CONFIG,5,(uint8_t*)&dev->cfg)
#define TLE9012_READ_GPIO(dev) tle9012_readRegister(dev,dev->id,REG_GPIO,5,(uint8_t*)&dev->gpio)
#define TLE9012_READ_GPIO_PWM(dev) tle9012_readRegister(dev,dev->id,REG_GPIO_PWM,5,(uint8_t*)&dev->gpio_pwm)
#define TLE9012_READ_ICVID(dev) tle9012_readRegister(dev,dev->id,REG_ICVID,5,(uint8_t*)&dev->icvid)
#define TLE9012_READ_MAILBOX(dev) tle9012_readRegister(dev,dev->id,REG_MAILBOX,5,(uint8_t*)&dev->mailbox)
#define TLE9012_READ_CID_0(dev) tle9012_readRegister(dev,dev->id,REG_CUSTOM_ID_0,5,(uint8_t*)&dev->cid_0)
#define TLE9012_READ_CID_1(dev) tle9012_readRegister(dev,dev->id,REG_CUSTOM_ID_1,5,(uint8_t*)&dev->cid_1)
#define TLE9012_READ_WDG_CNT(dev) tle9012_readRegister(dev,dev->id,WDG_CNT,5,(uint8_t*)&dev->wdg_cnt.read)

#define TLE9012_WRITE_PART_CONFIG(dev) tle9012_writeRegister(dev,dev->id,REG_PART_CONFIG,dev->part_config)
#define TLE9012_WRITE_OL_OV_THR(dev) tle9012_writeRegister(dev,dev->id,REG_OL_OV_THR,dev->ol_ov_thr)
#define TLE9012_WRITE_OL_UV_THR(dev) tle9012_writeRegister(dev,dev->id,REG_OL_UV_THR,dev->ol_uv_thr)
#define TLE9012_WRITE_TEMP_CONF(dev) tle9012_writeRegister(dev,dev->id,REG_TEMP_CONF,dev->temp_conf)
#define TLE9012_WRITE_INT_OT_WARN_CONF(dev) tle9012_writeRegister(dev,dev->id,REG_INT_OT_WARN_CONF,dev->int_ot_warn_conf)
#define TLE9012_WRITE_RR_ERR_CNT(dev) tle9012_writeRegister(dev,dev->id,REG_RR_ERR_CNT,dev->rr_err_cnt)
#define TLE9012_WRITE_RR_CONFIG(dev) tle9012_writeRegister(dev,dev->id,REG_RR_CONFIG,dev->rr_config)
#define TLE9012_WRITE_FAULT_MASK(dev) tle9012_writeRegister(dev,dev->id,REG_FAULT_MASK,dev->fault_mask)
#define TLE9012_WRITE_GEN_DIAG(dev) tle9012_writeRegister(dev,dev->id,REG_GEN_DIAG,dev->gen_diag)
#define TLE9012_WRITE_CELL_UV(dev) tle9012_writeRegister(dev,dev->id,REG_CELL_UV,dev->cell_uv)
#define TLE9012_WRITE_CELL_OV(dev) tle9012_writeRegister(dev,dev->id,REG_CELL_OV,dev->cell_ov)
#define TLE9012_WRITE_EXT_TEMP_DIAG(dev) tle9012_writeRegister(dev,dev->id,REG_EXT_TEMP_DIAG,dev->ext_temp_diag)
#define TLE9012_WRITE_DIAG_OL(dev) tle9012_writeRegister(dev,dev->id,REG_DIAG_OL,dev->diag_ol)
#define TLE9012_WRITE_REG_CRC_ERR(dev) tle9012_writeRegister(dev,dev->id,REG_CRC_ERR,dev->reg_crc_err)
#define TLE9012_WRITE_OPMODE(dev) tle9012_writeRegister(dev,dev->id,REG_OP_MODE,dev->op_mode)
#define TLE9012_WRITE_BAL_CURR_THR(dev) tle9012_writeRegister(dev,dev->id,REG_BAL_CURR_THR,dev->bal_curr_thr)
#define TLE9012_WRITE_BAL_SETTINGS(dev) tle9012_writeRegister(dev,dev->id,REG_BAL_SETTING,dev->bal_settings.write)
#define TLE9012_WRITE_AVM_CONFIG(dev) tle9012_writeRegister(dev,dev->id,REG_AVM_CONFIG,dev->avm_config)
#define TLE9012_WRITE_MEAS_CTRL(dev) tle9012_writeRegister(dev,dev->id,REG_MEAS_CTRL,dev->meas_ctrl)

#define TLE9012_WRITE_EXT_TEMP_R_DIAG(dev) tle9012_writeRegister(dev,dev->id,REG_EXT_TMP_R_DIAG,dev->et_r_diag)
#define TLE9012_WRITE_INT_TEMP(dev) tle9012_writeRegister(dev,dev->id,REG_INT_TEMP,dev->int_temp)
#define TLE9012_WRITE_MULTI_READ_CFG(dev) tle9012_writeRegister(dev,dev->id,REG_MULTI_READ_CFG,dev->multi_read_cfg)
#define TLE9012_WRITE_BAL_DIAL_OC(dev) tle9012_writeRegister(dev,dev->id,REG_BAL_DIAG_OC,2,dev->bal_dial_oc)
#define TLE9012_WRITE_BAL_DIAL_UC(dev) tle9012_writeRegister(dev,dev->id,REG_BAL_DIAG_UC,2,dev->bal_dial_uc)
#define TLE9012_WRITE_CONFIG(dev) tle9012_writeRegister(dev,dev->id,REG_CONFIG,2,(uint8_t*)&dev->cfg)
#define TLE9012_WRITE_GPIO(dev) tle9012_writeRegister(dev,dev->id,REG_GPIO,2,(uint8_t*)&dev->gpio)
#define TLE9012_WRITE_GPIO_PWM(dev) tle9012_writeRegister(dev,dev->id,REG_GPIO_PWM,2,(uint8_t*)&dev->gpio_pwm)
#define TLE9012_WRITE_ICVID(dev) tle9012_writeRegister(dev,dev->id,REG_ICVID,2,(uint8_t*)&dev->icvid)
#define TLE9012_WRITE_MAILBOX(dev) tle9012_writeRegister(dev,dev->id,REG_MAILBOX,2,(uint8_t*)&dev->mailbox)
#define TLE9012_WRITE_CID_0(dev) tle9012_writeRegister(dev,dev->id,REG_CUSTOM_ID_0,2,(uint8_t*)&dev->cid_0)
#define TLE9012_WRITE_CID_1(dev) tle9012_writeRegister(dev,dev->id,REG_CUSTOM_ID_1,2,(uint8_t*)&dev->cid_1)
#define TLE9012_WRITE_WDG_CNT(dev) tle9012_writeRegister(dev,dev->id,WDG_CNT,dev->wdg_cnt.write)

uint8_t crc8_msb(uint8_t poly,uint8_t *data, uint8_t sz);
TLE9012_ERR tle9012_readRegister(TLE9012Driver *dev,uint8_t id, uint8_t reg,uint8_t sz, uint8_t *value);
uint8_t tle9012_writeRegister(TLE9012Driver *dev, uint8_t id, uint8_t reg, uint16_t value);
void tle9012_feedWDT(TLE9012Driver *dev, uint8_t count);
#endif
