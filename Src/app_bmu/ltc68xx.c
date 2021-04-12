#include "hal.h"
//#include "ltc_defs.h"
#include "ltc68xx.h"
#include "ltc_pec.h"

static const uint8_t ltc_cmdDummy[1]={0x00};
static const uint8_t ltc_cmdWRCFG[4]={0x00, 0x01, 0x3D, 0x6E};
static const uint8_t ltc_cmdWRCFG2[4]={0x00, 0x24, 0xB1, 0x9E};

static const uint8_t ltc_cmdRDCVA[4] = {0x00, 0x04, 0x07, 0xC2};
static const uint8_t ltc_cmdRDCVB[4] = {0x00, 0x06, 0x9A, 0x94};
static const uint8_t ltc_cmdRDCVC[4] = {0x00, 0x08, 0x5E, 0x52};
static const uint8_t ltc_cmdRDCVD[4] = {0x00, 0x0A, 0xC3, 0x04};
static const uint8_t ltc_cmdRDCVE[4] = {0x00, 0x09, 0xD5, 0x60};
static const uint8_t ltc_cmdRDCVF[4] = {0x00, 0x0B, 0x48, 0x36};
static const uint8_t ltc_cmdWRCOMM[4] = {0x07, 0x21, 0x24, 0xB2};
static const uint8_t ltc_cmdSTCOMM[4] = {0x07, 0x23, 0xB9, 0xE4};
static const uint8_t ltc_cmdRDCOMM[4] = {0x07, 0x22, 0x32, 0xD6};
static const uint8_t ltc_cmdRDAUXA[4] = {0x00, 0x0C, 0xEF, 0xCC};
static const uint8_t ltc_cmdRDAUXB[4] = {0x00, 0x0E, 0x72, 0x9A};
static const uint8_t ltc_cmdRDAUXC[4] = {0x00, 0x0D, 0x64, 0xFE};
static const uint8_t ltc_cmdRDAUXD[4] = {0x00, 0x0F, 0xF9, 0xA8};

/* static const uint8_t ltc_cmdMUTE[4] = {0x00, 0x28, 0xE8, 0x0E};                    !< MUTE discharging via S pins */
/* static const uint8_t ltc_cmdUNMUTE[4] = {0x00, 0x29, 0x63, 0x3C};                  !< UN-MUTE discharging via S pins */

/* LTC I2C commands */
/* static const uint8_t ltc_I2CcmdDummy[6] = {0x7F, 0xF9, 0x7F, 0xF9, 0x7F, 0xF9};      !< dummy command (no transmit) */

static const uint8_t ltc_I2CcmdTempSens0[6] = {0x69, 0x08, 0x00, 0x09, 0x7F, 0xF9};  /*!< sets the internal data pointer of the temperature sensor (address 0x48) to 0x00 */
static const uint8_t ltc_I2CcmdTempSens1[6] = {0x69, 0x18, 0x0F, 0xF0, 0x0F, 0xF9};  /*!< reads two data bytes from the temperature sensor */

static const uint8_t ltc_I2CcmdPortExpander1[6] = {0x64, 0x18, 0x0F, 0xF9, 0x7F, 0xF9};  /*!< reads one data byte from the port expander */

/* Cells */
static const uint8_t ltc_cmdADCV_normal_DCP0[4] = {0x03, 0x60, 0xF4, 0x6C};        /*!< All cells, normal mode, discharge not permitted (DCP=0)    */
static const uint8_t ltc_cmdADCV_normal_DCP1[4] = {0x03, 0x70, 0xAF, 0x42};        /*!< All cells, normal mode, discharge permitted (DCP=1)        */
static const uint8_t ltc_cmdADCV_filtered_DCP0[4] = {0x03, 0xE0, 0xB0, 0x4A};      /*!< All cells, filtered mode, discharge not permitted (DCP=0)  */
static const uint8_t ltc_cmdADCV_filtered_DCP1[4] = {0x03, 0xF0, 0xEB, 0x64};      /*!< All cells, filtered mode, discharge permitted (DCP=1)      */
static const uint8_t ltc_cmdADCV_fast_DCP0[4] = {0x02, 0xE0, 0x38, 0x06};          /*!< All cells, fast mode, discharge not permitted (DCP=0)      */
static const uint8_t ltc_cmdADCV_fast_DCP1[4] = {0x02, 0xF0, 0x63, 0x28};          /*!< All cells, fast mode, discharge permitted (DCP=1)          */
static const uint8_t ltc_cmdADCV_fast_DCP0_twocells[4] = {0x02, 0xE1, 0xb3, 0x34}; /*!< Two cells (1 and 7), fast mode, discharge not permitted (DCP=0) */

/* GPIOs  */
static const uint8_t ltc_cmdADAX_normal_GPIO1[4] = {0x05, 0x61, 0x58, 0x92};      /*!< Single channel, GPIO 1, normal mode   */
static const uint8_t ltc_cmdADAX_filtered_GPIO1[4] = {0x05, 0xE1, 0x1C, 0xB4};    /*!< Single channel, GPIO 1, filtered mode */
static const uint8_t ltc_cmdADAX_fast_GPIO1[4] = {0x04, 0xE1, 0x94, 0xF8};        /*!< Single channel, GPIO 1, fast mode     */
static const uint8_t ltc_cmdADAX_normal_GPIO2[4] = {0x05, 0x62, 0x4E, 0xF6};      /*!< Single channel, GPIO 2, normal mode   */
static const uint8_t ltc_cmdADAX_filtered_GPIO2[4] = {0x05, 0xE2, 0x0A, 0xD0};    /*!< Single channel, GPIO 2, filtered mode */
static const uint8_t ltc_cmdADAX_fast_GPIO2[4] = {0x04, 0xE2, 0x82, 0x9C};        /*!< Single channel, GPIO 2, fast mode     */
static const uint8_t ltc_cmdADAX_normal_GPIO3[4] = {0x05, 0x63, 0xC5, 0xC4};      /*!< Single channel, GPIO 3, normal mode   */
static const uint8_t ltc_cmdADAX_filtered_GPIO3[4] = {0x05, 0xE3, 0x81, 0xE2};    /*!< Single channel, GPIO 3, filtered mode */
static const uint8_t ltc_cmdADAX_fast_GPIO3[4] = {0x04, 0xE3, 0x09, 0xAE};        /*!< Single channel, GPIO 3, fast mode     */
/* static const uint8_t ltc_cmdADAX_normal_GPIO4[4] = {0x05, 0x64, 0x62, 0x3E};      !< Single channel, GPIO 4, normal mode   */
/* static const uint8_t ltc_cmdADAX_filtered_GPIO4[4] = {0x05, 0xE4, 0x26, 0x18};    !< Single channel, GPIO 4, filtered mode */
/* static const uint8_t ltc_cmdADAX_fast_GPIO4[4] = {0x04, 0xE4, 0xAE, 0x54};        !< Single channel, GPIO 4, fast mode     */
/* static const uint8_t ltc_cmdADAX_normal_GPIO5[4] = {0x05, 0x65, 0xE9, 0x0C};      !< Single channel, GPIO 5, normal mode   */
/* static const uint8_t ltc_cmdADAX_filtered_GPIO5[4] = {0x05, 0xE5, 0xAD, 0x2A};    !< Single channel, GPIO 5, filtered mode */
/* static const uint8_t ltc_cmdADAX_fast_GPIO5[4] = {0x04, 0xE5, 0x25, 0x66};        !< Single channel, GPIO 5, fast mode     */
static const uint8_t ltc_cmdADAX_normal_ALLGPIOS[4] = {0x05, 0x60, 0xD3, 0xA0};   /*!< All channels, normal mode             */
static const uint8_t ltc_cmdADAX_filtered_ALLGPIOS[4] = {0x05, 0xE0, 0x97, 0x86}; /*!< All channels, filtered mode           */
static const uint8_t ltc_cmdADAX_fast_ALLGPIOS[4] = {0x04, 0xE0, 0x1F, 0xCA};     /*!< All channels, fast mode               */

/* Open-wire */
static const uint8_t ltc2_BC_cmdADOW_PUP_normal_DCP0[4] = {0x03, 0x68, 0x1C, 0x62};    /*!< Broadcast, Pull-up current, All cells, normal mode, discharge not permitted (DCP=0)   */
static const uint8_t ltc2_BC_cmdADOW_PDOWN_normal_DCP0[4] = {0x03, 0x28, 0xFB, 0xE8};  /*!< Broadcast, Pull-down current, All cells, normal mode, discharge not permitted (DCP=0) */
static const uint8_t ltc2_BC_cmdADOW_PUP_filtered_DCP0[4] = {0x03, 0xE8, 0x1C, 0x62};    /*!< Broadcast, Pull-up current, All cells, filtered mode, discharge not permitted (DCP=0)   */
static const uint8_t ltc2_BC_cmdADOW_PDOWN_filtered_DCP0[4] = {0x03, 0xA8, 0xFB, 0xE8};  /*!< Broadcast, Pull-down current, All cells, filtered mode, discharge not permitted (DCP=0) */



LTC68XX_ERR ltc68xx_init(LTC68XXDriver *dev)
{
  LTC68XX_ERR result = LTC68XX_NOERROR;
  if(dev->state != LTC68XX_UNINIT){
    return LTC68XX_ERR_STATE;
  }
  
  // config all gpio pull down, refon, detn = 0, adcopt = 0
  uint8_t txBuf[10];
  txBuf[4] = 0xFC;
  txBuf[5] = 0x0;
  txBuf[6] = 0x0;
  txBuf[7] = 0x0;
  txBuf[8] = 0x0;
  txBuf[9] = 0x0;
  
  for(uint8_t i=0;i<4;i++)
  {
    txBuf[i] = ltc_cmdWRCFG[i];
  }
  
  dev->registerWrite(dev,txBuf,10,1);
  
  return 0;
}


LTC68XX_ERR ltc68xx_balanceControl(LTC68XXDriver *dev)
{
  LTC68XX_ERR result = LTC68XX_NOERROR;
  uint8_t txBuf[12];
  for(uint8_t i=0;i<4;i++)
  {
    txBuf[i] = ltc_cmdWRCFG[i];
  }
  txBuf[4] = 0xFC;
  txBuf[5] = 0x00;
  txBuf[6] = 0x00;
  txBuf[7] = 0x00;
  txBuf[8] = 0x00;
  txBuf[9] = 0x00;
  
  for(uint8_t i=0;i<8;i++){
    if(dev->balance->balancing_state[i] == 1){
      txBuf[8] |= ( 1 << i);
    }
  }
  for(uint8_t i=8;i<12;i++){
    if(dev->balance->balancing_state[i] == 1){
      txBuf[9] |= ( 1 << (i-8));
    }
  }
  
  uint16_t pec = LTC_pec15_calc(6,&txBuf[4]);
  txBuf[10] = (uint8_t)((pec >> 8) & 0xff);
  txBuf[11] = (uint8_t)(pec & 0xff);
  
  dev->registerWrite(dev,txBuf,12,1);
  
  return 0;
}

LTC68XX_ERR ltc68xx_startVMeas(LTC68XXDriver *dev, LTC_ADCMODE_e adcMode, LTC_ADCMEAS_CHAN_e adcMeasCh)
{
  LTC68XX_ERR result = LTC68XX_NOERROR;
  uint8_t txBuf[10];
  txBuf[4] = 0x00;
  txBuf[5] = 0x00;
  txBuf[6] = 0x00;
  txBuf[7] = 0x00;
  txBuf[8] = 0x00;
  txBuf[9] = 0x00;
  
  if(adcMeasCh == LTC_ADCMEAS_ALLCHANNEL){
    if(adcMode == LTC_ADCMODE_FAST_DCP0){
      memcpy(txBuf,ltc_cmdADCV_fast_DCP0,4);
      dev->registerWrite(dev,txBuf,4,1);
    } 
    else if(adcMode == LTC_ADCMODE_NORMAL_DCP0){
      memcpy(txBuf,ltc_cmdADCV_normal_DCP0,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
    else if(adcMode == LTC_ADCMODE_FILTERED_DCP0){
      memcpy(txBuf,ltc_cmdADCV_filtered_DCP0,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
    else if(adcMode == LTC_ADCMODE_FAST_DCP1){
      memcpy(txBuf,ltc_cmdADCV_fast_DCP1,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
    else if(adcMode == LTC_ADCMODE_NORMAL_DCP1){
      memcpy(txBuf,ltc_cmdADCV_normal_DCP1,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
    else if(adcMode == LTC_ADCMODE_FILTERED_DCP1){
      memcpy(txBuf,ltc_cmdADCV_filtered_DCP1,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
    else{
      result = -1;
    }
  }
  else if(adcMeasCh == LTC_ADCMEAS_SINGLECHANNEL_TWOCELLS){
    if (adcMode == LTC_ADCMODE_FAST_DCP0) {
      memcpy(txBuf,ltc_cmdADCV_fast_DCP0_twocells,4);
      dev->registerWrite(dev,txBuf,4,1);
    } else {
        result = -1;
    }
  }
  else{
    result = -1;
  }

  
  return result;
}

LTC68XX_ERR ltc68xx_startGPIOVMeas(LTC68XXDriver *dev, LTC_ADCMODE_e adcMode, LTC_ADCMEAS_CHAN_e adcMeasCh)
{
  LTC68XX_ERR result = LTC68XX_NOERROR;
  uint8_t txBuf[10];
  txBuf[4] = 0x00;
  txBuf[5] = 0x00;
  txBuf[6] = 0x00;
  txBuf[7] = 0x00;
  txBuf[8] = 0x00;
  txBuf[9] = 0x00;
  
  if(adcMeasCh == LTC_ADCMEAS_ALLCHANNEL){
    if(adcMode == LTC_ADCMODE_FAST_DCP0 || adcMode == LTC_ADCMODE_FAST_DCP1){
      memcpy(txBuf,ltc_cmdADAX_fast_ALLGPIOS,4);
      dev->registerWrite(dev,txBuf,4,1);
    } 
    else if(adcMode == LTC_ADCMODE_NORMAL_DCP0 ||adcMode == LTC_ADCMODE_NORMAL_DCP1){
      memcpy(txBuf,ltc_cmdADAX_filtered_ALLGPIOS,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
    else{
      memcpy(txBuf,ltc_cmdADAX_normal_ALLGPIOS,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
  }
  else if(adcMeasCh == LTC_ADCMEAS_SINGLECHANNEL_GPIO1){
    if(adcMode == LTC_ADCMODE_FAST_DCP0 || adcMode == LTC_ADCMODE_FAST_DCP1){
      memcpy(txBuf,ltc_cmdADAX_fast_GPIO1,4);
      dev->registerWrite(dev,txBuf,4,1);
    } 
    else if(adcMode == LTC_ADCMODE_NORMAL_DCP0 ||adcMode == LTC_ADCMODE_NORMAL_DCP1){
      memcpy(txBuf,ltc_cmdADAX_filtered_GPIO1,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
    else{
      memcpy(txBuf,ltc_cmdADAX_normal_GPIO1,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
  }
  else if(adcMeasCh == LTC_ADCMEAS_SINGLECHANNEL_GPIO2){
    if(adcMode == LTC_ADCMODE_FAST_DCP0 || adcMode == LTC_ADCMODE_FAST_DCP1){
      memcpy(txBuf,ltc_cmdADAX_fast_GPIO2,4);
      dev->registerWrite(dev,txBuf,4,1);
    } 
    else if(adcMode == LTC_ADCMODE_NORMAL_DCP0 ||adcMode == LTC_ADCMODE_NORMAL_DCP1){
      memcpy(txBuf,ltc_cmdADAX_filtered_GPIO2,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
    else{
      memcpy(txBuf,ltc_cmdADAX_normal_GPIO2,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
  }
  else if(adcMeasCh == LTC_ADCMEAS_SINGLECHANNEL_GPIO3){
    if(adcMode == LTC_ADCMODE_FAST_DCP0 || adcMode == LTC_ADCMODE_FAST_DCP1){
      memcpy(txBuf,ltc_cmdADAX_fast_GPIO3,4);
      dev->registerWrite(dev,txBuf,4,1);
    } 
    else if(adcMode == LTC_ADCMODE_NORMAL_DCP0 ||adcMode == LTC_ADCMODE_NORMAL_DCP1){
      memcpy(txBuf,ltc_cmdADAX_filtered_GPIO3,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
    else{
      memcpy(txBuf,ltc_cmdADAX_normal_GPIO3,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
  }
  else{
    result = -1;
  }

  
  return result;
}

LTC68XX_ERR ltc68xx_startOpenWireMeas(LTC68XXDriver *dev, LTC_ADCMODE_e adcMode, uint8_t PUP)
{
  LTC68XX_ERR result = LTC68XX_NOERROR;
  uint8_t txBuf[10];
  txBuf[4] = 0x00;
  txBuf[5] = 0x00;
  txBuf[6] = 0x00;
  txBuf[7] = 0x00;
  txBuf[8] = 0x00;
  txBuf[9] = 0x00;
  
  if(PUP == 0){
    if(adcMode == LTC_ADCMODE_NORMAL_DCP0){
      memcpy(txBuf,ltc2_BC_cmdADOW_PDOWN_normal_DCP0,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
    else if(adcMode == LTC_ADCMODE_FILTERED_DCP0){
      memcpy(txBuf,ltc2_BC_cmdADOW_PDOWN_filtered_DCP0,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
    else{
      result = -1;
    }
  }
  else if(PUP == 1){
    if(adcMode == LTC_ADCMODE_NORMAL_DCP0){
      memcpy(txBuf,ltc2_BC_cmdADOW_PUP_normal_DCP0,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
    else if(adcMode == LTC_ADCMODE_FILTERED_DCP0){
      memcpy(txBuf,ltc2_BC_cmdADOW_PUP_filtered_DCP0,4);
      dev->registerWrite(dev,txBuf,4,1);
    }
    else{
      result = -1;
    }
  }
  else if(PUP == 0xff){
    memcpy(txBuf,ltc_cmdADCV_normal_DCP0,4);
    dev->registerWrite(dev,txBuf,4,1);
  }
  
 
  return result;
}


LTC68XX_ERR ltc68xx_RX(LTC68XXDriver *dev, uint8_t *command, uint8_t *dptr)
{
  LTC68XX_ERR result = LTC68XX_NOERROR;
  
  for(uint8_t i=0;i<6;i++){
    dptr[i] = 0x00;
  }
  
  dev->exchangeData(dev,command,4,dptr,8);
  
   
  return result;
}

LTC68XX_ERR ltc68xx_ReadVoltage(LTC68XXDriver *dev, uint8_t group, uint8_t *dptr)
{
  LTC68XX_ERR result = LTC68XX_NOERROR; 
  uint8_t txBuf[4];
  switch(group){
  case 0:
    memcpy(txBuf,ltc_cmdRDCVA,4);
    break;
  case 1:
    memcpy(txBuf,ltc_cmdRDCVB,4);
    break;
  case 2:
    memcpy(txBuf,ltc_cmdRDCVC,4);
    break;
  case 3:
    memcpy(txBuf,ltc_cmdRDCVD,4);
    break;
  case 4:
    memcpy(txBuf,ltc_cmdRDCVE,4);
    break;
  case 5:
    memcpy(txBuf,ltc_cmdRDCVF,4);
    break;
  default:
    return -1;
  }
  
  return ltc68xx_RX(dev,txBuf,dptr);
}

LTC68XX_ERR ltc68xx_ReadAUX(LTC68XXDriver *dev, uint8_t group, uint8_t *dptr)
{
  LTC68XX_ERR result = LTC68XX_NOERROR; 
  uint8_t txBuf[4];
  switch(group){
  case 0:
    memcpy(txBuf,ltc_cmdRDAUXA,4);
    break;
  case 1:
    memcpy(txBuf,ltc_cmdRDAUXB,4);
    break;
  case 2:
    memcpy(txBuf,ltc_cmdRDAUXC,4);
    break;
  case 3:
    memcpy(txBuf,ltc_cmdRDAUXD,4);
    break;
  default:
    return -1;
  }
  
  return ltc68xx_RX(dev,txBuf,dptr);
}

