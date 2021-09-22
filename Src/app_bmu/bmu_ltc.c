#include "ch.h"
#include "hal.h"
#include "bmu_ltc.h"
#include "ltc68xx.h"
#include "ltc_cfg.h"
#include "nvm.h"

#define LTC_N_LTC       1
#define BS_NR_OF_BAT_CELLS 12
#define BS_NR_OF_BAT_CELLS_PER_MODULE 12

#define BS_NR_OF_GPIOS_PER_MODULE 5

#define LTC_SAVELASTSTATES()    bmuState.lastState = bmuState.state; \
                                bmuState.lastSubState = bmuState.subState

#define LTC_ResetErrorTable()   bmuState.errTable.PEC_valid = 0; \
                                bmuState.errTable.mux0 = 0; \
                                bmuState.errTable.mux1 = 0; \
                                bmuState.errTable.mux2 = 0; \
                                bmuState.errTable.mux3 = 0  

static thread_t *thisThd;
static thread_reference_t thd_trp = NULL;

static int8_t registerRead(LTC68XXDriver *dev, uint8_t *b, uint16_t n, uint8_t cs);
static int8_t registerWrite(LTC68XXDriver *dev, uint8_t *b, uint16_t n, uint8_t cs);
int8_t dataExchange(LTC68XXDriver *dev, uint8_t *tx, uint16_t ntx, uint8_t *rx, uint16_t nrx);



static const SPIConfig spicfg = {
  false,
  NULL,
  GPIOB,
  12,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
};
const LTC68xxConfig ltcConfig = {
  &SPID2,
  &spicfg
};

_balancing_control_s balance_control;
//= {
//
//};

LTC68XXDriver ltc68xx = {
  LTC68XX_UNINIT,
  &ltcConfig,
  &balance_control,
  registerRead,
  registerWrite,
  dataExchange
};


static _bmu_state_s bmuState = {
    .timer = 0,
    .statereq = LTC_STATE_NO_REQUEST,
    .state = LTC_STATEMACH_UNINITIALIZED,
    .subState = 0,
    .lastState = LTC_STATEMACH_UNINITIALIZED,
    .lastSubState = 0,
    .adcModereq = LTC_ADCMODE_FAST_DCP0,
    .adcMode = LTC_ADCMODE_FAST_DCP0,
    .adcMeasChreq = LTC_ADCMEAS_UNDEFINED,
    .adcMeasCh = LTC_ADCMEAS_UNDEFINED,
    .triggerentry = 0,
    .errRetryCounter = 0,
    .errRequestCounter = 0,
    .voltageSampleTime = 0,
    .commandDataTransferTime = 3,
    .commandTransferTime = 3,
    .gpioClocksTransferTime = 3,
    .firstMeasMade = FALSE,
    .balance_control_done = FALSE,
};

static _cmu_nvm_s cmuNVM;

/**
 * @brief   checks the state requests that are made.
 *
 * This function checks the validity of the state requests.
 * The resuls of the checked is returned immediately.
 *
 * @param   statereq    state request to be checked
 *
 * @return              result of the state request that was made, taken from LTC_RETURN_TYPE_e
 */
static LTC_RETURN_TYPE_e LTC_CheckStateRequest(LTC_STATE_REQUEST_e statereq) {
    if (bmuState.statereq == LTC_STATE_NO_REQUEST) {
        /* init only allowed from the uninitialized state */
        if (statereq == LTC_STATE_INIT_REQUEST) {
            if (bmuState.state == LTC_STATEMACH_UNINITIALIZED) {
                return LTC_OK;
            } else {
                return LTC_ALREADY_INITIALIZED;
            }
        }

        return LTC_OK;

    } else {
        return LTC_REQUEST_PENDING;
    }
}

LTC_RETURN_TYPE_e LTC_SetStateRequest(LTC_STATE_REQUEST_e statereq) {
    LTC_RETURN_TYPE_e retVal = LTC_STATE_NO_REQUEST;

    chSysLock();
    retVal = LTC_CheckStateRequest(statereq);

    if (retVal == LTC_OK || retVal == LTC_BUSY_OK || retVal == LTC_OK_FROM_ERROR) {
            bmuState.statereq   = statereq;
        }
    chSysUnlock();

    return (retVal);
}

LTC_STATE_REQUEST_e LTC_TransferStateRequest(uint8_t *busIDptr, LTC_ADCMODE_e *adcModeptr, LTC_ADCMEAS_CHAN_e *adcMeasChptr) {
    LTC_STATE_REQUEST_e retval = LTC_STATE_NO_REQUEST;

    chSysLock();
    retval = bmuState.statereq;
    *adcModeptr = bmuState.adcModereq;
    *adcMeasChptr = bmuState.adcMeasChreq;
    bmuState.statereq = LTC_STATE_NO_REQUEST;
    chSysUnlock();

    return (retval);
}

/**
 * @brief   saves the voltage values read from the LTC daisy-chain.
 *
 * After a voltage measurement was initiated to measure the voltages of the cells,
 * the result is read via SPI from the daisy-chain.
 * There are 6 register to read _(A,B,C,D,E,F) to get all cell voltages.
 * Only one register can be read at a time.
 * This function is called to store the result from the transmission in a buffer.
 *
 * @param   registerSet    voltage register that was read (voltage register A,B,C,D,E or F)
 * @param   *rxBuffer      buffer containing the data obtained from the SPI transmission
 * @param   PEC_valid      tells the functions if the PEC is valid or not, if not, manage indices but do not store
 *
 */
static void LTC_SaveRXtoVoltagebuffer(uint8_t registerSet, uint8_t *rxBuffer) {
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t i_offset = 0;
    uint16_t voltage_index = 0;
    uint16_t val_ui = 0;
    uint16_t voltage = 0;
    uint8_t incrementations = 0;
    uint32_t bitmask = 0;
    uint16_t cellVoltage[12];
    /* reinitialize index counter at begin of cycle */

    if (registerSet == 0) {
    /* RDCVA command -> voltage register group A */
        i_offset = 0;
    } else if (registerSet == 1) {
    /* RDCVB command -> voltage register group B */
        i_offset = 3;
    } else if (registerSet == 2) {
    /* RDCVC command -> voltage register group C */
        i_offset = 6;
    } else if (registerSet == 3) {
    /* RDCVD command -> voltage register group D */
        i_offset = 9;
    } else if (registerSet == 4) {
    /* RDCVD command -> voltage register group E (only for 15 and 18 cell version) */
        i_offset = 12;
    } else if (registerSet == 5) {
    /* RDCVD command -> voltage register group F (only for 18 cell version) */
        i_offset = 15;
    } else {
        return;
    }

    /* Calculate bitmask for valid flags */
    bitmask |= 0x07 << i_offset;    /* 0x07: three voltages in each register */

    if (i_offset == 0) {
        bmuState.usedIndex = 0;
    }

    /* Retrieve data without command and CRC*/
//    for (i=0; i < LTC_N_LTC; i++) {
        i = 0;
        incrementations = 0;

        /* parse all three voltages (3 * 2bytes) contained in one register */
        for (j=0; j < 3; j++) {
            /* index considering maximum number of cells */
            voltage_index = j+i_offset;
//            voltage_index = bmuState.usedIndex;
//            if ((ltc_voltage_input_used[voltage_index] == 1) && (ltc_used_cells_index < BS_NR_OF_BAT_CELLS_PER_MODULE)) {
            if(bmuState.cells.activeCellQue[voltage_index] == 1){
                val_ui = *((uint16_t *)(&rxBuffer[2*j+i*8]));
                voltage = (uint16_t)(((float)(val_ui))*100e-6f*1000.0f);        /* Unit V -> in mV */
                /* Check PEC for every LTC in the daisy-chain */
                if (bmuState.errTable.PEC_valid == TRUE) {
                    //bmuState.cells.voltage[bmuState.usedIndex+i*(BS_NR_OF_BAT_CELLS_PER_MODULE)] = voltage;
                  if(bmuState.reusageMeasurementMode == LTC_REUSE_READVOLT_FOR_ADOW_PUP){
                    bmuState.cells.pullupFeedback[voltage_index] = voltage;
                  }
//                  else if(bmuState.reusageMeasurementMode == LTC_READ_VOLTAGES_PULLDOWN_OPENWIRE_CHECK){
//                    bmuState.cells.pulldnFeedback[voltage_index] = voltage;
//                  }
                  else if(bmuState.reusageMeasurementMode == LTC_REUSE_READVOLT_FOR_ADOW_PDOWN){
                    bmuState.cells.pulldnFeedback[voltage_index] = voltage;
                  }
                  else{
                    bmuState.cells.voltage[voltage_index] = voltage;
                    bitmask = ~bitmask;  /* negate bitmask to only validate flags of this voltage register */
                    bmuState.cells.validVolt &= bitmask;
                  }
                } else {
                    /* PEC_valid == FALSE: Invalidate only flags of this voltage register */
                    bmuState.cells.validVolt |= bitmask;
                }

                bmuState.usedIndex++;
                incrementations++;

                if (bmuState.usedIndex > BS_NR_OF_BAT_CELLS_PER_MODULE) {
                    return;
                }
            }
        }
        /* restore start value for next module */
#pragma GCC diagnostic push
        /* This warning is allowed for the edge case
         * LTC_N_LTC == 1
         */
#pragma GCC diagnostic ignored "-Wtype-limits"
        if (i < LTC_N_LTC-1) {
#pragma GCC diagnostic pop
            bmuState.usedIndex -= incrementations;
        }
//    }
      
        nvm_runtime_set_cellVoltage(bmuState.cells.voltage);
        
}

/**
 * @brief   saves the GPIO voltage values read from the LTC daisy-chain.
 *
 * After a voltage measurement was initiated to measure the voltages on all GPIOs,
 * the result is read via SPI from the daisy-chain. In order to read the result of all GPIO measurements,
 * it is necessary to read auxiliary register A and B.
 * Only one register can be read at a time.
 * This function is called to store the result from the transmission in a buffer.
 *
 * @param   registerSet    voltage register that was read (auxiliary register A, B, C or D)
 * @param   *rxBuffer      buffer containing the data obtained from the SPI transmission
 *
 */
static void LTC_SaveRXtoGPIOBuffer(uint8_t registerSet, uint8_t *rxBuffer) {
    uint16_t i = 0;
    uint8_t i_offset = 0;
    uint32_t bitmask = 0;
    static uint16_t gpioVoltage[5];

    if (registerSet == 0) {
    /* RDAUXA command -> GPIO register group A */
        i_offset = 0;
        bitmask = 0x07 << i_offset;  /* 0x07: three temperatures in this register */
        /* Retrieve data without command and CRC*/
        i = 0;
       // for (i = 0; i < LTC_N_LTC; i++) {
            /* Check if PEC is valid */
            if (bmuState.errTable.PEC_valid == TRUE) {
                bitmask = ~bitmask; /* negate bitmask to only validate flags of this voltage register */
                bmuState.cells.validGpioVolt &= bitmask;
                /* values received in 100uV -> divide by 10 to convert to mV */
                bmuState.cells.gpioVoltage[0 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[0+i*8]))/10;
                bmuState.cells.gpioVoltage[1 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[2+i*8]))/10;
                bmuState.cells.gpioVoltage[2 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[4+i*8]))/10;
                gpioVoltage[0 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[0+i*8]))/10;
                gpioVoltage[1 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[2+i*8]))/10;
                gpioVoltage[2 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[4+i*8]))/10;
            } else {
                bmuState.cells.validGpioVolt |= bitmask;
            }
       // }
    } else if (registerSet == 1) {
        /* RDAUXB command -> GPIO register group B */
        i_offset = 3;
        bitmask = 0x03 << i_offset;  /* 0x03: two temperatures in this register */
        /* Retrieve data without command and CRC*/
        i = 0;
//        for (i = 0; i < LTC_N_LTC; i++) {
            /* Check if PEC is valid */
            if (bmuState.errTable.PEC_valid == TRUE) {
                bitmask = ~bitmask; /* negate bitmask to only validate flags of this voltage register */
                bmuState.cells.validGpioVolt &= bitmask;
                /* values received in 100uV -> divide by 10 to convert to mV */
                bmuState.cells.gpioVoltage[0 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[0+i*8]))/10;
                bmuState.cells.gpioVoltage[1 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[2+i*8]))/10;
                gpioVoltage[0 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[0+i*8]))/10;
                gpioVoltage[1 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[2+i*8]))/10;
            } else {
                bmuState.cells.validGpioVolt |= bitmask;
            }
//        }
      nvm_runtime_set_gpioVoltage(gpioVoltage);
    } 
//    else if (registerSet == 2) {
//        /* RDAUXC command -> GPIO register group C, for 18 cell version */
//        i_offset = 5;
//        bitmask = 0x07 << i_offset;  /* 0x07: three temperatures in this register */
//        /* Retrieve data without command and CRC*/
//        for (i = 0; i < LTC_N_LTC; i++) {
//            /* Check if PEC is valid */
//            if (LTC_ErrorTable[i].PEC_valid == TRUE) {
//                bitmask = ~bitmask; /* negate bitmask to only validate flags of this voltage register */
//                ltc_allgpiovoltage.valid_gpiovoltages[i] &= bitmask;
//                /* values received in 100uV -> divide by 10 to convert to mV */
//                ltc_allgpiovoltage.gpiovoltage[0 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[4+i*8]))/10;
//                ltc_allgpiovoltage.gpiovoltage[1 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[6+i*8]))/10;
//                ltc_allgpiovoltage.gpiovoltage[2 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[8+i*8]))/10;
//            } else {
//                ltc_allgpiovoltage.valid_gpiovoltages[i] |= bitmask;
//            }
//        }
//    } else if (registerSet == 3) {
//        /* RDAUXD command -> GPIO register group D, for 18 cell version */
//        i_offset = 8;
//        bitmask = 0x01 << i_offset;  /* 0x01: one temperature in this register */
//        /* Retrieve data without command and CRC*/
//        for (i = 0; i < LTC_N_LTC; i++) {
//            /* Check if PEC is valid */
//            if (LTC_ErrorTable[i].PEC_valid == TRUE) {
//                bitmask = ~bitmask; /* negate bitmask to only validate flags of this voltage register */
//                ltc_allgpiovoltage.valid_gpiovoltages[i] &= bitmask;
//                /* values received in 100uV -> divide by 10 to convert to mV */
//                ltc_allgpiovoltage.gpiovoltage[0 + i_offset + BS_NR_OF_GPIOS_PER_MODULE*i]= *((uint16_t *)(&rxBuffer[4+i*8]))/10;
//            } else {
//                ltc_allgpiovoltage.valid_gpiovoltages[i] |= bitmask;
//            }
//        }
//    } 
    else {
        return;
    }
}

/**
 * @brief   stores the measured balancing feedback values in the database.
 *
 * This function stores the global balancing feedback value measured on GPIO3 of the LTC into the database
 *
 */
static void LTC_SaveBalancingFeedback(uint8_t *DataBufferSPI_RX) {
    uint16_t i = 0;
    uint16_t val_i = 0;

    //for (i=0; i < LTC_N_LTC; i++) {
        val_i = DataBufferSPI_RX[8+1*i*8] | (DataBufferSPI_RX[8+1*i*8+1] << 8);    /* raw value, GPIO3 */

            bmuState.cells.balanceFeedback[i] = val_i;
    //}

//    ltc_balancing_feedback.state++;
    //DB_WriteBlock(&ltc_balancing_feedback, DATA_BLOCK_ID_BALANCING_FEEDBACK_VALUES);
}

/**
 * @brief   function for setting LTC_Trigger state transitions
 *
 * @param  state:    state to transition into
 * @param  substate: substate to transition into
 * @param  timer_ms: transition into state, substate after timer elapsed
 */
static void LTC_StateTransition(LTC_STATEMACH_e state, uint8_t substate, uint16_t timer_ms) {
        bmuState.state = state;
        bmuState.subState = substate;
        bmuState.timer = timer_ms;
}

/**
 * @brief   condition-based state transition depending on retVal
 *
 * If retVal is E_OK, after timer_ms_ok is elapsed the LTC statemachine will
 * transition into state_ok and substate_ok, otherwise after timer_ms_nok the
 * statemachine will transition to state_nok and substate_nok. Depending on
 * value of retVal the corresponding diagnosis entry will be called.
 *
 * @param  retVal:       condition to determine if statemachine will transition into ok or nok states
 * @param  diagCode:     symbolic IDs for diagnosis entry, called with DIAG_EVENT_OK if retVal is E_OK, DIAG_EVENT_NOK otherwise
 * @param  state_ok      state to transition into if retVal is E_OK
 * @param  substate_ok:  substate to transition into if retVal is E_OK
 * @param  timer_ms_ok:  transition into state_ok, substate_ok after timer_ms_ok elapsed
 * @param  state_nok:    state to transition into if retVal is E_NOT_OK
 * @param  substate_nok: substate to transition into if retVal is E_NOT_OK
 * @param  timer_ms_nok: transition into state_nok, substate_nok after timer_ms_nok elapsed
 */
//static void LTC_CondBasedStateTransition(STD_RETURN_TYPE_e retVal, DIAG_CH_ID_e diagCode, uint8_t state_ok, uint8_t substate_ok, uint16_t timer_ms_ok, uint8_t state_nok, uint8_t substate_nok, uint16_t timer_ms_nok) {
//    if ((retVal != E_OK)) {
//        DIAG_Handler(diagCode, DIAG_EVENT_NOK, 0);
//        LTC_StateTransition(state_nok, substate_nok, timer_ms_nok);
//    } else {
//        DIAG_Handler(diagCode, DIAG_EVENT_OK, 0);
//        LTC_StateTransition(state_ok, substate_ok, timer_ms_ok);
//    }
//}
/**
 * @brief   brief missing
 *
 * Gets the measurement time needed by the LTC chip, depending on the measurement mode and the number of channels.
 * For all cell voltages or all 5 GPIOS, the measurement time is the same.
 * For 2 cell voltages or 1 GPIO, the measurement time is the same.
 * As a consequence, this function is used for cell voltage and for GPIO measurement.
 *
 * @param   adcMode     LTC ADCmeasurement mode (fast, normal or filtered)
 * @param   adcMeasCh   number of channels measured for GPIOS (one at a time for multiplexers or all five GPIOs)
 *                      or number of cell voltage measured (2 cells or all cells)
 *
 * @return  retVal      measurement time in ms
 */
static uint16_t LTC_Get_MeasurementTCycle(LTC_ADCMODE_e adcMode, LTC_ADCMEAS_CHAN_e  adcMeasCh) {
    uint16_t retVal = LTC_STATEMACH_MEAS_ALL_NORMAL_TCYCLE;  /* default */

    if (adcMeasCh == LTC_ADCMEAS_ALLCHANNEL) {
        if (adcMode == LTC_ADCMODE_FAST_DCP0 || adcMode == LTC_ADCMODE_FAST_DCP1) {
            retVal = LTC_STATEMACH_MEAS_ALL_FAST_TCYCLE;
        } else if (adcMode == LTC_ADCMODE_NORMAL_DCP0 || adcMode == LTC_ADCMODE_NORMAL_DCP1) {
            retVal = LTC_STATEMACH_MEAS_ALL_NORMAL_TCYCLE;
        } else if (adcMode == LTC_ADCMODE_FILTERED_DCP0 || adcMode == LTC_ADCMODE_FILTERED_DCP1) {
            retVal = LTC_STATEMACH_MEAS_ALL_FILTERED_TCYCLE;
        }
    } else if (adcMeasCh == LTC_ADCMEAS_SINGLECHANNEL_GPIO1 || adcMeasCh == LTC_ADCMEAS_SINGLECHANNEL_GPIO2
            || adcMeasCh == LTC_ADCMEAS_SINGLECHANNEL_GPIO3 || adcMeasCh == LTC_ADCMEAS_SINGLECHANNEL_GPIO4
            || adcMeasCh == LTC_ADCMEAS_SINGLECHANNEL_GPIO5 || adcMeasCh == LTC_ADCMEAS_SINGLECHANNEL_TWOCELLS) {
        if (adcMode == LTC_ADCMODE_FAST_DCP0 || adcMode == LTC_ADCMODE_FAST_DCP1) {
            retVal = LTC_STATEMACH_MEAS_SINGLE_FAST_TCYCLE;
        } else if (adcMode == LTC_ADCMODE_NORMAL_DCP0 || adcMode == LTC_ADCMODE_NORMAL_DCP1) {
            retVal = LTC_STATEMACH_MEAS_SINGLE_NORMAL_TCYCLE;
        } else if (adcMode == LTC_ADCMODE_FILTERED_DCP0 || adcMode == LTC_ADCMODE_FILTERED_DCP1) {
            retVal = LTC_STATEMACH_MEAS_SINGLE_FILTERED_TCYCLE;
        }
    } else {
        retVal = LTC_STATEMACH_MEAS_ALL_NORMAL_TCYCLE;
    }

    return retVal;
}

/**
 * @brief   checks if the data received from the daisy-chain is not corrupt.
 *
 * This function computes the PEC (CRC) from the data received by the daisy-chain.
 * It compares it with the PEC sent by the LTCs.
 * If there are errors, the array LTC_ErrorTable is updated to locate the LTCs in daisy-chain
 * that transmitted corrupt data.
 *
 * @param   *DataBufferSPI_RX_with_PEC   data obtained from the SPI transmission
 *
 * @return  retVal                       E_OK if PEC check is OK, E_NOT_OK otherwise
 *
 */
static STD_RETURN_TYPE_e LTC_RX_PECCheck(uint8_t *DataBufferSPI_RX_with_PEC) {
    uint16_t i = 0;
    STD_RETURN_TYPE_e retVal = E_OK;
    uint8_t PEC_TX[2];
    uint16_t PEC_result = 0;
    uint8_t PEC_Check[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    /* check all PECs and put data without command and PEC in DataBufferSPI_RX (easier to use) */
    for (i=0; i < LTC_N_LTC; i++) {
        PEC_Check[0] = DataBufferSPI_RX_with_PEC[0+i*8];
        PEC_Check[1] = DataBufferSPI_RX_with_PEC[1+i*8];
        PEC_Check[2] = DataBufferSPI_RX_with_PEC[2+i*8];
        PEC_Check[3] = DataBufferSPI_RX_with_PEC[3+i*8];
        PEC_Check[4] = DataBufferSPI_RX_with_PEC[4+i*8];
        PEC_Check[5] = DataBufferSPI_RX_with_PEC[5+i*8];

        PEC_result = LTC_pec15_calc(6, PEC_Check);
        PEC_TX[0]=(uint8_t)((PEC_result>>8)&0xff);
        PEC_TX[1]=(uint8_t)(PEC_result&0xff);

        /* if calculated PEC not equal to received PEC */
        if ((PEC_TX[0] != DataBufferSPI_RX_with_PEC[6+i*8]) || (PEC_TX[1] != DataBufferSPI_RX_with_PEC[7+i*8])) {
            /* update error table of the corresponding LTC only if PEC check is activated */
            if (LTC_DISCARD_PEC == FALSE) {
              bmuState.errTable.PEC_valid = FALSE;
//                LTC_ErrorTable[i].PEC_valid = FALSE;
            }
            retVal = E_NOT_OK;

        } else {
            /* update error table of the corresponding LTC */
            bmuState.errTable.PEC_valid = TRUE;
//            LTC_ErrorTable[i].PEC_valid = TRUE;
        }
    }

    if (LTC_DISCARD_PEC == TRUE) {
        return E_OK;
    } else {
        return (retVal);
    }
}

static virtual_timer_t vtMeas;
static uint32_t measCntr; 
static void meas_cb(void *arg)
{
  chSysLockFromISR();
  measCntr++;
  if(measCntr == 5){
    measCntr = 0;
    LTC_SetStateRequest(LTC_STATE_OPENWIRE_CHECK_REQUEST);
  }else{
    LTC_SetStateRequest(LTC_STATE_ALLGPIOMEASUREMENT_REQUEST);
  }
  chVTSetI(&vtMeas,TIME_MS2I(100),meas_cb,NULL);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waBMU, 1024);
static THD_FUNCTION(procBMU, arg) 
{
  (void)arg;
  STD_RETURN_TYPE_e retVal = E_OK;
  LTC_STATE_REQUEST_e statereq = LTC_STATE_NO_REQUEST;
  uint8_t tmpbusID = 0;
  LTC_ADCMODE_e tmpadcMode = LTC_ADCMODE_UNDEFINED;
  LTC_ADCMEAS_CHAN_e tmpadcMeasCh = LTC_ADCMEAS_UNDEFINED;
  static uint8_t ltc_RXPECbuffer[24];

  measCntr = 0;
  chVTObjectInit(&vtMeas);
  chVTSet(&vtMeas,TIME_MS2I(100),meas_cb,NULL);

  //ltc68xx_init(&ltc68xx);
  LTC_SetStateRequest(LTC_STATE_INIT_REQUEST);
  
  chThdResume(&thd_trp, MSG_OK);

  while(1){
    switch(bmuState.state){
    case LTC_STATEMACH_UNINITIALIZED:
      statereq = LTC_TransferStateRequest(&tmpbusID,&tmpadcMode,&tmpadcMeasCh);
      if(statereq == LTC_STATE_INIT_REQUEST){
//        LTC_StateTransition(LTC_STATEMACH_INITIALIZATION, LTC_ENTRY_UNINITIALIZED, LTC_STATEMACH_SHORTTIME);
        LTC_StateTransition(LTC_STATEMACH_INITIALIZATION, LTC_START_INIT_INITIALIZATION, LTC_STATEMACH_SHORTTIME);
        bmuState.adcMode = tmpadcMode;
        bmuState.adcMeasCh = tmpadcMeasCh;
      }
      else if(statereq == LTC_STATE_NO_REQUEST){
        
      }
      else{
        bmuState.errPECCounter++;
      }
      break;
    case LTC_STATEMACH_INITIALIZATION:
      if(bmuState.subState == LTC_ENTRY_INITIALIZATION){
        
      }
      else if(bmuState.state == LTC_RE_ENTRY_INITIALIZATION){
        
      }
      else if(bmuState.subState == LTC_START_INIT_INITIALIZATION){
        // driver initialize
        ltc68xx_init(&ltc68xx);
        bmuState.lastSubState = bmuState.subState;
        LTC_StateTransition(LTC_STATEMACH_INITIALIZATION, LTC_EXIT_INITIALIZATION, bmuState.commandDataTransferTime);
      }
      else if(bmuState.subState == LTC_EXIT_INITIALIZATION){
        // reset variables
        LTC_SAVELASTSTATES();
        LTC_ResetErrorTable();
        LTC_StateTransition(LTC_STATEMACH_INITIALIZED, LTC_ENTRY_INITIALIZATION, LTC_STATEMACH_SHORTTIME);
      }
      break;
    case LTC_STATEMACH_INITIALIZED:
//      LTC_CondBasedStateTransition(E_OK, DIAG_CH_LTC_CONFIG,
//              LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME,
//              LTC_STATEMACH_INITIALIZED, LTC_ENTRY_INITIALIZATION, LTC_STATEMACH_SHORTTIME);
      LTC_SAVELASTSTATES();
      LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
      break;
    case LTC_STATEMACH_STARTMEAS:
      bmuState.adcMode = LTC_VOLTAGE_MEASUREMENT_MODE;
      bmuState.adcMeasCh = LTC_ADCMEAS_ALLCHANNEL;
      retVal = ltc68xx_startVMeas(&ltc68xx,bmuState.adcMode, bmuState.adcMeasCh);
      LTC_StateTransition(LTC_STATEMACH_READVOLTAGE, LTC_READ_VOLTAGE_REGISTER_A_RDCVA_READVOLTAGE, (bmuState.commandTransferTime + LTC_Get_MeasurementTCycle(bmuState.adcMode, bmuState.adcMeasCh)));
      break;
    case LTC_STATEMACH_READVOLTAGE:
      if (bmuState.subState == LTC_READ_VOLTAGE_REGISTER_A_RDCVA_READVOLTAGE) {
        retVal = ltc68xx_ReadVoltage(&ltc68xx,0,ltc_RXPECbuffer);
        LTC_StateTransition(LTC_STATEMACH_READVOLTAGE, LTC_READ_VOLTAGE_REGISTER_B_RDCVB_READVOLTAGE, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT));
        break;
      } 
      else if (bmuState.subState == LTC_READ_VOLTAGE_REGISTER_B_RDCVB_READVOLTAGE) {
          retVal = LTC_RX_PECCheck(ltc_RXPECbuffer);
          LTC_SaveRXtoVoltagebuffer(0, ltc_RXPECbuffer);
          retVal = ltc68xx_ReadVoltage(&ltc68xx,1,ltc_RXPECbuffer);
          LTC_StateTransition(LTC_STATEMACH_READVOLTAGE, LTC_READ_VOLTAGE_REGISTER_C_RDCVC_READVOLTAGE, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT));
          break;

      } else if (bmuState.subState == LTC_READ_VOLTAGE_REGISTER_C_RDCVC_READVOLTAGE) {
          retVal = LTC_RX_PECCheck(ltc_RXPECbuffer);
          LTC_SaveRXtoVoltagebuffer(1, ltc_RXPECbuffer);
          retVal = ltc68xx_ReadVoltage(&ltc68xx,2,ltc_RXPECbuffer);
          LTC_StateTransition(LTC_STATEMACH_READVOLTAGE, LTC_READ_VOLTAGE_REGISTER_D_RDCVD_READVOLTAGE, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT));
          break;

      } else if (bmuState.subState == LTC_READ_VOLTAGE_REGISTER_D_RDCVD_READVOLTAGE) {
          retVal = LTC_RX_PECCheck(ltc_RXPECbuffer);
          LTC_SaveRXtoVoltagebuffer(2, ltc_RXPECbuffer);
          retVal = ltc68xx_ReadVoltage(&ltc68xx,3,ltc_RXPECbuffer);
          LTC_StateTransition(LTC_STATEMACH_READVOLTAGE, LTC_EXIT_READVOLTAGE, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT));
          break;

      } else if (bmuState.subState == LTC_READ_VOLTAGE_REGISTER_E_RDCVE_READVOLTAGE) {
//          retVal = LTC_RX_PECCheck(ltc_RXPECbuffer);
//          DIAG_checkEvent(retVal, DIAG_CH_LTC_PEC, 0);
//          LTC_SaveRXtoVoltagebuffer(3, ltc_RXPECbuffer);
//
//          SPI_SetTransmitOngoing();
//          retVal = LTC_RX((uint8_t*)(ltc_cmdRDCVE), ltc_RXPECbuffer);
//           if (BS_MAX_SUPPORTED_CELLS > 15) {
//              LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                      LTC_STATEMACH_READVOLTAGE, LTC_READ_VOLTAGE_REGISTER_F_RDCVF_READVOLTAGE, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT),
//                      LTC_STATEMACH_READVOLTAGE, LTC_READ_VOLTAGE_REGISTER_F_RDCVF_READVOLTAGE, LTC_STATEMACH_SHORTTIME);
//          } else {
//              LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                      LTC_STATEMACH_READVOLTAGE, LTC_EXIT_READVOLTAGE, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT),
//                      LTC_STATEMACH_READVOLTAGE, LTC_EXIT_READVOLTAGE, LTC_STATEMACH_SHORTTIME);
//          }
          break;

      } else if (bmuState.subState == LTC_READ_VOLTAGE_REGISTER_F_RDCVF_READVOLTAGE) {
//          retVal = LTC_RX_PECCheck(ltc_RXPECbuffer);
//          DIAG_checkEvent(retVal, DIAG_CH_LTC_PEC, 0);
//          LTC_SaveRXtoVoltagebuffer(4, ltc_RXPECbuffer);
//
//          SPI_SetTransmitOngoing();
//          retVal = LTC_RX((uint8_t*)(ltc_cmdRDCVF), ltc_RXPECbuffer);
//          LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                  LTC_STATEMACH_READVOLTAGE, LTC_EXIT_READVOLTAGE, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT),
//                  LTC_STATEMACH_READVOLTAGE, LTC_EXIT_READVOLTAGE, LTC_STATEMACH_SHORTTIME);
          break;

      } else if (bmuState.subState == LTC_EXIT_READVOLTAGE) {
          retVal = LTC_RX_PECCheck(ltc_RXPECbuffer);
          LTC_SaveRXtoVoltagebuffer(3, ltc_RXPECbuffer);
          
          //calcluate sum of cells
          uint32_t sumOfCell = 0;
          for(uint8_t i=0;i<12;i++){
            sumOfCell += bmuState.cells.voltage[i];
          }
          bmuState.cells.sumOfCells = sumOfCell;
          /* Switch to different state if read voltage state is reused
           * e.g. open-wire check...                                */
          if (bmuState.balance_control_done == TRUE) {
              statereq = LTC_TransferStateRequest(&tmpbusID, &tmpadcMode, &tmpadcMeasCh);
              if (statereq == LTC_STATE_USER_IO_WRITE_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_USER_IO_CONTROL, LTC_USER_IO_SET_OUTPUT_REGISTER, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATE_USER_IO_READ_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_USER_IO_FEEDBACK, LTC_USER_IO_READ_INPUT_REGISTER, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATE_USER_IO_WRITE_REQUEST_TI) {
                  LTC_StateTransition(LTC_STATEMACH_USER_IO_CONTROL_TI, LTC_USER_IO_SET_DIRECTION_REGISTER_TI, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATE_USER_IO_READ_REQUEST_TI) {
                  LTC_StateTransition(LTC_STATEMACH_USER_IO_FEEDBACK_TI, LTC_USER_IO_SET_DIRECTION_REGISTER_TI, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATE_EEPROM_READ_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_EEPROM_READ, LTC_EEPROM_READ_DATA1, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATE_EEPROM_WRITE_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_EEPROM_WRITE, LTC_EEPROM_WRITE_DATA1, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATE_TEMP_SENS_READ_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_SEND_DATA1, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATEMACH_BALANCEFEEDBACK_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_BALANCEFEEDBACK, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              }  else if (statereq == LTC_STATE_OPENWIRE_CHECK_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_OPENWIRE_CHECK, LTC_REQUEST_PULLUP_CURRENT_OPENWIRE_CHECK, LTC_STATEMACH_SHORTTIME);
                  /* Send ADOW command with PUP two times */
                  bmuState.resendCommandCounter = LTC_NMBR_REQ_ADOW_COMMANDS;
                  bmuState.balance_control_done = FALSE;
              } 
              else if(statereq == LTC_STATEMACH_ALLGPIOMEASUREMENT){
                  LTC_StateTransition(LTC_STATEMACH_ALLGPIOMEASUREMENT, 0, LTC_STATEMACH_SHORTTIME);
                  /* Send ADOW command with PUP two times */
                  bmuState.resendCommandCounter = LTC_NMBR_REQ_ADOW_COMMANDS;
                  bmuState.balance_control_done = FALSE;
              }
              else {
                  LTC_StateTransition(LTC_STATEMACH_BALANCECONTROL, LTC_CONFIG_BALANCECONTROL, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = TRUE;
              }
          } else {
              if (bmuState.reusageMeasurementMode == LTC_NOT_REUSED) {
                  //LTC_SaveVoltages();
                  LTC_StateTransition(LTC_STATEMACH_BALANCECONTROL, LTC_CONFIG_BALANCECONTROL, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = TRUE;
              } else if (bmuState.reusageMeasurementMode == LTC_REUSE_READVOLT_FOR_ADOW_PUP) {
                  LTC_StateTransition(LTC_STATEMACH_OPENWIRE_CHECK, LTC_READ_VOLTAGES_PULLUP_OPENWIRE_CHECK, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (bmuState.reusageMeasurementMode == LTC_REUSE_READVOLT_FOR_ADOW_PDOWN) {
                  LTC_StateTransition(LTC_STATEMACH_OPENWIRE_CHECK, LTC_READ_VOLTAGES_PULLDOWN_OPENWIRE_CHECK, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              }else{
                LTC_StateTransition(LTC_STATEMACH_BALANCECONTROL, LTC_CONFIG_BALANCECONTROL, LTC_STATEMACH_SHORTTIME);
                bmuState.balance_control_done = TRUE;
              }
          }
      }
      break;
    case LTC_STATEMACH_BALANCECONTROL:
      if (bmuState.subState == LTC_CONFIG_BALANCECONTROL) {
          nvm_runtime_get_balancing(ltc68xx.balance->balancing_state);
          
          ltc68xx_balanceControl(&ltc68xx);
          LTC_StateTransition(LTC_STATEMACH_BALANCECONTROL, LTC_CONFIG2_BALANCECONTROL_END, LTC_STATEMACH_SHORTTIME);
//          LTC_StateTransition(LTC_STATEMACH_BALANCECONTROL, LTC_CONFIG2_BALANCECONTROL, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT));
          break;

      } 
      else if (bmuState.subState == LTC_CONFIG2_BALANCECONTROL) {
          if (bmuState.timer == 0) {
              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
              break;
          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
          }

          LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);

          break;

      } 
      else if (bmuState.subState == LTC_CONFIG2_BALANCECONTROL_END) {
//          if (bmuState.timer == 0 && SPI_IsTransmitOngoing() == TRUE) {
          if (bmuState.timer == 0) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
              break;
          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
          }
          /* More than 12 cells, balancing control finished */
//          bmuState.check_spi_flag = FALSE;
          LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);

          break;
      }
      break;
    case LTC_STATEMACH_ALLGPIOMEASUREMENT:
      bmuState.adcMode = LTC_GPIO_MEASUREMENT_MODE;
      bmuState.adcMeasCh = LTC_ADCMEAS_ALLCHANNEL;
      retVal = ltc68xx_startGPIOVMeas(&ltc68xx,bmuState.adcMode, bmuState.adcMeasCh);
      LTC_StateTransition(LTC_STATEMACH_READALLGPIO, LTC_READ_AUXILIARY_REGISTER_A_RDAUXA, (bmuState.commandTransferTime + LTC_Get_MeasurementTCycle(bmuState.adcMode, bmuState.adcMeasCh)));
      break;
    case LTC_STATEMACH_READALLGPIO:
      if (bmuState.subState == LTC_READ_AUXILIARY_REGISTER_A_RDAUXA) {
          retVal = ltc68xx_ReadAUX(&ltc68xx,0,ltc_RXPECbuffer);
          LTC_StateTransition(LTC_STATEMACH_READALLGPIO, LTC_READ_AUXILIARY_REGISTER_B_RDAUXB, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT));
          break;

      } else if (bmuState.subState == LTC_READ_AUXILIARY_REGISTER_B_RDAUXB) {
          retVal = LTC_RX_PECCheck(ltc_RXPECbuffer);
          LTC_SaveRXtoGPIOBuffer(0, ltc_RXPECbuffer);
          retVal = ltc68xx_ReadAUX(&ltc68xx,1,ltc_RXPECbuffer);
//          if (BS_MAX_SUPPORTED_CELLS > 12) {
//              LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                                        LTC_STATEMACH_READALLGPIO, LTC_READ_AUXILIARY_REGISTER_C_RDAUXC, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT),
//                                        LTC_STATEMACH_READALLGPIO, LTC_READ_AUXILIARY_REGISTER_C_RDAUXC, LTC_STATEMACH_SHORTTIME);
//          } else {
//              LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                                        LTC_STATEMACH_READALLGPIO, LTC_EXIT_READAUXILIARY_ALLGPIOS, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT),
//                                        LTC_STATEMACH_READALLGPIO, LTC_EXIT_READAUXILIARY_ALLGPIOS, LTC_STATEMACH_SHORTTIME);
//          }
          retVal = ltc68xx_ReadAUX(&ltc68xx,1,ltc_RXPECbuffer);
          LTC_StateTransition(LTC_STATEMACH_READALLGPIO, LTC_EXIT_READAUXILIARY_ALLGPIOS, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT));
          break;

      } else if (bmuState.subState == LTC_READ_AUXILIARY_REGISTER_C_RDAUXC) {
//          retVal = LTC_RX_PECCheck(ltc_RXPECbuffer);
//          DIAG_checkEvent(retVal, DIAG_CH_LTC_PEC, 0);
//          LTC_SaveRXtoGPIOBuffer(1, ltc_RXPECbuffer);
//
//          SPI_SetTransmitOngoing();
//          retVal = LTC_RX((uint8_t*)(ltc_cmdRDAUXC), ltc_RXPECbuffer);
//          LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                                    LTC_STATEMACH_READALLGPIO, LTC_READ_AUXILIARY_REGISTER_D_RDAUXD, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT),
//                                    LTC_STATEMACH_READALLGPIO, LTC_READ_AUXILIARY_REGISTER_D_RDAUXD, LTC_STATEMACH_SHORTTIME);
          break;

      } else if (bmuState.subState == LTC_READ_AUXILIARY_REGISTER_D_RDAUXD) {
//          retVal = LTC_RX_PECCheck(ltc_RXPECbuffer);
//          DIAG_checkEvent(retVal, DIAG_CH_LTC_PEC, 0);
//          LTC_SaveRXtoGPIOBuffer(2, ltc_RXPECbuffer);
//
//          SPI_SetTransmitOngoing();
//          retVal = LTC_RX((uint8_t*)(ltc_cmdRDAUXD), ltc_RXPECbuffer);
//          LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                                    LTC_STATEMACH_READALLGPIO, LTC_EXIT_READAUXILIARY_ALLGPIOS, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT),
//                                    LTC_STATEMACH_READALLGPIO, LTC_EXIT_READAUXILIARY_ALLGPIOS, LTC_STATEMACH_SHORTTIME);
          break;

      } else if (bmuState.subState == LTC_EXIT_READAUXILIARY_ALLGPIOS) {
          retVal = LTC_RX_PECCheck(ltc_RXPECbuffer);
          LTC_SaveRXtoGPIOBuffer(1, ltc_RXPECbuffer);
//          DIAG_checkEvent(retVal, DIAG_CH_LTC_PEC, 0);

//          if (BS_MAX_SUPPORTED_CELLS == 12) {
//              LTC_SaveRXtoGPIOBuffer(1, ltc_RXPECbuffer);
//          } else if (BS_MAX_SUPPORTED_CELLS > 12) {
//              LTC_SaveRXtoGPIOBuffer(3, ltc_RXPECbuffer);
//          }

//          LTC_SaveAllGPIOMeasurement();

          if (bmuState.balance_control_done == TRUE) {
              statereq = LTC_TransferStateRequest(&tmpbusID, &tmpadcMode, &tmpadcMeasCh);
              if (statereq == LTC_STATE_USER_IO_WRITE_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_USER_IO_CONTROL, LTC_USER_IO_SET_OUTPUT_REGISTER, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATE_USER_IO_READ_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_USER_IO_FEEDBACK, LTC_USER_IO_READ_INPUT_REGISTER, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATE_USER_IO_WRITE_REQUEST_TI) {
                  LTC_StateTransition(LTC_STATEMACH_USER_IO_CONTROL_TI, LTC_USER_IO_SET_DIRECTION_REGISTER_TI, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATE_USER_IO_READ_REQUEST_TI) {
                  LTC_StateTransition(LTC_STATEMACH_USER_IO_FEEDBACK_TI, LTC_USER_IO_SET_DIRECTION_REGISTER_TI, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATE_EEPROM_READ_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_EEPROM_READ, LTC_EEPROM_READ_DATA1, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATE_EEPROM_WRITE_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_EEPROM_WRITE, LTC_EEPROM_WRITE_DATA1, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATE_TEMP_SENS_READ_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_SEND_DATA1, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              } else if (statereq == LTC_STATEMACH_BALANCEFEEDBACK_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_BALANCEFEEDBACK, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = FALSE;
              }  else if (statereq == LTC_STATE_OPENWIRE_CHECK_REQUEST) {
                  LTC_StateTransition(LTC_STATEMACH_OPENWIRE_CHECK, LTC_REQUEST_PULLUP_CURRENT_OPENWIRE_CHECK, LTC_STATEMACH_SHORTTIME);
                  /* Send ADOW command with PUP two times */
                  bmuState.resendCommandCounter = LTC_NMBR_REQ_ADOW_COMMANDS;
                  bmuState.balance_control_done = FALSE;
              } else {
                  LTC_StateTransition(LTC_STATEMACH_BALANCECONTROL, LTC_CONFIG_BALANCECONTROL, LTC_STATEMACH_SHORTTIME);
                  bmuState.balance_control_done = TRUE;
              }
          } else {
              LTC_StateTransition(LTC_STATEMACH_BALANCECONTROL, LTC_CONFIG_BALANCECONTROL, LTC_STATEMACH_SHORTTIME);
              bmuState.balance_control_done = TRUE;
          }
      }
      break;
    case LTC_STATEMACH_BALANCEFEEDBACK:
       if (bmuState.subState == LTC_ENTRY) {
          bmuState.adcMode = LTC_ADCMODE_NORMAL_DCP0;
          bmuState.adcMeasCh = LTC_ADCMEAS_SINGLECHANNEL_GPIO3;

          retVal = ltc68xx_startGPIOVMeas(&ltc68xx,bmuState.adcMode, bmuState.adcMeasCh);
          if(retVal == E_OK){
            LTC_StateTransition(LTC_STATEMACH_BALANCEFEEDBACK, LTC_READ_FEEDBACK_BALANCECONTROL, (bmuState.commandDataTransferTime + LTC_Get_MeasurementTCycle(bmuState.adcMode, bmuState.adcMeasCh)));
          }
          else
          {
            LTC_StateTransition(LTC_STATEMACH_BALANCEFEEDBACK, LTC_READ_FEEDBACK_BALANCECONTROL,LTC_STATEMACH_SHORTTIME );
          }
          break;

      } else if (bmuState.subState == LTC_READ_FEEDBACK_BALANCECONTROL) {
//          retVal = LTC_RX((uint8_t*)ltc_cmdRDAUXA, ltc_RXPECbuffer);  /* read AUXA register */
//          LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                  LTC_STATEMACH_BALANCEFEEDBACK, LTC_SAVE_FEEDBACK_BALANCECONTROL, bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT,
//                  LTC_STATEMACH_BALANCEFEEDBACK, LTC_SAVE_FEEDBACK_BALANCECONTROL, LTC_STATEMACH_SHORTTIME);
          retVal = ltc68xx_ReadAUX(&ltc68xx,0,ltc_RXPECbuffer);
          if(retVal == E_OK){
            LTC_StateTransition(LTC_STATEMACH_BALANCEFEEDBACK, LTC_SAVE_FEEDBACK_BALANCECONTROL, (bmuState.commandDataTransferTime + LTC_TRANSMISSION_TIMEOUT));
          }
          else
          {
            LTC_StateTransition(LTC_STATEMACH_BALANCEFEEDBACK, LTC_SAVE_FEEDBACK_BALANCECONTROL,LTC_STATEMACH_SHORTTIME );
          }

      } else if (bmuState.subState == LTC_SAVE_FEEDBACK_BALANCECONTROL) {
//          if (bmuState.timer == 0 && SPI_IsTransmitOngoing() == TRUE) {
          if (bmuState.timer == 0) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
              break;
          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
          }

          if (LTC_RX_PECCheck(ltc_RXPECbuffer) != E_OK) {
//              DIAG_Handler(DIAG_CH_LTC_PEC, DIAG_EVENT_NOK, 0);
          } else {
//              DIAG_Handler(DIAG_CH_LTC_PEC, DIAG_EVENT_OK, 0);
              LTC_SaveBalancingFeedback(ltc_RXPECbuffer);
          }
          LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
          break;
      }
      break;
    case LTC_STATEMACH_TEMP_SENS_READ:
      if (bmuState.subState == LTC_TEMP_SENS_SEND_DATA1) {
//          retVal = LTC_Send_I2C_Command(ltc_TXBuffer, ltc_TXPECbuffer, (uint8_t*)ltc_I2CcmdTempSens0);
//
//          if (retVal != E_OK) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
//              ++bmuState.muxmeas_seqptr;
//              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
//          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
//              LTC_StateTransition(LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_SEND_CLOCK_STCOMM1, bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT);
//          }
//
//          LTC_StateTransition(LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_SEND_CLOCK_STCOMM1, bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT);
          break;

      } else if (bmuState.subState == LTC_TEMP_SENS_SEND_CLOCK_STCOMM1) {
          if (bmuState.timer == 0) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
              break;
          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
          }

//          SPI_SetTransmitOngoing();
//          retVal = LTC_I2CClock(ltc_TXBufferClock, ltc_TXPECBufferClock);
//          LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                                    LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_READ_DATA1, (bmuState.gpioClocksTransferTime+LTC_TRANSMISSION_TIMEOUT),
//                                    LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_READ_DATA1, LTC_STATEMACH_SHORTTIME);
          LTC_StateTransition(LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_READ_DATA1, (bmuState.gpioClocksTransferTime+LTC_TRANSMISSION_TIMEOUT));
          break;

      } else if (bmuState.subState == LTC_TEMP_SENS_READ_DATA1) {
          if (bmuState.timer == 0) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
              break;
          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
          }

//          SPI_SetTransmitOngoing();
//          retVal = LTC_Send_I2C_Command(ltc_TXBuffer, ltc_TXPECbuffer, (uint8_t*)ltc_I2CcmdTempSens1);

//          if (retVal != E_OK) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
//              ++bmuState.muxmeas_seqptr;
//              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
//          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
//              LTC_StateTransition(LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_SEND_CLOCK_STCOMM2, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT));
//          }

          LTC_StateTransition(LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_SEND_CLOCK_STCOMM2, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT));
          break;

      } else if (bmuState.subState == LTC_TEMP_SENS_SEND_CLOCK_STCOMM2) {
          if (bmuState.timer == 0) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
              break;
          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
          }

//          SPI_SetTransmitOngoing();
//          retVal = LTC_I2CClock(ltc_TXBufferClock, ltc_TXPECBufferClock);
//          LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                                    LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_READ_I2C_TRANSMISSION_RESULT_RDCOMM, (bmuState.gpioClocksTransferTime+LTC_TRANSMISSION_TIMEOUT),
//                                    LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_READ_I2C_TRANSMISSION_RESULT_RDCOMM, LTC_STATEMACH_SHORTTIME);
          LTC_StateTransition(LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_READ_I2C_TRANSMISSION_RESULT_RDCOMM, (bmuState.gpioClocksTransferTime+LTC_TRANSMISSION_TIMEOUT));
          break;
      }  else if (bmuState.subState == LTC_TEMP_SENS_READ_I2C_TRANSMISSION_RESULT_RDCOMM) {
          if (bmuState.timer == 0) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
              break;
          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
          }

//          SPI_SetTransmitOngoing();
//          retVal = LTC_RX((uint8_t*)ltc_cmdRDCOMM, ltc_RXPECbuffer);
//          LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                                    LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_SAVE_TEMP, bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT,
//                                    LTC_STATEMACH_TEMP_SENS_READ, LTC_TEMP_SENS_SAVE_TEMP, LTC_STATEMACH_SHORTTIME);
          break;

      } else if (bmuState.subState == LTC_TEMP_SENS_SAVE_TEMP) {
          if (bmuState.timer == 0) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
              break;
          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
          }

          if (LTC_RX_PECCheck(ltc_RXPECbuffer) != E_OK) {
//              DIAG_Handler(DIAG_CH_LTC_PEC, DIAG_EVENT_NOK, 0);
          } else {
//              DIAG_Handler(DIAG_CH_LTC_PEC, DIAG_EVENT_OK, 0);
//              LTC_TempSensSaveTemp(ltc_RXPECbuffer);
          }

          LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
          break;
      }
      break;
    case LTC_STATEMACH_USER_IO_CONTROL:
      if (bmuState.subState == LTC_USER_IO_SET_OUTPUT_REGISTER) {
//          bmuState.check_spi_flag = TRUE;
//          SPI_SetTransmitOngoing();
//          retVal = LTC_SetPortExpander(ltc_TXBuffer, ltc_TXPECbuffer);
//
//          if (retVal != E_OK) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
//              ++bmuState.muxmeas_seqptr;
//              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
//          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
//              LTC_StateTransition(LTC_STATEMACH_USER_IO_CONTROL, LTC_SEND_CLOCK_STCOMM_MUXMEASUREMENT_CONFIG, (bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT));
//          }
          break;

      } else if (bmuState.subState == LTC_SEND_CLOCK_STCOMM_MUXMEASUREMENT_CONFIG) {
//          if (bmuState.timer == 0 && SPI_IsTransmitOngoing() == TRUE) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
//              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
//              break;
//          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
//          }
//
//          bmuState.check_spi_flag = FALSE;
//          retVal = LTC_I2CClock(ltc_TXBufferClock, ltc_TXPECBufferClock);
//          LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                                    LTC_STATEMACH_STARTMEAS, LTC_ENTRY, bmuState.gpioClocksTransferTime,
//                                    LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
          break;
      }
      break;
    case LTC_STATEMACH_USER_IO_FEEDBACK:
      if (bmuState.subState == LTC_USER_IO_READ_INPUT_REGISTER) {
//          bmuState.check_spi_flag = TRUE;
//          SPI_SetTransmitOngoing();
//          retVal = LTC_Send_I2C_Command(ltc_TXBuffer, ltc_TXPECbuffer, (uint8_t*)ltc_I2CcmdPortExpander1);
//
//          if (retVal != E_OK) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
//              ++bmuState.muxmeas_seqptr;
//              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
//          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
//              LTC_StateTransition(LTC_STATEMACH_USER_IO_FEEDBACK, LTC_USER_IO_SEND_CLOCK_STCOMM, bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT);
//          }

          break;

      } else if (bmuState.subState == LTC_USER_IO_SEND_CLOCK_STCOMM) {
//        if (bmuState.timer == 0 && SPI_IsTransmitOngoing() == TRUE) {
//            DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
//            LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
//            break;
//        } else {
//            DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
//        }
//
//        bmuState.check_spi_flag = FALSE;
//        retVal = LTC_I2CClock(ltc_TXBufferClock, ltc_TXPECBufferClock);
//        LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                                  LTC_STATEMACH_USER_IO_FEEDBACK, LTC_USER_IO_READ_I2C_TRANSMISSION_RESULT_RDCOMM, bmuState.gpioClocksTransferTime,
//                                  LTC_STATEMACH_USER_IO_FEEDBACK, LTC_USER_IO_READ_I2C_TRANSMISSION_RESULT_RDCOMM, LTC_STATEMACH_SHORTTIME);
        break;

      } else if (bmuState.subState == LTC_USER_IO_READ_I2C_TRANSMISSION_RESULT_RDCOMM) {
//          if (bmuState.timer == 0 && SPI_IsTransmitOngoing() == TRUE) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
//              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
//              break;
//          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
//          }
//
//          SPI_SetTransmitOngoing();
//          retVal = LTC_RX((uint8_t*)ltc_cmdRDCOMM, ltc_RXPECbuffer);
//          LTC_CondBasedStateTransition(retVal, DIAG_CH_LTC_SPI,
//                                    LTC_STATEMACH_USER_IO_FEEDBACK, LTC_USER_IO_SAVE_DATA, bmuState.commandDataTransferTime+LTC_TRANSMISSION_TIMEOUT,
//                                    LTC_STATEMACH_USER_IO_FEEDBACK, LTC_USER_IO_SAVE_DATA, LTC_STATEMACH_SHORTTIME);
          break;

      } else if (bmuState.subState == LTC_USER_IO_SAVE_DATA) {
//          if (bmuState.timer == 0 && SPI_IsTransmitOngoing() == TRUE) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_NOK, 0);
//              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
//              break;
//          } else {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
//          }
//
//          if (LTC_RX_PECCheck(ltc_RXPECbuffer) != E_OK) {
//              DIAG_Handler(DIAG_CH_LTC_PEC, DIAG_EVENT_NOK, 0);
//          } else {
//              DIAG_Handler(DIAG_CH_LTC_PEC, DIAG_EVENT_OK, 0);
//              LTC_PortExpanderSaveValues(ltc_RXPECbuffer);
//          }
//
//          LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
          break;
      }
      break;
    case LTC_STATEMACH_OPENWIRE_CHECK:
      if (bmuState.subState == LTC_REQUEST_PULLUP_CURRENT_OPENWIRE_CHECK) {
          /* Run ADOW command with PUP = 1 */
          bmuState.adcMode = LTC_OW_MEASUREMENT_MODE;
          retVal = ltc68xx_startOpenWireMeas(&ltc68xx,bmuState.adcMode, 1);
          if (retVal == E_OK) {
//              DIAG_Handler(DIAG_CH_LTC_SPI, DIAG_EVENT_OK, 0);
              LTC_StateTransition(LTC_STATEMACH_OPENWIRE_CHECK, LTC_REQUEST_PULLUP_CURRENT_OPENWIRE_CHECK, (bmuState.commandDataTransferTime + LTC_Get_MeasurementTCycle(bmuState.adcMode, LTC_ADCMEAS_ALLCHANNEL)));
              bmuState.resendCommandCounter--;

              /* Check how many retries are left */
              if (bmuState.resendCommandCounter == 0) {
                  /* Switch to read voltage state to read cell voltages */
                  LTC_StateTransition(LTC_STATEMACH_READVOLTAGE, LTC_READ_VOLTAGE_REGISTER_A_RDCVA_READVOLTAGE, (bmuState.commandDataTransferTime + LTC_Get_MeasurementTCycle(bmuState.adcMode, LTC_ADCMEAS_ALLCHANNEL)));
                  /* Reuse read voltage register */
                  bmuState.reusageMeasurementMode = LTC_REUSE_READVOLT_FOR_ADOW_PUP;
              }
          } else {
              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
          }
      } else if (bmuState.subState == LTC_READ_VOLTAGES_PULLUP_OPENWIRE_CHECK) {
          /* Previous state: Read voltage -> information stored in voltage buffer */
          bmuState.reusageMeasurementMode = LTC_NOT_REUSED;

          /* Copy data from voltage struct into open-wire struct */
//          for (uint16_t i = 0; i < BS_NR_OF_BAT_CELLS; i++) {
//              bmuState.cells.pullupFeedback[i] = bmuState.cells.voltage[i];
//          }

          /* Set number of ADOW retries - send ADOW command with pull-down two times */
          bmuState.resendCommandCounter = LTC_NMBR_REQ_ADOW_COMMANDS;
          LTC_StateTransition(LTC_STATEMACH_OPENWIRE_CHECK, LTC_REQUEST_PULLDOWN_CURRENT_OPENWIRE_CHECK, LTC_STATEMACH_SHORTTIME);

      } else if (bmuState.subState == LTC_REQUEST_PULLDOWN_CURRENT_OPENWIRE_CHECK) {
          /* Run ADOW command with PUP = 0 */
          bmuState.adcMode = LTC_OW_MEASUREMENT_MODE;
          retVal = ltc68xx_startOpenWireMeas(&ltc68xx,bmuState.adcMode, 0);
          if (retVal == E_OK) {
              LTC_StateTransition(LTC_STATEMACH_OPENWIRE_CHECK, LTC_REQUEST_PULLDOWN_CURRENT_OPENWIRE_CHECK, (bmuState.commandDataTransferTime + LTC_Get_MeasurementTCycle(bmuState.adcMode, LTC_ADCMEAS_ALLCHANNEL)));
              bmuState.resendCommandCounter--;

              /* Check how many retries are left */
              if (bmuState.resendCommandCounter == 0) {
                  /* Switch to read voltage state to read cell voltages */
                  LTC_StateTransition(LTC_STATEMACH_READVOLTAGE, LTC_READ_VOLTAGE_REGISTER_A_RDCVA_READVOLTAGE, (bmuState.commandDataTransferTime + LTC_Get_MeasurementTCycle(bmuState.adcMode, LTC_ADCMEAS_ALLCHANNEL)));
                  /* Reuse read voltage register */
                  bmuState.reusageMeasurementMode = LTC_REUSE_READVOLT_FOR_ADOW_PDOWN;
              }
          } else {
              LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
          }
      } else if (bmuState.subState == LTC_READ_VOLTAGES_PULLDOWN_OPENWIRE_CHECK) {
          /* Previous state: Read voltage -> information stored in voltage buffer */
          bmuState.reusageMeasurementMode = LTC_NOT_REUSED;
          retVal = ltc68xx_startOpenWireMeas(&ltc68xx,bmuState.adcMode, 0xff);

          /* Copy data from voltage struct into open-wire struct */
//          for (uint16_t i = 0; i < BS_NR_OF_BAT_CELLS; i++) {
//              bmuState.cells.pulldnFeedback[i] = bmuState.cells.voltage[i];
//          }
          LTC_StateTransition(LTC_STATEMACH_OPENWIRE_CHECK, LTC_PERFORM_OPENWIRE_CHECK, LTC_STATEMACH_SHORTTIME);
      } else if (bmuState.subState == LTC_PERFORM_OPENWIRE_CHECK) {
          /* Perform actual open-wire check */
//          for (uint8_t m = 0; m < BS_NR_OF_MODULES; m++) {
          bmuState.reusageMeasurementMode = LTC_NOT_REUSED;
            uint8_t m = 0;
              /* Open-wire at C0: cell_pup(0) == 0 */
              if (bmuState.cells.pullupFeedback[0 + (m*BS_NR_OF_BAT_CELLS_PER_MODULE)] == 0) {
                  bmuState.cells.openWire[0 + (m*(BS_NR_OF_BAT_CELLS_PER_MODULE))] = 1;
              }
              else{
                bmuState.cells.openWire[0] = 0;
              }
              /* Open-wire at Cmax: cell_pdown(BS_NR_OF_BAT_CELLS_PER_MODULE-1) == 0 */
              if (bmuState.cells.pulldnFeedback[((BS_NR_OF_BAT_CELLS_PER_MODULE-1) + (m*BS_NR_OF_BAT_CELLS_PER_MODULE))] == 0) {
                  bmuState.cells.openWire[BS_NR_OF_BAT_CELLS_PER_MODULE + (m*BS_NR_OF_BAT_CELLS_PER_MODULE)] = 1;
              }
              else{
                bmuState.cells.openWire[BS_NR_OF_BAT_CELLS_PER_MODULE] = 0;
              }
//          }

          /* Take difference between pull-up and pull-down measurement */
          int32_t ltc_openwire_delta[12];
          for (uint16_t i = 1; i < BS_NR_OF_BAT_CELLS; i++) {
              ltc_openwire_delta[i] = (int32_t)(bmuState.cells.pullupFeedback[i] - bmuState.cells.pulldnFeedback[i]);
          }

          /* Open-wire at C(N): delta cell(n+1) < -400mV */
          
          for (uint8_t m = 0; m < BS_NR_OF_MODULES; m++) {
              for (uint8_t c = 1; c < BS_NR_OF_BAT_CELLS_PER_MODULE; c++) {
                  if ((ltc_openwire_delta[c + (m*BS_NR_OF_BAT_CELLS_PER_MODULE)] < -400)) {
                      bmuState.cells.openWire[c + (m*BS_NR_OF_BAT_CELLS_PER_MODULE)] = 1;
                  }
                  else{
                      bmuState.cells.openWire[c + (m*BS_NR_OF_BAT_CELLS_PER_MODULE)] = 0;
                  }
              }
          }
          
          for(uint8_t c=0;c < BS_NR_OF_BAT_CELLS_PER_MODULE ; c++){
            if((bmuState.cells.pullupFeedback[c] < 200) && (bmuState.cells.pulldnFeedback[c] < 200)){
              bmuState.cells.openWire[c] = 1;
            }
            else{
              bmuState.cells.openWire[c] = 0;
            }
          }

          for(uint8_t c=0;c<BS_NR_OF_BAT_CELLS_PER_MODULE;c++){
            if(bmuState.cells.activeCellQue[c] == 0){
              bmuState.cells.openWire[c] = 0;
            }
          }
          
//          for(uint8_t c=1;c < BS_NR_OF_BAT_CELLS_PER_MODULE-1 ; c++){
//            if(bmuState.cells.pullupFeedback[c] == 0){
//              bmuState.cells.openWire[c+1] = 1;
//            }
//          }
          
          /* 2021/08/18 open wire check pup, pdown result in 0 mv reading
              if battery is unplugged, temporary use this behavior to decide
              whether open wire or not*/

          /* Write database entry */
//          DB_WriteBlock(&ltc_openwire, DATA_BLOCK_ID_OPEN_WIRE);
          nvm_runtime_set_openWire(bmuState.cells.openWire);
          /* Start new measurement cycle */
          LTC_StateTransition(LTC_STATEMACH_STARTMEAS, LTC_ENTRY, LTC_STATEMACH_SHORTTIME);
      }
      break;
    default:
      break;
    }
    chThdSleepMilliseconds(20);
  }
  
}




msg_t bmu_ltc_init()
{
  // read nvm
  
//  for(uint8_t i=0;i<12;i++)
//    bmuState.cells.activeCellQue[i] = 1;
  
  nvm_get_cellQueue(bmuState.cells.activeCellQue);
  
  bmuState.usedIndex = 0;
//  thisThd = chThdCreateStatic(waBMU, sizeof(waBMU), NORMALPRIO, procBMU, NULL);
  chThdCreateStatic(waBMU, sizeof(waBMU), NORMALPRIO, procBMU, NULL);
  chSysLock();
  msg_t ret = chThdSuspendS(&thd_trp);
  chSysUnlock();
  return ret;
}

msg_t bmu_ltc_read_cellVolt(uint8_t offset, uint8_t n, uint8_t *d)
{
  chSysLock();
  for(uint8_t i=0;i<n;i++){
    memcpy(d,(uint8_t*)&bmuState.cells.voltage[i+offset],2);
    d+=2;
  }
  chSysUnlock();
  return MSG_OK;
}

void bmu_ltc_set_balance(uint8_t* val)
{
  for(uint8_t i=0;i<BS_NR_OF_BAT_CELLS_PER_MODULE;i++){
    ltc68xx.balance->balancing_state[i] = val[i];
  }
}

static int8_t registerRead(LTC68XXDriver *dev, uint8_t *b, uint16_t n, uint8_t cs)
{
//  spiAcquireBus(dev->config->devp);
//  spiStart(dev->config->devp,dev->config->config);
//  if(cs)
//    spiSelect(dev);
//  spiSend(dev->config->devp,1,&reg);
//  spiReceive(dev->config->devp,n,b);
//  dev->chipSel(0);
//  spiStop(dev->config->devp);
//  spiReleaseBus(dev->config->devp);
  return 0;
}

static int8_t registerWrite(LTC68XXDriver *dev, uint8_t *b, uint16_t n, uint8_t cs)
{
  if(b != NULL){
    spiAcquireBus(dev->config->devp);
    spiStart(dev->config->devp,dev->config->config);
    spiSelect(dev->config->devp);
    spiSend(dev->config->devp,n,b);
    spiUnselect(dev->config->devp);
    spiStop(dev->config->devp);
    spiReleaseBus(dev->config->devp);
  }
  return 0;
}

int8_t dataExchange(LTC68XXDriver *dev, uint8_t *tx, uint16_t ntx, uint8_t *rx, uint16_t nrx)
{
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp,dev->config->config);

  spiSelect(dev->config->devp);
  spiSend(dev->config->devp,ntx,tx);
  spiReceive(dev->config->devp,nrx,rx);
  spiUnselect(dev->config->devp);

  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);
  return 0;
}


