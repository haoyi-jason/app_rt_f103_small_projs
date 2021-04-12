#ifndef _TLE9012_H
#define _TLE9012_H

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

#endif