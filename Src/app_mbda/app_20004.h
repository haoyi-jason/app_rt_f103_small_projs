/*
 * app_20004.h
 *
 *  Created on: 2020¦~5¤ë30¤é
 *      Author: Jason
 */

#ifndef SOURCE_APP_20004_H_
#define SOURCE_APP_20004_H_

#define NOF_DA_CH   3
#define NOF_AD_CH   4
#define NOF_DO_CH   2
#define EEP_FLAG    "EP02"
#define FW_VERSION      0x20092501
struct da_config{
  uint16_t zeroValue; // pwm output for 4mA
  uint16_t fullValue; // pwm output for 20mA
  float engLow; // engineering value for 4mA output
  float engHigh;// engineering value for 20mA output
};

struct do_config{
  uint8_t logic; // 0: normal, 1: reverse
  uint8_t temp[15]; // reserve for later use
};

struct module_config{
  uint8_t query_wait;   // delay between querys
  uint16_t cycletime;
};

struct serial_config{
  uint8_t cfg[4]; // 0: id, 1: parity, 2/3 not defined
  uint32_t baudrate;
};

struct channel_map_s{
  uint8_t enabled;      // 0: inactive,1: active
  uint8_t input;        // could b 'A','B'...
  uint8_t output;       // 0x00/0x01/0x02, 0x10/0x11
};

#define NOF_MAP_QUERY   16
typedef struct{
  uint8_t flag[4];
  uint8_t moduleInfo[64];
  struct da_config da_config[NOF_DA_CH];
  struct do_config do_config[NOF_DO_CH];
  struct module_config module;
  struct serial_config serialConfig;
  struct channel_map_s map[NOF_MAP_QUERY]; // max 8 maps
  struct da_config ad_config[NOF_AD_CH];
}_nvm_param_t;

typedef struct led_map_s{
  virtual_timer_t vt;
  ioline_t line;
  uint16_t ms_on;
  uint16_t ms_off;
}led_map_t;


typedef struct{
  uint32_t runSecs;
  uint16_t duty[NOF_DA_CH];
  led_map_t leds[4];
  uint16_t analogIn_raw[4];
  uint8_t analog_channel;
}_rumtime_param_t;

typedef struct{
  _nvm_param_t nvm;
  _rumtime_param_t runtime;
}_app_param_t;


void app_2004_init(void);

#endif /* SOURCE_APP_20004_H_ */
