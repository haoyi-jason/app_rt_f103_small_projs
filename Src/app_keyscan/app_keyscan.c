/*
 * app_20004.c
 *
 *  Created on: 2020¦~5¤ë30¤é
 *      Author: Jason
 */

#include "ch.h"
#include "hal.h"
#include "at24_eep.h"
#include "app_20004.h"
#include "lora_osi_wrapper.h"
#include "ads1015.h"
#include "port_20004.h"

static const _nvm_param_t  default_nvm_param = {
  EEP_FLAG,
  "MODULE PARAM",
  {{1070,7930,0,100},{1070,7930,0,100},{1070,7930,0,100}},
  {{1,0},{1,0}},
  {2,100},
  {{1,0,0,0},9600},
};

static _app_param_t appParam;

static SPIConfig spicfg_lora = {
  false,
  NULL,
  GPIOB,
  GPIOB_SPI2_CS,
  SPI_CR1_BR_2, //  SPI_CR1_BR_2  | SPI_CR1_CPHA | SPI_CR1_CPOL
  0
};

static const I2CConfig i2ccfg = {
  OPMODE_I2C,
  100000,
  STD_DUTY_CYCLE
};
static ads1x15_config_t ads1115_config = {
 &I2CD1,
 &i2ccfg,
 DEV_ADS1115
};
ADS1x15Driver ads1015 = {
  &ads1115_config,
  0x0580,
  0x0,
  0x1,
  ADS1015_ADDR_GND,
};


void app_2004_do_write(uint8_t ch, uint8_t v);
thread_t *self;

static PWMConfig pwmcfg = {
  36000000,                                    /* 10MHz PWM clock frequency.   */
  8192,                                    /* Initial PWM frequency 610 Hz.       */
  NULL,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  0,
  0,
};

//#define PERCENT_TO_WIDTH(x) ((pwmcnt_t)

static SerialConfig serialCfg={
  9600
};

#define MSG_BUF_SZ      32
#define MSG_BUF_MASK    MSG_BUF_SZ-1
#define CMD_PACKET_SIZE 13


#define QUERY_PACKET_SZ 9
#define NOF_QUERY   2
static char query_str[2][9]={
 {0x2,0x30,0x31,0x52,0x50,0x41,0x34,0x34,0x03},
 {0x2,0x30,0x31,0x52,0x50,0x42,0x34,0x35,0x03}
};

static uint8_t pwm_map[3] = {2,1,0};

enum query_state_e{
  QUERY_CHA,
  QUERY_CHB,
  QUERY_NUMBER
};

enum led_e{
  LED_R,
  LED_G,
  LED_AUXG,
  LED_AUXR
};

static ioline_t leds[] = {
  PAL_LINE(GPIOA,11),
  PAL_LINE(GPIOA,12),
  PAL_LINE(GPIOA,4),
  PAL_LINE(GPIOA,5)
};

uint16_t AO_TO_DUTY(uint8_t ch,float engVal)
{
  uint16_t pwm = 0;
  if(engVal < appParam.nvm.da_config[ch].engLow) pwm = appParam.nvm.da_config[ch].zeroValue;
  else if(engVal > appParam.nvm.da_config[ch].engHigh) pwm = appParam.nvm.da_config[ch].fullValue;
  else{
    uint16_t span = appParam.nvm.da_config[ch].fullValue - appParam.nvm.da_config[ch].zeroValue+1;
    float ratio = (engVal - appParam.nvm.da_config[ch].engLow)/(appParam.nvm.da_config[ch].engHigh - appParam.nvm.da_config[ch].engLow);
    pwm = appParam.nvm.da_config[ch].zeroValue + (uint16_t)(ratio*span);
  }
  return pwm;
}

pwmcnt_t map_pwm_output(uint8_t id,float x)
{
  pwmcnt_t ret = 0;
  struct da_config *da = &appParam.nvm.da_config[id];

  if(x <= da->engLow) ret = da->zeroValue;
  else if(x >= da->engHigh) ret = da->fullValue;
  else{
    uint16_t span = da->fullValue - da->zeroValue+1;
    float r = (x - da->engLow);
    float s = (da->engHigh-da->engLow);
    r = r / s;
    ret = (pwmcnt_t)(r*span)+da->zeroValue;
  }
  return ret;
}
uint8_t extract_hex_from_ascii(uint8_t *ptr)
{
  uint8_t ret = 0x0;
  for(uint8_t i=0;i<2;i++){
    ret <<= 4;
    uint8_t c = *ptr++;
    if(c > 9) c -= 0x30;
    else c -= ('A'-10-0x30);
    ret |= c;
  }
  return ret;
}

uint8_t *extract_val_from_hexASC(uint8_t *src, uint8_t *dst, uint8_t typeSz)
{
  uint8_t ret = 0x0;
  for(uint8_t j=0;j<typeSz;j++){
    ret = 0;
    for(uint8_t i=0;i<2;i++){
      ret <<= 4;
      uint8_t c = *src++;
      if(c < 0x3a) c -= 0x30;
      else c -= 0x37;
      ret |= c;
    }
    *dst++ = ret;
  }
  return src;
}

uint8_t* encode_ascii_from_hex(uint8_t *src, uint8_t *dst, uint8_t typeSz)
{
  uint8_t c;
  for(uint8_t i=0;i<typeSz;i++){
      c = *src;
      c >>= 4;
      if(c > 9) c += 0x37;
      else c += 0x30;
      *dst++ = c;
      c = *src++;
      c &= 0xf;
      if(c > 9) c += 0x37;
      else c += 0x30;
      *dst++ = c;
  }
  return dst;
}

void save_param(void)
{
  eepromWrite(0x0,sizeof(_nvm_param_t),&appParam.nvm);
}

void load_param(void)
{
  eepromRead(0x0,sizeof(_nvm_param_t),&appParam.nvm);
  if(strncmp(EEP_FLAG,appParam.nvm.flag,4) != 0){
    memcpy(&appParam.nvm,&default_nvm_param,sizeof(_nvm_param_t));
    save_param();
  }
}


void app_2004_do_write(uint8_t ch, uint8_t v)
{
  switch(ch){
  case 0:
    if(v == 0)
      palClearPad(GPIOC,13);
    else
      palSetPad(GPIOC,13);
    break;
  case 1:
    if(v == 0)
      palClearPad(GPIOA,3);
    else
      palSetPad(GPIOA,3);
    break;
  default:
    break;
  }
}

void set_da_output(uint8_t ch, float v)
{
  pwmEnableChannel(&PWMD2, pwm_map[ch], map_pwm_output(ch,v));
}

void set_da_output_raw(uint8_t ch, pwmcnt_t raw)
{
  pwmEnableChannel(&PWMD2, pwm_map[ch], raw);
}
static void blinker_cb(void *arg)
{
  led_map_t *led = (led_map_t*)arg;
  chSysLockFromISR();
  if(led->ms_on == 0){
    palSetLine(led->line);
    chVTSetI(&led->vt,TIME_MS2I(1000),blinker_cb,led);
  }
  else if(led->ms_off == 0){
    palClearLine(led->line);
    chVTSetI(&led->vt,TIME_MS2I(1000),blinker_cb,led);
  }
  else{
    if(palReadLine(led->line) == PAL_HIGH){
      palClearLine(led->line);
      chVTSetI(&led->vt,TIME_MS2I(led->ms_off),blinker_cb,led);
    }
    else{
      palSetLine(led->line);
      chVTSetI(&led->vt,TIME_MS2I(led->ms_on),blinker_cb,led);
    }
  }
  chSysUnlockFromISR();
}

void lora_callback(msg_t msg)
{

}
//#define LORA_ENABLE
void ads1x15_drdy(void *arg);
static event_source_t ads_rdy_event;
static THD_WORKING_AREA(waADS, 512);
static THD_FUNCTION(procADS, arg) {
  (void)arg;
  event_listener_t el_adsrdy;
  chEvtObjectInit(&ads_rdy_event);
  chEvtRegister(&ads_rdy_event,&el_adsrdy,2);

  appParam.runtime.analog_channel = 0;
  ads1015_set_mux(&ads1015,MUX_AIN0_GND);
//  ads1015_set_pga(&ads1015,PGA_2_048);
  ads1015_set_pga(&ads1015,PGA_4_096);
  ads1015_set_mode(&ads1015,MODE_CONT);
  ads1015_set_dr(&ads1015,DR_250);
  ads1015_set_thresLow(&ads1015);
  ads1015_set_thresHigh(&ads1015);
  //ads1015_config(&ads1015);
  // config adc drdy signal
  palEnableLineEvent(PAL_LINE(GPIOA, 15U),PAL_EVENT_MODE_FALLING_EDGE);
  palSetLineCallback(PAL_LINE(GPIOA, 15U),ads1x15_drdy,NULL);

  ads1015_start_conversion(&ads1015);
  while(1){
    chThdSleepMilliseconds(50);
//    chEvtWaitOne(EVENT_MASK(2));
//    chSysLock();
//    eventflags_t flags = chEvtGetAndClearFlags(&el_adsrdy);
//    chSysUnlock();
//    if(flags & EVENT_MASK(2)){
      appParam.runtime.analogIn_raw[appParam.runtime.analog_channel++] = ads1015_read_data(&ads1015);
      if(appParam.runtime.analog_channel == 4)
        appParam.runtime.analog_channel = 0;
      ads1015_set_mux(&ads1015,MUX_AIN0_GND+appParam.runtime.analog_channel);
//    }
  }
  chThdExit(0);
}
static THD_WORKING_AREA(waCmdParser, 2048);
static THD_FUNCTION(procCmdParse, arg) {

  (void)arg;
  chRegSetThreadName("CMD_PARSER");
  eventflags_t flags;
  uint8_t msgBuf[128];
  uint8_t txBuf[128];
  uint16_t duty = 0;
  uint32_t bau = appParam.nvm.serialConfig.baudrate;
  if((bau == 2400) || (bau == 9600) || (bau == 19200) || (bau = 38400) || (bau == 57600) || (bau == 115200)){
    serialCfg.speed = bau;
  }else{
    serialCfg.speed = 9600;
  }
  sdStart(&SD1,&serialCfg);
  uint8_t rxIndex,wrIndex;

  event_listener_t elSerialData;

  chEvtRegisterMask((event_source_t*)chnGetEventSource(&SD1),&elSerialData,EVENT_MASK(1));

  rxIndex = wrIndex = 0;
  bool validPacket = false;
  uint8_t query_target = QUERY_CHA;
  uint8_t waitResp = 0;
  for(uint8_t i=0;i<3;i++){
    appParam.runtime.duty[i] = 0;
  }
  uint8_t test = 0;
  uint16_t cycletime = appParam.nvm.module.cycletime;
  if(cycletime < 100) cycletime = 100;
  bool enablePoll = true;
  uint16_t pollTimeout = 0;
#ifdef LORA_ENABLE
  // start lora
  spiStart(&SPID2,&spicfg_lora);
  lora_osi_init();
  osi_lora_set_callback(lora_callback);
  osi_lora_set_rx(0);
#endif
  char msg[64];
  msg[0] = msg[1] = 0;
  msg[2] = msg[3] = 0x1;
  uint16_t pid = 0;
  while (true) {
    chEvtWaitOneTimeout(EVENT_MASK(1),TIME_MS2I(10));
    chSysLock();
    flags = chEvtGetAndClearFlags(&elSerialData);
    chSysUnlock();
    if(flags & CHN_INPUT_AVAILABLE){
      msg_t charBuf;
      do{
        charBuf = chnGetTimeout(&SD1,TIME_IMMEDIATE);
        if(charBuf != Q_TIMEOUT){
          if(charBuf == 0x02){
            wrIndex = 0;
          }else if(charBuf == 0x03){
            validPacket = true;
          }else{
            msgBuf[(wrIndex++) & MSG_BUF_MASK] = (char)charBuf;
          }
        }
      }while(charBuf != Q_TIMEOUT);

      // buffer valid
      if(validPacket){
        validPacket = false;
        if(strncmp(msgBuf,"01rP",4)==0){
          waitResp = 0;
          float v = 0;
          v = (msgBuf[6]-0x30)*10+(msgBuf[7]-0x30);
          v += (msgBuf[8]-0x30)*0.1;
          //v += (msgBuf[10]-0x30)*0.01;
//          v += (msgBuf[11]-0x30)*0.1;
          if(msgBuf[5] == 'F')
            v *= -1;

          switch(msgBuf[4]){
          case 'A':
//            query_target = QUERY_CHB;
            // update pwm
            set_da_output(0,v);
            break;
          case 'B':
            //query_target = QUERY_CHA;
            set_da_output(1,v);
            break;
          default:break;
//            query_target = QUERY_CHA;
          }
          pollTimeout = 0;
        }
        else if(strncmp(msgBuf,"02w",3)==0){ // write param 02wxx111122223333333344444444, x:ch, 1: pwm low, 2: pwm high, 3:eng low, 4:eng high
          uint16_t v1,v2;
          float f1,f2;
          uint8_t *ptr = &msgBuf[3];
          uint8_t id;// = msgBuf[4] - 0x30;
          ptr = extract_val_from_hexASC(ptr,&id,1);
          if((id <= 2)){
            ptr = extract_val_from_hexASC(ptr,(uint8_t*)&v1,2);
            ptr = extract_val_from_hexASC(ptr,(uint8_t*)&v2,2);
            ptr = extract_val_from_hexASC(ptr,(uint8_t*)&f1,4);
            ptr = extract_val_from_hexASC(ptr,(uint8_t*)&f2,4);
            appParam.nvm.da_config[id].zeroValue = v1;
            appParam.nvm.da_config[id].fullValue = v2;
            appParam.nvm.da_config[id].engLow = f1;
            appParam.nvm.da_config[id].engHigh = f2;
            // save parameters
            save_param();
          }
          else if(id == 0x80){
            uint8_t v3;
            ptr = extract_val_from_hexASC(ptr,(uint8_t*)&v3,1);
            ptr = extract_val_from_hexASC(ptr,(uint8_t*)&v2,2);

            appParam.nvm.module.query_wait = v3;
            appParam.nvm.module.cycletime = v2;
            save_param();
          }
          else if(id == 0x90){
            uint8_t v3;
            ptr = extract_val_from_hexASC(ptr,(uint8_t*)&v3,1);
            enablePoll = v3==0?false:true;
            if(enablePoll){
              pollTimeout = 0;
            }
          }
          else if(id == 0xf0){
            uint8_t v3;
            ptr = extract_val_from_hexASC(ptr,(uint8_t*)&v3,1);
            if(v3 == 0x01){ // load default
              memcpy(&appParam.nvm,&default_nvm_param,sizeof(_nvm_param_t));
              save_param();
            }
            else if(v3 == 0xff){ // reset

            }

          }
        }
        else if(strncmp(msgBuf,"02r",3)==0){ // read param 02rxx, xx: index
          uint16_t v1,v2;
          float f1,f2;
          uint8_t *ptr = &msgBuf[3];
          uint8_t *wptr = txBuf;
          uint8_t id;
          ptr = extract_val_from_hexASC(ptr,&id,1);
          if(id < 2){
            v1 = appParam.nvm.da_config[id].zeroValue ;
            v2 = appParam.nvm.da_config[id].fullValue ;
            f1 = appParam.nvm.da_config[id].engLow ;
            f2 = appParam.nvm.da_config[id].engHigh;

            *wptr++ = 0x02;
            memcpy(wptr,"02r",3);
            wptr += 3;
            wptr = encode_ascii_from_hex(&id,wptr,1);
            wptr = encode_ascii_from_hex((uint8_t*)&v1,wptr,2);
            wptr = encode_ascii_from_hex((uint8_t*)&v2,wptr,2);
            wptr = encode_ascii_from_hex((uint8_t*)&f1,wptr,4);
            wptr = encode_ascii_from_hex((uint8_t*)&f2,wptr,4);
            *wptr++ = 0x3;
            chnWriteTimeout(&SD1,txBuf,(wptr - txBuf +1),TIME_MS2I(100));
          }else if(id == 0x80){
            uint8_t v3;
            v3 = appParam.nvm.module.query_wait;
            v2 = appParam.nvm.module.cycletime;
            *wptr++ = 0x02;
            memcpy(wptr,"02r",3);
            wptr += 3;
            wptr = encode_ascii_from_hex(&id,wptr,1);
            wptr = encode_ascii_from_hex(&v3,wptr,1);
            wptr = encode_ascii_from_hex((uint8_t*)&v1,wptr,2);
            *wptr++ = 0x3;
            chnWriteTimeout(&SD1,txBuf,(wptr - txBuf +1),TIME_MS2I(100));
          }
        }
        else if(strncmp(msgBuf,"02t",3) == 0){ // test command, 02txxyyyy, xx: channel, 0/1/2, yyyy uint16 value
          uint8_t ch;
          uint16_t val;
          uint8_t *ptr = &msgBuf[3];
          ptr = extract_val_from_hexASC(ptr,&ch,1);
          ptr = extract_val_from_hexASC(ptr,(uint8_t*)&val,2);
          if((ch >=0) && (ch <=2)){
            set_da_output_raw(ch,val);
          }
          else if((ch >= 0x10) && (ch <= 0x11)){
            uint8_t id = ch - 0x10;
            app_2004_do_write(id,val==0?0:1);
          }

        }
        else if(strncmp(msgBuf,"02s",3)==0){  // serial config, 02siibbbb, ii: id, bbbb: baudrate
          uint8_t id;
          uint16_t baud;
          uint8_t *ptr = &msgBuf[3];
          uint8_t save = 0;
          ptr = extract_val_from_hexASC(ptr,&id,1);
          ptr = extract_val_from_hexASC(ptr,(uint8_t*)&baud,2);
          if((id >0) && (id < 250)){
            appParam.nvm.serialConfig.cfg[0] = id;
            save = 1;
          }
          if((baud == 96) || (baud == 192) || (baud == 384) ||(baud == 576)||(baud == 1152)){
            appParam.nvm.serialConfig.baudrate = baud*100;
            save  = 1;
          }

          if(save){
            save_param();
          }
        }
      }
    } // end serial input available
    if(waitResp == 0){
      query_target++;
      if(query_target == QUERY_NUMBER) query_target = QUERY_CHA;
      if(enablePoll){
        pollTimeout++;
        chnWriteTimeout(&SD1,query_str[query_target],QUERY_PACKET_SZ,TIME_MS2I(100));
#ifdef LORA_ENABLE
        msg[4] = sprintf(&msg[5],"T=%5.1f,RH=%5.1f,PID=%d\n",11.2,22.3,pid++);
        lora_send(msg,msg[4]+4);
#endif
      }
      waitResp = appParam.nvm.module.query_wait;
    }else{
      waitResp--;
    }
    if(test == 1){
      uint16_t val = 0;
      set_da_output_raw(0,val);
      set_da_output_raw(1,val);
    }

    if(pollTimeout < 5){ // normal
      appParam.runtime.leds[LED_R].ms_on = appParam.runtime.leds[LED_R].ms_off = 500;
      appParam.runtime.leds[LED_G].ms_off = 0;
      appParam.runtime.leds[LED_G].ms_on = 0xffff;
      
      appParam.runtime.leds[LED_AUXG].ms_off = 0;
      appParam.runtime.leds[LED_AUXG].ms_on = 0xffff;
      appParam.runtime.leds[LED_AUXR].ms_on = appParam.runtime.leds[LED_AUXR].ms_off = 500;
    }else{ // device lost?
      appParam.runtime.leds[LED_R].ms_on = appParam.runtime.leds[LED_R].ms_off = 250;
      appParam.runtime.leds[LED_G].ms_on = 0;
      appParam.runtime.leds[LED_G].ms_off = 0xffff;

      appParam.runtime.leds[LED_AUXR].ms_on = appParam.runtime.leds[LED_AUXR].ms_off = 250;

      appParam.runtime.leds[LED_AUXG].ms_on = 0;
      appParam.runtime.leds[LED_AUXG].ms_off = 0xffff;
    }
    chThdSleepMilliseconds(cycletime);
  }
}


void app_2004_init(void)
{
  at24eep_init(&I2CD1,32,1024,0x50,2);
  load_param();

  for(uint8_t i=0;i<4;i++){
    appParam.runtime.leds[i].line = leds[i];
    appParam.runtime.leds[i].ms_on = 0;
    appParam.runtime.leds[i].ms_off = 0xffff;
    // create vt
    chVTObjectInit(&appParam.runtime.leds[i].vt);
    chVTSet(&appParam.runtime.leds[i].vt,TIME_MS2I(1000),blinker_cb,&appParam.runtime.leds[i]);
  }

  pwmStart(&PWMD2,&pwmcfg);
  pwmEnableChannel(&PWMD2, pwm_map[0], appParam.nvm.da_config[0].zeroValue);
  pwmEnableChannel(&PWMD2, pwm_map[1], appParam.nvm.da_config[1].zeroValue);
  pwmEnableChannel(&PWMD2, pwm_map[2], appParam.nvm.da_config[2].zeroValue);

  self = chThdCreateStatic(waCmdParser, sizeof(waCmdParser), NORMALPRIO, procCmdParse, NULL);
  //chThdCreateStatic(waADS, sizeof(waADS), NORMALPRIO, procADS, NULL);
}

void ads1x15_drdy(void *arg)
{
  (void)arg;
  chSysLockFromISR();
  chEvtBroadcastI(&ads_rdy_event);
  chSysUnlockFromISR();
}

int main(void) {
  halInit();
  chSysInit();

  app_2004_init();
  while (true) {
    chThdSleepMilliseconds(1000);
  }
}

