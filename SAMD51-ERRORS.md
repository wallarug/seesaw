./bsp/bsp_gpio.h: In function 'void gpio_reset_eic()':
./bsp/bsp_gpio.h:16:7: error: 'struct Eic' has no member named 'CTRL'; did you mean 'CTRLA'?
  EIC->CTRL.bit.SWRST = 1;
       ^~~~
       CTRLA
./bsp/bsp_gpio.h:17:13: error: 'struct Eic' has no member named 'CTRL'; did you mean 'CTRLA'?
  while(EIC->CTRL.bit.SWRST || EIC->STATUS.bit.SYNCBUSY);
             ^~~~
             CTRLA
./bsp/bsp_gpio.h:17:36: error: 'struct Eic' has no member named 'STATUS'
  while(EIC->CTRL.bit.SWRST || EIC->STATUS.bit.SYNCBUSY);
                                    ^~~~~~
In file included from source/bsp.cpp:43:0:
./bsp/bsp_adc.h: In function 'void syncADC()':
./bsp/bsp_adc.h:9:32: error: 'ADC' was not declared in this scope
 inline void syncADC() { while (ADC->STATUS.bit.SYNCBUSY == 1); }
                                ^~~
./bsp/bsp_adc.h:9:32: note: suggested alternative: 'AC'
 inline void syncADC() { while (ADC->STATUS.bit.SYNCBUSY == 1); }
                                ^~~
                                AC
./bsp/bsp_adc.h: In function 'void adc_trigger()':
./bsp/bsp_adc.h:10:39: error: 'ADC' was not declared in this scope
 inline void adc_trigger(){ syncADC(); ADC->SWTRIG.bit.START = 1;}
                                       ^~~
./bsp/bsp_adc.h:10:39: note: suggested alternative: 'AC'
 inline void adc_trigger(){ syncADC(); ADC->SWTRIG.bit.START = 1;}
                                       ^~~
                                       AC
source/bsp.cpp: In function 'void BspInit()':
source/bsp.cpp:75:6: error: 'struct Pm' has no member named 'APBCMASK'
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_TC1 | PM_APBCMASK_TC2;
      ^~~~~~~~
source/bsp.cpp:75:22: error: 'PM_APBCMASK_SERCOM0' was not declared in this scope
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_TC1 | PM_APBCMASK_TC2;
                      ^~~~~~~~~~~~~~~~~~~
source/bsp.cpp:75:22: note: suggested alternative: 'MCLK_APBAMASK_SERCOM0'
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_TC1 | PM_APBCMASK_TC2;
                      ^~~~~~~~~~~~~~~~~~~
                      MCLK_APBAMASK_SERCOM0
source/bsp.cpp:75:44: error: 'PM_APBCMASK_SERCOM1' was not declared in this scope
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_TC1 | PM_APBCMASK_TC2;
                                            ^~~~~~~~~~~~~~~~~~~
source/bsp.cpp:75:44: note: suggested alternative: 'MCLK_APBAMASK_SERCOM1'
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_TC1 | PM_APBCMASK_TC2;
                                            ^~~~~~~~~~~~~~~~~~~
                                            MCLK_APBAMASK_SERCOM1
source/bsp.cpp:75:66: error: 'PM_APBCMASK_TC1' was not declared in this scope
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_TC1 | PM_APBCMASK_TC2;
                                                                  ^~~~~~~~~~~~~~~
source/bsp.cpp:75:66: note: suggested alternative: 'MCLK_APBCMASK_TC4'
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_TC1 | PM_APBCMASK_TC2;
                                                                  ^~~~~~~~~~~~~~~
                                                                  MCLK_APBCMASK_TC4
source/bsp.cpp:75:84: error: 'PM_APBCMASK_TC2' was not declared in this scope
 eg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_TC1 | PM_APBCMASK_TC2;
                                                                     ^~~~~~~~~~~~~~~
source/bsp.cpp:75:84: note: suggested alternative: 'MCLK_APBBMASK_TC2'
 eg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_TC1 | PM_APBCMASK_TC2;
                                                                     ^~~~~~~~~~~~~~~
                                                                                    MCLK_APBBMASK_TC2
source/bsp.cpp:79:6: error: 'struct Pm' has no member named 'APBCMASK'
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0 ;
      ^~~~~~~~
source/bsp.cpp:79:22: error: 'PM_APBCMASK_TCC0' was not declared in this scope
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0 ;
                      ^~~~~~~~~~~~~~~~
source/bsp.cpp:79:22: note: suggested alternative: 'MCLK_APBCMASK_TCC2'
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0 ;
                      ^~~~~~~~~~~~~~~~
                      MCLK_APBCMASK_TCC2
source/bsp.cpp:83:6: error: 'struct Pm' has no member named 'APBCMASK'
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM2 ;
      ^~~~~~~~
source/bsp.cpp:83:22: error: 'PM_APBCMASK_SERCOM2' was not declared in this scope
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM2 ;
                      ^~~~~~~~~~~~~~~~~~~
source/bsp.cpp:83:22: note: suggested alternative: 'MCLK_APBBMASK_SERCOM2'
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM2 ;
                      ^~~~~~~~~~~~~~~~~~~
                      MCLK_APBBMASK_SERCOM2
source/bsp.cpp:87:6: error: 'struct Pm' has no member named 'APBCMASK'
  PM->APBCMASK.reg |= PM_APBCMASK_ADC | PM_APBCMASK_DAC ;
      ^~~~~~~~
source/bsp.cpp:87:22: error: 'PM_APBCMASK_ADC' was not declared in this scope
  PM->APBCMASK.reg |= PM_APBCMASK_ADC | PM_APBCMASK_DAC ;
                      ^~~~~~~~~~~~~~~
source/bsp.cpp:87:22: note: suggested alternative: 'MCLK_APBCMASK_AC'
  PM->APBCMASK.reg |= PM_APBCMASK_ADC | PM_APBCMASK_DAC ;
                      ^~~~~~~~~~~~~~~
                      MCLK_APBCMASK_AC
source/bsp.cpp:87:40: error: 'PM_APBCMASK_DAC' was not declared in this scope
  PM->APBCMASK.reg |= PM_APBCMASK_ADC | PM_APBCMASK_DAC ;
                                        ^~~~~~~~~~~~~~~
source/bsp.cpp:87:40: note: suggested alternative: 'MCLK_APBCMASK_AC'
  PM->APBCMASK.reg |= PM_APBCMASK_ADC | PM_APBCMASK_DAC ;
                                        ^~~~~~~~~~~~~~~
                                        MCLK_APBCMASK_AC
In file included from ./include/SeesawConfig.h:4:0,
                 from ./include/PinMap.h:5,
                 from ./bsp/bsp_neopix.h:4,
                 from source/bsp.cpp:42:
source/bsp.cpp: In static member function 'static void QP::QF::onStartup()':
./boards/robohatmm1/board_config.h:180:31: error: 'SERCOM3_IRQn' was not declared in this scope
 #define CONFIG_I2C_SLAVE_IRQn SERCOM3_IRQn
                               ^
source/bsp.cpp:169:19: note: in expansion of macro 'CONFIG_I2C_SLAVE_IRQn'
  NVIC_SetPriority(CONFIG_I2C_SLAVE_IRQn, I2C_SLAVE_ISR_PRIO);
                   ^~~~~~~~~~~~~~~~~~~~~
./boards/robohatmm1/board_config.h:180:31: note: suggested alternative: 'SERCOM5_3_IRQn'
 #define CONFIG_I2C_SLAVE_IRQn SERCOM3_IRQn
                               ^
source/bsp.cpp:169:19: note: in expansion of macro 'CONFIG_I2C_SLAVE_IRQn'
  NVIC_SetPriority(CONFIG_I2C_SLAVE_IRQn, I2C_SLAVE_ISR_PRIO);
                   ^~~~~~~~~~~~~~~~~~~~~
source/bsp.cpp:182:19: error: 'SERCOM0_IRQn' was not declared in this scope
  NVIC_SetPriority(SERCOM0_IRQn, SERCOM_ISR_PRIO);
                   ^~~~~~~~~~~~
source/bsp.cpp:182:19: note: suggested alternative: 'SERCOM5_0_IRQn'
  NVIC_SetPriority(SERCOM0_IRQn, SERCOM_ISR_PRIO);
                   ^~~~~~~~~~~~
                   SERCOM5_0_IRQn
source/bsp.cpp:186:19: error: 'SERCOM1_IRQn' was not declared in this scope
  NVIC_SetPriority(SERCOM1_IRQn, SERCOM_ISR_PRIO);
                   ^~~~~~~~~~~~
source/bsp.cpp:186:19: note: suggested alternative: 'SERCOM5_1_IRQn'
  NVIC_SetPriority(SERCOM1_IRQn, SERCOM_ISR_PRIO);
                   ^~~~~~~~~~~~
                   SERCOM5_1_IRQn
source/bsp.cpp:190:19: error: 'SERCOM2_IRQn' was not declared in this scope
  NVIC_SetPriority(SERCOM2_IRQn, SERCOM_ISR_PRIO);
                   ^~~~~~~~~~~~
source/bsp.cpp:190:19: note: suggested alternative: 'SERCOM5_2_IRQn'
  NVIC_SetPriority(SERCOM2_IRQn, SERCOM_ISR_PRIO);
                   ^~~~~~~~~~~~
                   SERCOM5_2_IRQn
source/bsp.cpp:194:19: error: 'SERCOM5_IRQn' was not declared in this scope
  NVIC_SetPriority(SERCOM5_IRQn, SERCOM_ISR_PRIO);
                   ^~~~~~~~~~~~
source/bsp.cpp:194:19: note: suggested alternative: 'SERCOM5_3_IRQn'
  NVIC_SetPriority(SERCOM5_IRQn, SERCOM_ISR_PRIO);
                   ^~~~~~~~~~~~
                   SERCOM5_3_IRQn
source/bsp.cpp:237:17: error: 'USB_IRQn' was not declared in this scope
  NVIC_EnableIRQ(USB_IRQn);
                 ^~~~~~~~
source/bsp.cpp:237:17: note: suggested alternative: 'USB_3_IRQn'
  NVIC_EnableIRQ(USB_IRQn);
                 ^~~~~~~~
                 USB_3_IRQn
Makefile:200: recipe for target 'build/robohatmm1/source/bsp.o' failed
make: *** [build/robohatmm1/source/bsp.o] Error 1
