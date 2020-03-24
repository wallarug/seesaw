///***************************************************************************
// Product: DPP example, STM32 NUCLEO-L152RE board, preemptive QK kernel
// Last updated for version 5.6.5
// Last updated on  2016-07-05
//
//                    Q u a n t u m     L e a P s
//                    ---------------------------
//                    innovating embedded systems
//
// Copyright (C) Quantum Leaps, LLC. All rights reserved.
//
// This program is open source software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Alternatively, this program may be distributed and modified under the
// terms of Quantum Leaps commercial licenses, which expressly supersede
// the GNU General Public License and are specifically designed for
// licensees interested in retaining the proprietary status of their code.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.
//
// Contact information:
// http://www.state-machine.com
// mailto:info@state-machine.com
//****************************************************************************

//#include <string.h>
#include "qpcpp.h"
#include "bsp.h"
#include "bsp_nvmctrl.h"
#include "sam.h"
#include "bsp_gpio.h"
#include "bsp_sercom.h"
#include "bsp_neopix.h"
#include "bsp_adc.h"

#include "SeesawConfig.h"
#include "Delegate.h"
#include <stdlib.h>

//Q_DEFINE_THIS_FILE

#define ENABLE_BSP_PRINT

volatile uint32_t lastGPIOState = 0;
volatile uint32_t _systemMs = 0;

void operator delete(void * p)
{
  free(p);
}

void * operator new(size_t n)
{
  void * const p = malloc(n);
  // handle p == 0
  //if(p == 0) __BKPT();
  return p;
}

void BspInit() {
	//initialize some clocks
#if defined(SAMD51)
	MCLK->APBAMASK.reg |= MCLK_APBAMASK_SERCOM0 | MCLK_APBAMASK_SERCOM1 | MCLK_APBAMASK_TC0 | MCLK_APBAMASK_TC1;
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_SERCOM2 | MCLK_APBBMASK_SERCOM3 | MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1 | MCLK_APBBMASK_TC3 | MCLK_APBBMASK_TC2;
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TCC3 | MCLK_APBCMASK_TC4 | MCLK_APBCMASK_TC5;
#elif defined(SAMD21)
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 | PM_APBCMASK_SERCOM3 | PM_APBCMASK_SERCOM4 | PM_APBCMASK_SERCOM5 ;
	PM->APBCMASK.reg |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 ;
#else
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_TC1 | PM_APBCMASK_TC2;
#endif

#ifndef SAMD51
#ifdef TCC0
	PM->APBCMASK.reg |= PM_APBCMASK_TCC0 ;
#endif

#ifdef SERCOM2
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM2 ;
#endif

#ifdef DAC
	PM->APBCMASK.reg |= PM_APBCMASK_ADC | PM_APBCMASK_DAC ;
#endif
#endif

#if CONFIG_EEPROM
	eeprom_init();
#endif
/*
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC_Val));
	// Enable EIC
	EIC->CTRL.bit.ENABLE = 1;
	while (EIC->STATUS.bit.SYNCBUSY == 1) { }
		*/

#ifdef ENABLE_LOGGING
    pinPeripheral(CONFIG_LOG_UART_PIN_TX, CONFIG_LOG_UART_PIN_TX_MUX);

    initUART(CONFIG_LOG_SERCOM, SAMPLE_RATE_x16, CONFIG_LOG_UART_BAUD_RATE);
    initFrame(CONFIG_LOG_SERCOM, CONFIG_LOG_UART_CHAR_SIZE, LSB_FIRST, CONFIG_LOG_UART_PARITY, CONFIG_LOG_UART_STOP_BIT);
    initPads(CONFIG_LOG_SERCOM, CONFIG_LOG_UART_PAD_TX, CONFIG_LOG_UART_PAD_RX);

    enableUART(CONFIG_LOG_SERCOM);
#endif
}

void BspWrite(char const *buf, uint32_t len) {
	//TODO:
}

uint32_t GetSystemMs() {
    return _systemMs;
}

extern "C" {
	void SysTick_Handler(void) {
		QXK_ISR_ENTRY();
		QP::QF::tickX_(0);
		_systemMs++;
		
		//process GPIO interrupts
		uint32_t GPIOState = gpio_read_bulk();
		if( (Delegate::m_inten & GPIOState) != (Delegate::m_inten & lastGPIOState) ){
			Delegate::m_intflag |= GPIOState ^ lastGPIOState;
			Delegate::intCallback();
		}
		lastGPIOState = GPIOState;

#if defined(SAMD21)
		tickReset();
#endif
		
		QXK_ISR_EXIT();
	}

#if defined(SAMD21)	
	static void (*usb_isr)(void) = NULL;

	void USB_Handler(void)
	{
		if (usb_isr)
		usb_isr();
	}

	void USB_SetHandler(void (*new_usb_isr)(void))
	{
		usb_isr = new_usb_isr;
	}
#endif
}

// namespace QP **************************************************************
namespace QP {

// QF callbacks ==============================================================
void QF::onStartup(void) {
	
    // assigning all priority bits for preemption-prio. and none to sub-prio.
    //NVIC_SetPriorityGrouping(0U);
	
	NVIC_SetPriority(PendSV_IRQn, 0xFF);
	SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);
	NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIO);

#ifndef SAMD51
#if CONFIG_I2C_SLAVE
	NVIC_SetPriority(CONFIG_I2C_SLAVE_IRQn, I2C_SLAVE_ISR_PRIO);
#endif
#endif

#ifndef SAMD51
#if CONFIG_SPI_SLAVE
	NVIC_SetPriority(CONFIG_SPI_SLAVE_IRQn, SPI_SLAVE_ISR_PRIO);
#endif
#endif

#if defined(SAMD21)
#ifndef SAMD51
	NVIC_SetPriority(USB_IRQn, USB_ISR_PRIO);
#else
	NVIC_SetPriority(USB_0_IRQn, USB_ISR_PRIO);
	NVIC_SetPriority(USB_1_IRQn, USB_ISR_PRIO);
	NVIC_SetPriority(USB_2_IRQn, USB_ISR_PRIO);
	NVIC_SetPriority(USB_3_IRQn, USB_ISR_PRIO);
#endif
#endif
	//NVIC_SetPriority(NVMCTRL_IRQn, NVMCTRL_ISR_PRIO);

#if defined(SERCOM0)
#ifndef SAMD51
	NVIC_SetPriority(SERCOM0_IRQn, SERCOM_ISR_PRIO);
#else
	NVIC_SetPriority(SERCOM0_0_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM0_1_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM0_2_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM0_3_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
#endif
#endif

#if defined(SERCOM1)
#ifndef SAMD51
	NVIC_SetPriority(SERCOM1_IRQn, SERCOM_ISR_PRIO);
#else
	NVIC_SetPriority(SERCOM1_0_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM1_1_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM1_2_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM1_3_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
#endif
#endif

#if defined(SERCOM2)
#ifndef SAMD51
	NVIC_SetPriority(SERCOM2_IRQn, SERCOM_ISR_PRIO);
#else
	NVIC_SetPriority(SERCOM2_0_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM2_1_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM2_2_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM2_3_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
#endif
#endif

#ifdef SAMD51
	NVIC_SetPriority(SERCOM3_0_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM3_1_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM3_2_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM3_3_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
#endif

#ifdef SAMD51
	NVIC_SetPriority(SERCOM4_0_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM4_1_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM4_2_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM4_3_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
#endif

#if defined(SERCOM5)
#ifndef SAMD51
	NVIC_SetPriority(SERCOM5_IRQn, SERCOM_ISR_PRIO);
#else
	NVIC_SetPriority(SERCOM5_0_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM5_1_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM5_2_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	NVIC_SetPriority(SERCOM5_3_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
#endif
#endif

#if CONFIG_ENCODER
	NVIC_SetPriority(CONFIG_ENCODER_IRQn, ENCODER_ISR_PRIO);
#endif
    
    // set priorities of ALL ISRs used in the system, see NOTE00
    //
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!! CAUTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Assign a priority to EVERY ISR explicitly by calling NVIC_SetPriority().
    // DO NOT LEAVE THE ISR PRIORITIES AT THE DEFAULT VALUE!
    //NVIC_SetPriority(EXTI0_1_IRQn,   DPP::EXTI0_1_PRIO);
    // ...

    // enable IRQs...
    NVIC_EnableIRQ(SysTick_IRQn);
	//NVIC_EnableIRQ(NVMCTRL_IRQn);
#ifndef SAMD51
#if CONFIG_I2C_SLAVE
	NVIC_EnableIRQ(CONFIG_I2C_SLAVE_IRQn);
#endif
#endif

#ifndef SAMD51
#if CONFIG_SPI_SLAVE
	NVIC_EnableIRQ(CONFIG_SPI_SLAVE_IRQn);
#endif
#endif

#if CONFIG_SERCOM0
#ifndef SAMD51
	NVIC_EnableIRQ(SERCOM0_IRQn);
#else
	NVIC_EnableIRQ(SERCOM0_0_IRQn);
	NVIC_EnableIRQ(SERCOM0_1_IRQn);
	NVIC_EnableIRQ(SERCOM0_2_IRQn);
	NVIC_EnableIRQ(SERCOM0_3_IRQn);
#endif
#endif

#if CONFIG_SERCOM1
#ifndef SAMD51
	NVIC_EnableIRQ(SERCOM1_IRQn);
#else
	NVIC_EnableIRQ(SERCOM1_0_IRQn);
	NVIC_EnableIRQ(SERCOM1_1_IRQn);
	NVIC_EnableIRQ(SERCOM1_2_IRQn);
	NVIC_EnableIRQ(SERCOM1_3_IRQn);
#endif
#endif

#if CONFIG_SERCOM2
#ifndef SAMD51
	NVIC_EnableIRQ(SERCOM2_IRQn);
#else
	NVIC_EnableIRQ(SERCOM2_0_IRQn);
	NVIC_EnableIRQ(SERCOM2_1_IRQn);
	NVIC_EnableIRQ(SERCOM2_2_IRQn);
	NVIC_EnableIRQ(SERCOM2_3_IRQn);
#endif
#endif

#ifdef SAMD51
	NVIC_EnableIRQ(SERCOM3_0_IRQn);
	NVIC_EnableIRQ(SERCOM3_1_IRQn);
	NVIC_EnableIRQ(SERCOM3_2_IRQn);
	NVIC_EnableIRQ(SERCOM3_3_IRQn);
#endif

#ifdef SAMD51
	NVIC_EnableIRQ(SERCOM4_0_IRQn);
	NVIC_EnableIRQ(SERCOM4_1_IRQn);
	NVIC_EnableIRQ(SERCOM4_2_IRQn);
	NVIC_EnableIRQ(SERCOM4_3_IRQn);
#endif

#if CONFIG_SERCOM5
#ifndef SAMD51
	NVIC_EnableIRQ(SERCOM5_IRQn);
#else
	NVIC_EnableIRQ(SERCOM5_0_IRQn);
	NVIC_EnableIRQ(SERCOM5_1_IRQn);
	NVIC_EnableIRQ(SERCOM5_2_IRQn);
	NVIC_EnableIRQ(SERCOM5_3_IRQn);
#endif
#endif

#if CONFIG_USB
#ifndef SAMD51
	NVIC_EnableIRQ(USB_IRQn);
#else
	NVIC_EnableIRQ(USB_0_IRQn);
	NVIC_EnableIRQ(USB_1_IRQn);
	NVIC_EnableIRQ(USB_2_IRQn);
	NVIC_EnableIRQ(USB_3_IRQn);
#endif
#endif

#if CONFIG_ENCODER
	NVIC_EnableIRQ(CONFIG_ENCODER_IRQn);
#endif
}

//............................................................................
void QF::onCleanup(void) {
}
//............................................................................
void QXK::onIdle(void) {
    // toggle the User LED on and then off (not enough LEDs, see NOTE01)
    //QF_INT_DISABLE();

    //QF_INT_ENABLE();

    // Put the CPU and peripherals to the low-power mode.
    // you might need to customize the clock management for your application,
    // see the datasheet for your particular Cortex-M3 MCU.
    //
    // !!!CAUTION!!!
    // The WFI instruction stops the CPU clock, which unfortunately disables
    // the JTAG port, so the ST-Link debugger can no longer connect to the
    // board. For that reason, the call to __WFI() has to be used with CAUTION.
    //
    // NOTE: If you find your board "frozen" like this, strap BOOT0 to VDD and
    // reset the board, then connect with ST-Link Utilities and erase the part.
    // The trick with BOOT(0) is it gets the part to run the System Loader
    // instead of your broken code. When done disconnect BOOT0, and start over.
    //
    __WFI(); //  Wait-For-Interrupt
}

//............................................................................
extern "C" void Q_onAssert(char const * const module, int loc) {
	//
    // NOTE: add here your application-specific error handling
    //

    QF_INT_DISABLE();
#ifdef ENABLE_LOGGING
    char __ms[50];
    sprintf(__ms, "[%li] ***QASSERT**** ", GetSystemMs());
    writeDataUART(CONFIG_LOG_SERCOM, __ms);
    writeDataUART(CONFIG_LOG_SERCOM, module);
    sprintf(__ms, " at %i", loc);
    writeDataUART(CONFIG_LOG_SERCOM, __ms);
#endif
    __BKPT();
	while(1);
}

extern "C" void assert_failed(char const *module, int loc) {
	__BKPT();
	while(1);
}

#if defined(SAMD21)

#ifdef __cplusplus
extern "C" {
#endif

static void banzai() {
	// Disable all interrupts
	__disable_irq();
	
	//THESE MUST MATCH THE BOOTLOADER
	#define DOUBLE_TAP_MAGIC 			0xf01669efUL
	#define BOOT_DOUBLE_TAP_ADDRESS     (HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4)

	unsigned long *a = (unsigned long *)BOOT_DOUBLE_TAP_ADDRESS;
	*a = DOUBLE_TAP_MAGIC;
	
	// Reset the device
	NVIC_SystemReset() ;

	while (true);
}

static int ticks = -1;

void initiateReset(int _ticks) {
	ticks = _ticks;
}

void cancelReset() {
	ticks = -1;
}

void tickReset() {
	if (ticks == -1)
		return;
	ticks--;
	if (ticks == 0)
		banzai();
}

#ifdef __cplusplus
}
#endif
#endif


} // namespace QP

