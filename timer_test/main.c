//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Timer Demo
// Application Overview - This application is to showcases the usage of Timer 
//                        DriverLib APIs. The objective of this application is 
//                        to showcase the usage of 16 bit timers to generate 
//                        interrupts which in turn toggle the state of the GPIO 
//                        (driving LEDs).
//                        Two timers with different timeout value(one is twice 
//                        the other) are set to toggle two different GPIOs which 
//                        in turn drives two different LEDs, which will give a 
//                        blinking effect.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup timer_demo
//! @{
//
//*****************************************************************************

// Standard include
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "timer.h"
#include "utils.h"
#include "gpio.h"
#include "uart_if.h"

// Common interface includes
#include "timer_if.h"
#include "gpio_if.h"



#include "pinmux.h"


//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION        "1.4.0"
#define FOREVER                    1


//
// HC-SR04 trigger pin: PIN_15 --> GPIOA2, GPIO_PIN_6
//
#define HC_SR04_TRIG_BASE    GPIOA2_BASE
#define HC_SR04_TRIG_PIN     GPIO_PIN_6


//*****************************************************************************
//                      Global Variables for Vector Table
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//*****************************************************************************
//
// Globals used by the timer interrupt handler.
//
//*****************************************************************************
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;
int startFlag = 1;


//****************************************************************
// TriggerPulse
//
// Generates a ~10 us HIGH on HC_SR04_TRIG_PIN.
//*************************************************************
void
TriggerPulse(void)
{
    // Drive trigger HIGH
    MAP_GPIOPinWrite(HC_SR04_TRIG_BASE, HC_SR04_TRIG_PIN, HC_SR04_TRIG_PIN);
    //
    // Delay ~10 us: Assuming an 80 MHz clock, each loop of UtilsDelay(~8) is ~1 µs.
    // You may need to tune this constant based on actual clock speed.
    //
    //GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    //Report("On\r\n");
    MAP_UtilsDelay(100);
    //GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
    //Report("Off\r\n");
    //
    // Drive trigger LOW
    //
    MAP_GPIOPinWrite(HC_SR04_TRIG_BASE, HC_SR04_TRIG_PIN, 0);
}


//*****************************************************************************
// The interrupt handler for the first timer interrupt.
//*****************************************************************************

void
TimerBaseIntHandler(void)
{

if(startFlag) {
        MAP_TimerIntClear(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
        return;
    }

    // Clear the timer interrupt.
    //Timer_IF_InterruptClear(g_ulBase);
    MAP_TimerIntClear(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);


    g_ulTimerInts ++;
    //GPIO_IF_LedToggle(MCU_GREEN_LED_GPIO);

    TriggerPulse();
    //Report("Pulse %lu \r\n", g_ulTimerInts);
}


//*****************************************************************************
// The interrupt handler for the second timer interrupt.
//*****************************************************************************
void
TimerRefIntHandler(void)
{

    // Clear timer interrupt.
    Timer_IF_InterruptClear(g_ulRefBase);

    g_ulRefTimerInts ++;
    GPIO_IF_LedToggle(MCU_RED_LED_GPIO);
}

//****************************************************************************
// Board Initialization & Configuration
//*****************************************************************************
static void
BoardInit(void)
{
#ifndef USE_TIRTOS
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*************************************************************************
//    main function uses timer to generate periodic interrupts.
//**************************************************************************

//********* **********************************************
//Echo pin
//*********************************

static volatile unsigned long ulEchoStart = 0;
static volatile unsigned long ulEchoEnd   = 0;

void
EchoIntHandler(void)
{
    // 1) Read & clear the interrupt flag
    if(startFlag) {
        MAP_GPIOIntClear(GPIOA1_BASE, 0x1);
        return;
    }
    unsigned long ulStatus = MAP_GPIOIntStatus(GPIOA1_BASE, true);
    MAP_GPIOIntClear(GPIOA1_BASE,0x1);
    //Report("Literally anything.\r\n");
    if (ulStatus & 0x1) {
        // Read current pin level to distinguish rising vs. falling
        if (MAP_GPIOPinRead(GPIOA1_BASE, 0x1)) {
            // Rising edge --> record start time
            ulEchoStart = MAP_TimerValueGet(TIMERA1_BASE, TIMER_A);
            //Report("Rising\r\n");
        } else {
            // Falling edge --> record end time

            ulEchoEnd = MAP_TimerValueGet(TIMERA1_BASE, TIMER_A);
            //Report("Falling\r\n");
            // Compute pulse width in microseconds
            unsigned long ulDelta;
            if (ulEchoStart >= ulEchoEnd) {
                ulDelta = ulEchoStart - ulEchoEnd;
            } else {
                // Timer wrapped around
                ulDelta = ulEchoStart + (0xFFFFFFFF - ulEchoEnd) + 1;
            }


            // Convert to distance in inches:
            // distance_in = (ulDelta * 0.00675)
            //  since 0.00675 in/us = (34300 cm/s / 2.54 cm/in) / 2 / 1e6 s/us

            //============================================================================
            //IMPORTANT CALIBRATION VALUE: scalar calibration based on measured vs true values.
            unsigned long distanceCal = 39/36;
            //=============================================================================

            unsigned long ulDistIn = distanceCal * (ulDelta * 675UL) / 100000UL;

            // 5) Now ulDistIn holds the one-way distance in inches.
            Report("Distance: %u in\r\n", ulDistIn);

        }
    }
}

int
main(void)
{
    unsigned long g_ulBase = TIMERA0_BASE;
    unsigned long g_ulEchoTimerBase = TIMERA1_BASE;

    // INIT
    BoardInit();
    PinMuxConfig();
    InitTerm();
    ClearTerm();
    Report("this works!\r\n");

    //Echo-pin interrupt

    MAP_GPIOIntRegister(GPIOA1_BASE, EchoIntHandler);
    MAP_GPIOIntTypeSet(GPIOA1_BASE, 0x1, GPIO_BOTH_EDGES);
    MAP_GPIOIntClear(GPIOA1_BASE, 0x1);
    MAP_GPIOIntEnable(GPIOA1_BASE, 0x1);
    MAP_IntEnable(INT_GPIOA1);


//    MAP_GPIOIntEnable(GPIOA0_BASE, GPIO_PIN_2);
//    MAP_GPIOIntTypeSet(GPIOA0_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);
//    MAP_GPIOIntRegister(GPIOA0_BASE, EchoIntHandler);


    //TimerA1 as 1us running counter

    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA1, PRCM_RUN_MODE_CLK);
    MAP_TimerConfigure(g_ulEchoTimerBase, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    MAP_TimerPrescaleSet(g_ulEchoTimerBase, TIMER_A, 79);
    MAP_TimerLoadSet(g_ulEchoTimerBase, TIMER_A, 0xFFFFFFFF);
    MAP_TimerEnable(g_ulEchoTimerBase, TIMER_A);


    //TimerA0 for periodic trigger pulses (every 500 ms here), 2Hz

                 //TESTING WITHOUT THESE INTERRUPTS
    g_ulBase = TIMERA0_BASE;
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);
    Timer_IF_Start(g_ulBase, TIMER_A, 500);

    startFlag = 0;

    while (1)
    {
        // We can add any additional features here since everything else is interrupt-driven.
    }
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
