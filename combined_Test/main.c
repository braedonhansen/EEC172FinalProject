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
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

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
#include "i2c_if.h"

// Common interface includes
#include "timer_if.h"
#include "gpio_if.h"

// OLED includes
#include "spi.h"
#include "oled/Adafruit_GFX.h"
#include "oled/Adafruit_SSD1351.h"
#include "oled/glcdfont.h"
#include "oled/oled_test.h"




#include "pinmux.h"


//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION        "1.4.0"
#define FOREVER                    1


#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100


//
// HC-SR04 trigger pin: PIN_15 --> GPIOA2, GPIO_PIN_6
//
#define HC_SR04_TRIG_BASE    GPIOA2_BASE
#define HC_SR04_TRIG_PIN     GPIO_PIN_6

// IMU-specific defines
#define IMU_ADDR        0x6A    // SA0 = 0 0x6A
#define OUTX_L_G        0x22
#define OUTX_L_A        0x28
#define CTRL1_XL        0x10
#define CTRL9_XL        0x18
#define CTRL2_G         0x11


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



void
readGyroXYZ(int16_t *gx, int16_t *gy, int16_t *gz)
{
    unsigned char buf[6];
    unsigned char reg = OUTX_L_G;
    // Point to OUTX_L_G
    I2C_IF_Write(IMU_ADDR, &reg, 1, 0);
    // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    I2C_IF_Read(IMU_ADDR, buf, 6);
    *gx = (int16_t)((buf[1] << 8) | buf[0]);
    *gy = (int16_t)((buf[3] << 8) | buf[2]);
    *gz = (int16_t)((buf[5] << 8) | buf[4]);
}

void
readAccelXYZ(int16_t *ax, int16_t *ay, int16_t *az)
{
    unsigned char buf[6];
    unsigned char reg = OUTX_L_A;
    // Point to OUTX_L_A
    I2C_IF_Write(IMU_ADDR, &reg, 1, 0);
    // Read 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    I2C_IF_Read(IMU_ADDR, buf, 6);
    *ax = (int16_t)((buf[1] << 8) | buf[0]);
    *ay = (int16_t)((buf[3] << 8) | buf[2]);
    *az = (int16_t)((buf[5] << 8) | buf[4]);
}


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

// Calibration offsets (raw LSB units, 16bit int)
static int16_t gx_cal, gy_cal, gz_cal;
static int16_t ax_cal, ay_cal, az_cal;
// Last filtered values (physical units -> float)
static float gx_last, gy_last, gz_last;
static float ax_last, ay_last, az_last;
// Buffers and raw readings
uint8_t  buf[6];
int16_t  raw_ax, raw_ay, raw_az;
int16_t  raw_gx, raw_gy, raw_gz;

static volatile unsigned long ulEchoStart = 0;
static volatile unsigned long ulEchoEnd   = 0;
float vel_x = 0, pos_x = 0;
float vel_y = 0, pos_y = 0;
float dt = 0.05;
void
EchoIntHandler(void)
{
    // Read & clear the interrupt flag
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
            float distanceCal = 39.0f / 36.0f; // true/displayed, for an inaccurate reading
            //=============================================================================

            float ulDistIn = distanceCal * ((float)ulDelta * 0.00675f);
            //Report("Distance: %u in\r\n", ulDistIn);

            //Code for IMU will go here:
            readGyroXYZ(&raw_gx, &raw_gy, &raw_gz);
            readAccelXYZ(&raw_ax, &raw_ay, &raw_az);
            //Report("raw delGyro (X,Y,Z) = %d, %d, %d LSB\r\n", raw_gx, raw_gy, raw_gz);

            // Convert to physical units (subtract offsets, apply sensitivity)
            float gx_f = ((float)(raw_gx - gx_cal)) * 0.0175f;   // +-125 dps -> 8.75 mdps/LSB
            float gy_f = ((float)(raw_gy - gy_cal)) * 0.0175f;
            float gz_f = ((float)(raw_gz - gz_cal)) * 0.0175f;
//            float gx_f = ((float)(raw_gx - gx_cal)) * 0.00875f;   // +-250 dps -> 8.75 mdps/LSB
//            float gy_f = ((float)(raw_gy - gy_cal)) * 0.00875f;
//            float gz_f = ((float)(raw_gz - gz_cal)) * 0.00875f;

            float ax_f = ((float)(raw_ax - ax_cal)) * 0.000061f;  // +-2 g -> 0.061 mg/LSB
            float ay_f = ((float)(raw_ay - ay_cal)) * 0.000061f;
            float az_f = ((float)(raw_az - az_cal)) * 0.000061f;

            //  low-pass: y[n] = 0.7*x[n] + 0.3*y[n-1]
//            gx_f = 0.7f * gx_f + 0.3f * gx_last;
//            gy_f = 0.7f * gy_f + 0.3f * gy_last;
//            gz_f = 0.7f * gz_f + 0.3f * gz_last;
//
//            ax_f = 0.7f * ax_f + 0.3f * ax_last;
//            ay_f = 0.7f * ay_f + 0.3f * ay_last;
//            az_f = 0.7f * az_f + 0.3f * az_last;

            // Save filtered values for next iteration
//            gx_last = gx_f;   gy_last = gy_f;   gz_last = gz_f;
//            ax_last = ax_f;   ay_last = ay_f;   az_last = az_f;

            // Print to UART (in dps and g)
//            Report("%u,%7.f,%7.f,%7.f,%7.f,%7.f,%7.f\r\n", ulDistIn,
                   //gx_f, gy_f, gz_f,ax_f, ay_f, az_f);
            Report("%7.1f,%7.2f,%7.2f,%7.2f,%7.3f,%7.3f,%7.3f\r\n", ulDistIn,
                   gx_f, gy_f, gz_f,ax_f, ay_f, az_f);
            // state variables (initialized to zero before you start)



            //pos_y = (int)(pos_y * 100.0f) / 100.0f;
            //Report("x=%.3f, y=%.3f\r\n", pos_x, pos_y);

//            Report("Gyro (X, Y, Z) = %7.2f, %7.2f, %7.2f dps\r\n",
//                   gx_f, gy_f, gz_f);
//            Report("Accel(X, Y, Z) = %7.3f, %7.3f, %7.3f g\r\n",
//                   ax_f, ay_f, az_f);
//            Report("----------------------------------------\r\n");
        }
    }
}

void
IMUinit(void){
    //unsigned char cfg;
    uint8_t cfg;

    // Disable I3C (CTRL9_XL = 0x18, bit0 = 1)
    cfg = 0x01;
    I2C_IF_Write(IMU_ADDR, (unsigned char[]){ CTRL9_XL, cfg }, 2, 1);

    // Configure accel --> 104 Hz, +/-2 g (CTRL1_XL = 0x10, 0x40)
    cfg = 0x40;
    I2C_IF_Write(IMU_ADDR, (unsigned char[]){ CTRL1_XL, cfg }, 2, 1);

    // Configure gyro  --> 104 Hz, +/-250 dps (CTRL2_G  = 0x11, 0x44)
    cfg = 0x44;
    I2C_IF_Write(IMU_ADDR, (unsigned char[]){ CTRL2_G,  cfg }, 2, 1);

    // Allow time for settings to take effect
    MAP_UtilsDelay(4000000);

    // Then, do what we used to do with int first; once at initalization,
    // effectively zero the IMU. Save computation during running.

    readGyroXYZ(&raw_gx, &raw_gy, &raw_gz);
    readAccelXYZ(&raw_ax, &raw_ay, &raw_az);
    gx_cal = raw_gx;
    gy_cal = raw_gy;
    gz_cal = raw_gz;
    ax_cal = raw_ax;
    ay_cal = raw_ay;
    az_cal = raw_az;
//    gx_last = gy_last = gz_last = 0.0f;
//    ax_last = ay_last = az_last = 0.0f;

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
    I2C_IF_Open(I2C_MASTER_MODE_FST);
    Report("d,gx,gy,gz,ax,ay,az\r\n");

    // Enable the SPI module clock
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    // Reset the peripheral
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    // Reset SPI interface settings.
    MAP_SPIReset(GSPI_BASE);

    // Configure SPI interface
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    // Enable SPI for communication
    MAP_SPIEnable(GSPI_BASE);

    // Initialize Adafruit OLED
    Adafruit_Init();
    fillScreen(WHITE);

    IMUinit();

//    uint8_t regval;
//    // Read WHO_AM_I (optional sanity check)
//    I2C_IF_Write(IMU_ADDR, (uint8_t[]){ 0x0F }, 1, 0);
//    I2C_IF_Read(IMU_ADDR, &regval, 1);
//    Report("WHO_AM_I = 0x%02X (should be 0x6B)\n", regval);
//
//    // Read back CTRL2_G (0x11) to confirm FS bits
//    I2C_IF_Write(IMU_ADDR, (uint8_t[]){ CTRL2_G }, 1, 0);
//    I2C_IF_Read(IMU_ADDR, &regval, 1);
//    Report("CTRL2_G = 0x%02X\n", regval);


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


    //TimerA0 for periodic trigger pulses (every 50 ms here), 20Hz

                 //TESTING WITHOUT THESE INTERRUPTS
    g_ulBase = TIMERA0_BASE;
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA0, PRCM_RUN_MODE_CLK);
    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);
    Timer_IF_Start(g_ulBase, TIMER_A, dt*1000);

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
