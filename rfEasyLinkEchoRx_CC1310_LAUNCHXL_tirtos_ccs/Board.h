/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/Power.h>

#include <ti/drivers/UART.h>
#include <ti/drivers/Watchdog.h>

#include "CC1310DK_5XD.h"


/* These #defines allow us to reuse TI-RTOS across other device families */
#define     Board_LED3              Board_DK_LED3
#define     Board_LED4              Board_DK_LED4

#define     Board_LED0              Board_DK_LED3
#define     Board_LED1              Board_DK_LED4
#define     Board_LED2              Board_DK_LED4

#define     Board_BUTTON0           Board_KEY_UP
#define     Board_BUTTON1           Board_KEY_DOWN

#define     Board_UART0             Board_UART
#define     Board_AES0              Board_AES
#define     Board_WATCHDOG0         CC1310_5XD_WATCHDOG0

#define     Board_initGeneral() { \
    Power_init(); \
    if (PIN_init(BoardGpioInitTable) != PIN_SUCCESS) \
        {System_abort("Error with PIN_init\n"); \
    } \
}

#define     Board_initGPIO()
#define     Board_initPWM()        PWM_init()
#define     Board_initSPI()         SPI_init()
#define     Board_initUART()        UART_init()
#define     Board_initWatchdog()    Watchdog_init()
#define     Board_initADCBuf()      ADCBuf_init()
#define     GPIO_toggle(n)
#define     GPIO_write(n,m)

#ifdef __cplusplus
}
#endif

/* Product Fubction Types*/
//#define TEST_ALLOW
//#define TEG_TRUE_RMS
//#define TEG_REF_V148
//#define TEG_STAY_AWAKE
//#define TEG_CT_CHARGE
//#define TEG_EDGE
//#define TEG_PROTOCOL_20
//#define TEG_PROTOCOL_30
//#define TEG_CM0X_0B // TEG_CT_CHARGE
//#define TEG_CM0X_AB // TEG_CT_CHARGE, TEG_EDGE, TEG_PROTOCOL_30
//#define TEG_FM0X_0B // TEG_TRUE_RMS
//#define TEG_FM0X_AB // TEG_TRUE_RMS, TEG_REF_V148, TEG_EDGE
#define TEG_FM00_0B
//#define TEG_FM05_0B
//#define TEG_FM06_0B
//#define TEG_FM07_0B
//#define TEG_CM03_2B
//#define TEG_CM04_2B
#if defined( TEG_CM0X_0B )
    #define FW_VERSION 0x16
    #define TEG_CT_CHARGE
#elif defined( TEG_CM0X_AB )
    #define FW_VERSION 0x51
    #define TEG_CT_CHARGE
    #define TEG_EDGE
    #define TEG_PROTOCOL_20
#elif defined( TEG_FM0X_0B )
    #define FW_VERSION 0x16
    #define TEG_TRUE_RMS
#elif defined( TEG_FM0X_AB )
    //#define FW_VERSION 0x51
    #define FW_VERSION 0x52 //修正零電流有值問題
    #define TEG_TRUE_RMS
    #define TEG_REF_V148
    #define TEG_EDGE
    #define TEG_PROTOCOL_20
#elif defined( TEG_FM00_0B )
    #define FW_VERSION 0x10
    #define TEG_TRUE_RMS
    #define TEG_REF_V148
    #define TEG_EDGE
    #define TEG_PROTOCOL_30
//    #define TEG_RANGE0_MAX 2222 //50A(2020)*110%=55A(2222), dgt(50/2020)=0.0248
    #define TEG_RANGE0_MAX 1778 //40A(1616)*110%=44A(1778), dgt(50/2020)=0.0248
//    #define TEG_RANGE1_MIN 243 //50A(270)*90%=45A(243), dgt(500/2700)=0.1852
    #define TEG_RANGE1_MIN 194 //40A(216)*90%=36A(194), dgt(500/2700)=0.1852
    #define TEG_RANGE_HIGHT 1 //0.1mA
    #define TEG_RANGE_LOW 2 //0.01mA
    #define TEG_RANGELOW_MAX 5000 //50.00A
    #define TEG_RANGE0_ZERO 45
    #define TEG_RANGE_MULTIPLE 10
#elif defined( TEG_FM05_0B )
    #define FW_VERSION 0x10
    #define TEG_TRUE_RMS
    #define TEG_REF_V148
    #define TEG_EDGE
    #define TEG_PROTOCOL_30
    #define TEG_RANGE0_MAX 2336 //100A(2124)*1010%=110A(2336), dgt(100.00/2124)=0.0471
    #define TEG_RANGE1_MIN 243 //100A(276)*90%=90A(248), dgt(1000.0/2690)=0.37
    #define TEG_RANGE_HIGHT 1 //0.1mA
    #define TEG_RANGE_LOW 2 //0.01mA
    #define TEG_RANGELOW_MAX 10000 //100.00A
    #define TEG_RANGE0_ZERO 45
#elif defined( TEG_FM06_0B )
    #define FW_VERSION 0x10
    #define TEG_TRUE_RMS
    #define TEG_REF_V148
    #define TEG_EDGE
    #define TEG_PROTOCOL_30
    #define TEG_RANGE0_MAX 2336 //100A(2124)*1010%=110A(2336), dgt(100.00/2124)=0.0471
    #define TEG_RANGE1_MIN 124 //100A(138)*90%=90A(124), dgt(2000.0/2690)=0.7435
    #define TEG_RANGE_HIGHT 1 //0.1mA
    #define TEG_RANGE_LOW 2 //0.01mA
    #define TEG_RANGELOW_MAX 10000 //100.00A
    #define TEG_RANGE0_ZERO 45
#elif defined( TEG_FM07_0B )
    #define FW_VERSION 0x10
    #define TEG_TRUE_RMS
    #define TEG_REF_V148
    #define TEG_EDGE
    #define TEG_PROTOCOL_30
    #define TEG_RANGE0_MAX 2336 //100A(2124)*1010%=110A(2336), dgt(100.00/2124)=0.0471
    #define TEG_RANGE1_MIN 82 //100A(92)*90%=90A(82), dgt(3000/2690)=1.1152
    #define TEG_RANGE_HIGHT 1 //0.1mA
    #define TEG_RANGE_LOW 2 //0.01mA
    #define TEG_RANGELOW_MAX 10000 //100.00A
    #define TEG_RANGE0_ZERO 45
#elif defined( TEG_CM03_2B )
    #define FW_VERSION 0x10
    #define TEG_CM0X
    #define TEG_TRUE_RMS
    #define TEG_REF_V148
    #define TEG_EDGE
    #define TEG_PROTOCOL_30
//    #define TEG_RANGE0_MAX 2968 //30A(2827)*105%=31.5A(2968), dgt(30/2827)=0.0106
    #define TEG_RANGE0_MAX 2177 //20A(1979)*110%=22A(2177), dgt(30/2827)=0.0106
//    #define TEG_RANGE1_MIN 247 //30A(274)*90%=27A(247), dgt(300/2740)=0.109
    #define TEG_RANGE1_MIN 165 //20A(183)*90%=18A(165), dgt(300/2740)=0.109
    #define TEG_RANGE_HIGHT 1 //0.1mA
    #define TEG_RANGE_LOW 2 //0.01mA
    #define TEG_RANGELOW_MAX 3000 //30.00A
    #define TEG_RANGE0_ZERO 9
    #define TEG_CT_CHARGE
#elif defined( TEG_CM04_2B )
    #define FW_VERSION 0x10
    #define TEG_CM0X
    #define TEG_TRUE_RMS
    #define TEG_REF_V148
    #define TEG_EDGE
    #define TEG_PROTOCOL_30
//    #define TEG_RANGE0_MAX 2968 //30A(2827)*105%=31.5A(2968), dgt(30/2827)=0.0106
    #define TEG_RANGE0_MAX 2177 //20A(1979)*110%=22A(2177), dgt(30/2827)=0.0106
//    #define TEG_RANGE1_MIN 185 //30A(206)*90%=27A(185), dgt(400/2740)=0.146
    #define TEG_RANGE1_MIN 123 //20A(137)*90%=18A(123), dgt(400/2740)=0.109
    #define TEG_RANGE_HIGHT 1 //0.1mA
    #define TEG_RANGE_LOW 2 //0.01mA
    #define TEG_RANGELOW_MAX 3000 //30.00A
    #define TEG_RANGE0_ZERO 9
    #define TEG_CT_CHARGE
#endif

#endif /* __BOARD_H */
