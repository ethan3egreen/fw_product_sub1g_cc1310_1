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

/*
 *  ====================== CC1310DK_5XD.c =============================================
 *  This file is responsible for setting up the board specific items for the
 *  SRF06EB with the CC1310EM_5XD board.
 */

/*
 *  ====================== Includes ============================================
 */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerCC26XX.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/ioc.h>
#include <driverlib/udma.h>

#include <ti/sysbios/BIOS.h>

#include <Board.h>


#include <ti/drivers/ADC.h>

#include <ti/drivers/adc/ADCCC26XX.h>

/*
 *  ========================== ADCBuf begin =========================================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(ADCBuf_config, ".const:ADCBuf_config")
#pragma DATA_SECTION(adcBufCC26xxHWAttrs, ".const:adcBufCC26xxHWAttrs")
#pragma DATA_SECTION(ADCBufCC26XX_adcChannelLut, ".const:ADCBufCC26XX_adcChannelLut")
#endif

/* Include drivers */
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufCC26XX.h>

/* ADC objects */
//ADCBufCC26XX_Object adcBufCC26xxObjects[CC1310_LAUNCHXL_ADCBufCOUNT];
ADCBufCC26XX_Object adcBufCC26xxObjects[CC1310_5XD_ADCBufCOUNT];
/*
 *  This table converts a virtual adc channel into a dio and internal analogue input signal.
 *  This table is necessary for the functioning of the adc and adcBuf drivers.
 *  Comment out unused entries.
 *  Dio and internal signal pairs are hardwired. Do not remap them in the table.
 *  The mapping of dio and internal signals is package dependent. Make sure you copied the
 *  correct table from an example board file.
 */
const ADCBufCC26XX_AdcChannelLutEntry ADCBufCC26XX_adcChannelLut[] = {
    {PIN_UNASSIGNED, ADC_COMPB_IN_VDDS},
    {PIN_UNASSIGNED, ADC_COMPB_IN_DCOUPL},
    {PIN_UNASSIGNED, ADC_COMPB_IN_VSS},
    {Board_DIO07_ANALOG, ADC_COMPB_IN_AUXIO7},
    {Board_DIO08_ANALOG, ADC_COMPB_IN_AUXIO6},
    {Board_DIO09_ANALOG, ADC_COMPB_IN_AUXIO5},
    {Board_DIO10_ANALOG, ADC_COMPB_IN_AUXIO4},
    {Board_DIO11_ANALOG, ADC_COMPB_IN_AUXIO3},
    {Board_DIO12_ANALOG, ADC_COMPB_IN_AUXIO2},
    {Board_DIO13_ANALOG, ADC_COMPB_IN_AUXIO1},
    {Board_DIO14_ANALOG, ADC_COMPB_IN_AUXIO0},
};

//const ADCBufCC26XX_HWAttrs adcBufCC26xxHWAttrs[CC1310_LAUNCHXL_ADCBufCOUNT] = {
const ADCBufCC26XX_HWAttrs adcBufCC26xxHWAttrs[CC1310_5XD_ADCBufCOUNT] = {
    {
        .intPriority = ~0,
        .swiPriority = 0,
        .adcChannelLut = ADCBufCC26XX_adcChannelLut,
        .gpTimerUnit = Board_GPTIMER0A,
        .gptDMAChannelMask = 1 << UDMA_CHAN_TIMER0_A,
    }
};

//const ADCBuf_Config ADCBuf_config[] = {
//    {&ADCBufCC26XX_fxnTable, &adcBufCC26xxObjects[0], &adcBufCC26xxHWAttrs[0]},
//    {NULL, NULL, NULL},
//};
const ADCBuf_Config ADCBuf_config[CC1310_5XD_ADCBufCOUNT] = {
    {&ADCBufCC26XX_fxnTable, &adcBufCC26xxObjects[CC1310_5XD_ADCBuf0], &adcBufCC26xxHWAttrs[CC1310_5XD_ADCBuf0]}
};

const uint_least8_t ADCBuf_count = CC1310_5XD_ADCBufCOUNT;

/*
 *  ========================== ADCBuf end =========================================
 */

/*
 *  =============================== ADC ===============================
 */

ADCCC26XX_Object adcCC26xxObjects[CC1310_5XD_ADCCOUNT];

const ADCCC26XX_HWAttrs adcCC26xxHWAttrs[CC1310_5XD_ADCCOUNT] = {
    {
        .adcDIO              = IOID_7,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO7,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
#if defined( TEG_REF_V148 )
        .inputScalingEnabled = false,
#else
        .inputScalingEnabled = true,
#endif
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    }
};

const ADC_Config ADC_config[CC1310_5XD_ADCCOUNT] = {
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_5XD_ADC0], &adcCC26xxHWAttrs[CC1310_5XD_ADC0]}
};

const uint_least8_t ADC_count = CC1310_5XD_ADCCOUNT;


/*
 *  ========================= IO driver initialization =========================
 *  From main, PIN_init(BoardGpioInitTable) should be called to setup safe
 *  settings for this board.
 *  When a pin is allocated and then de-allocated, it will revert to the state
 *  configured in this table.
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(BoardGpioInitTable, ".const:BoardGpioInitTable")
#pragma DATA_SECTION(PINCC26XX_hwAttrs, ".const:PINCC26XX_hwAttrs")
#endif

const PIN_Config BoardGpioInitTable[] = {
                                         // MATT REMOVED 20210308
                                         /*
                                        IOID_0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                                        IOID_1 | PIN_INPUT_EN | PIN_NOPULL,
                                          IOID_2 | PIN_GPIO_OUTPUT_EN
                                                  | PIN_GPIO_HIGH | PIN_PUSHPULL
                                                  | PIN_DRVSTR_MAX,
                                          IOID_3 | PIN_GPIO_OUTPUT_EN
                                                  | PIN_GPIO_LOW | PIN_PUSHPULL
                                                  | PIN_DRVSTR_MAX,
                                          IOID_4 | PIN_GPIO_OUTPUT_EN
                                                  | PIN_GPIO_LOW | PIN_PUSHPULL
                                                  | PIN_DRVSTR_MAX,
                                          IOID_5 | PIN_GPIO_OUTPUT_EN
                                                  | PIN_GPIO_LOW | PIN_PUSHPULL
                                                  | PIN_DRVSTR_MAX,
                                          IOID_6 | PIN_GPIO_OUTPUT_EN
                                                  | PIN_GPIO_LOW | PIN_PUSHPULL
                                                  | PIN_DRVSTR_MAX,
                                          IOID_7 | PIN_GPIO_OUTPUT_EN
                                                  | PIN_INPUT_EN | PIN_GPIO_LOW,
                                          IOID_8 | PIN_GPIO_OUTPUT_EN
                                                  | PIN_GPIO_LOW | PIN_PUSHPULL
                                                  | PIN_DRVSTR_MAX,
                                          IOID_9 | PIN_INPUT_EN | PIN_NOPULL,
                                          IOID_10 | PIN_GPIO_OUTPUT_EN
                                                  | PIN_GPIO_LOW | PIN_PUSHPULL
                                                  | PIN_DRVSTR_MAX,
                                          IOID_11 | PIN_INPUT_EN | PIN_NOPULL,
                                          IOID_12 | PIN_GPIO_OUTPUT_EN
                                                  | PIN_GPIO_LOW | PIN_PUSHPULL
                                                  | PIN_DRVSTR_MAX,
                                          IOID_13 | PIN_INPUT_EN | PIN_NOPULL,
                                          IOID_14 | PIN_GPIO_OUTPUT_EN
                                          | PIN_GPIO_LOW | PIN_PUSHPULL
                                          | PIN_DRVSTR_MAX,
                                          PIN_TERMINATE */

                                         // MATT ADDED 20210308
                                         IOID_0  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX, // UART0 TX
                                         IOID_1  | PIN_INPUT_EN       | PIN_NOPULL, // UART0 RX
#if defined( TEG_CM0X )
                                         IOID_2  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX, // Charge
                                         IOID_3  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX, // Mode
#else
                                         IOID_2  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX, // Mode
                                         IOID_3  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
                                         IOID_4  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                                         IOID_5  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                                         IOID_6  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                                         //IOID_7  | PIN_GPIO_OUTPUT_EN | PIN_INPUT_EN  | PIN_GPIO_LOW, // Current
                                         IOID_7  | PIN_INPUT_EN       | PIN_NOPULL, // Current
                                         IOID_8  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
                                         IOID_9  | PIN_INPUT_EN       | PIN_NOPULL, // BackDoor
                                         IOID_10 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX, // RMS
                                         //IOID_10 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH  | PIN_PUSHPULL | PIN_DRVSTR_MAX, // RMS
                                         IOID_11 | PIN_INPUT_EN       | PIN_NOPULL, // Wake
                                         IOID_12 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX, // Done
                                         IOID_13 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#if defined( TEG_CM0X )
                                         IOID_14 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#else
                                         IOID_14 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
                                         PIN_TERMINATE /* Terminate list */
};

const PINCC26XX_HWAttrs PINCC26XX_hwAttrs = { .intPriority = ~0, .swiPriority = 0 };

/*============================================================================*/

/*
 *  ============================= Power begin ===================================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(PowerCC26XX_config, ".const:PowerCC26XX_config")
#endif
const PowerCC26XX_Config PowerCC26XX_config = {
        .policyInitFxn = NULL, .policyFxn = &PowerCC26XX_standbyPolicy,
        .calibrateFxn = &PowerCC26XX_calibrate, .enablePolicy = TRUE,
        .calibrateRCOSC_LF = TRUE, .calibrateRCOSC_HF = TRUE, };
/*
 *  ============================= Power end ===================================
 */

/*
 *  =============================== UART ===============================
 */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

UARTCC26XX_Object uartCC26XXObjects[CC1310DK_5XD_UARTCOUNT];

uint8_t uartCC26XXRingBuffer[CC1310DK_5XD_UARTCOUNT][32];

const UARTCC26XX_HWAttrsV2 uartCC26XXHWAttrs[CC1310DK_5XD_UARTCOUNT] = { {
        .baseAddr = UART0_BASE, .powerMngrId = PowerCC26XX_PERIPH_UART0,
        .intNum = INT_UART0_COMB, .intPriority = ~0, .swiPriority = 0, .txPin =
                Board_UART_TX,
        .rxPin = Board_UART_RX, .ctsPin = PIN_UNASSIGNED, .rtsPin =
                PIN_UNASSIGNED,
        .ringBufPtr = uartCC26XXRingBuffer[CC1310DK_5XD_UART0], .ringBufSize =
                sizeof(uartCC26XXRingBuffer[CC1310DK_5XD_UART0]),
        .txIntFifoThr = UARTCC26XX_FIFO_THRESHOLD_DEFAULT, .rxIntFifoThr =
                UARTCC26XX_FIFO_THRESHOLD_DEFAULT } };

const UART_Config UART_config[CC1310DK_5XD_UARTCOUNT] = { {
        .fxnTablePtr = &UARTCC26XX_fxnTable, .object =
                &uartCC26XXObjects[CC1310DK_5XD_UART0],
        .hwAttrs = &uartCC26XXHWAttrs[CC1310DK_5XD_UART0] }, };

const uint_least8_t UART_count = CC1310DK_5XD_UARTCOUNT;
/*
 *  ============================= UART end =====================================
 */

/*
 *  ============================= UDMA begin ===================================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UDMACC26XX_config, ".const:UDMACC26XX_config")
#pragma DATA_SECTION(udmaHWAttrs, ".const:udmaHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/dma/UDMACC26XX.h>

/* UDMA objects */
UDMACC26XX_Object udmaObjects[CC1310DK_5XD_UDMACOUNT];

/* UDMA configuration structure */
const UDMACC26XX_HWAttrs udmaHWAttrs[CC1310DK_5XD_UDMACOUNT] = { {
        .baseAddr = UDMA0_BASE, .powerMngrId = PowerCC26XX_PERIPH_UDMA,
        .intNum = INT_DMA_ERR, .intPriority = ~0 } };

/* UDMA configuration structure */
const UDMACC26XX_Config UDMACC26XX_config[] = { { .object = &udmaObjects[0],
                                                  .hwAttrs = &udmaHWAttrs[0] },
                                                { NULL, NULL } };
/*
 *  ============================= UDMA end =====================================
 */

/*
 *  ========================== SPI DMA begin ===================================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(SPI_config, ".const:SPI_config")
#pragma DATA_SECTION(spiCC26XXDMAHWAttrs, ".const:spiCC26XXDMAHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/spi/SPICC26XXDMA.h>

/* SPI objects */
SPICC26XXDMA_Object spiCC26XXDMAObjects[CC1310DK_5XD_SPICOUNT];

/* SPI configuration structure, describing which pins are to be used */
const SPICC26XXDMA_HWAttrsV1 spiCC26XXDMAHWAttrs[CC1310DK_5XD_SPICOUNT] = { {
        .baseAddr = SSI0_BASE, .intNum = INT_SSI0_COMB, .intPriority = ~0,
        .swiPriority = 0, .powerMngrId = PowerCC26XX_PERIPH_SSI0,
        .defaultTxBufValue = 0, .rxChannelBitMask = 1 << UDMA_CHAN_SSI0_RX,
        .txChannelBitMask = 1 << UDMA_CHAN_SSI0_TX, .mosiPin = Board_SPI0_MOSI,
        .misoPin = Board_SPI0_MISO, .clkPin = Board_SPI0_CLK, .csnPin =
                Board_SPI0_CSN } };

/* SPI configuration structure */
const SPI_Config SPI_config[] = { { .fxnTablePtr = &SPICC26XXDMA_fxnTable,
                                    .object = &spiCC26XXDMAObjects[0],
                                    .hwAttrs = &spiCC26XXDMAHWAttrs[0] },
                                  { NULL, NULL, NULL } };
/*
 *  ========================== SPI DMA end =====================================
 */

/*
 *  ========================== Crypto begin ====================================
 *  NOTE: The Crypto implementation should be considered experimental
 *        and not validated!
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(CryptoCC26XX_config, ".const:CryptoCC26XX_config")
#pragma DATA_SECTION(cryptoCC26XXHWAttrs, ".const:cryptoCC26XXHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/crypto/CryptoCC26XX.h>

/* Crypto objects */
CryptoCC26XX_Object cryptoCC26XXObjects[CC1310DK_5XD_CRYPTOCOUNT];

/* Crypto configuration structure, describing which pins are to be used */
const CryptoCC26XX_HWAttrs cryptoCC26XXHWAttrs[CC1310DK_5XD_CRYPTOCOUNT] = { {
        .baseAddr = CRYPTO_BASE, .powerMngrId = PowerCC26XX_PERIPH_CRYPTO,
        .intNum = INT_CRYPTO_RESULT_AVAIL_IRQ, .intPriority = ~0, } };

/* Crypto configuration structure */
const CryptoCC26XX_Config CryptoCC26XX_config[] = { {
        .object = &cryptoCC26XXObjects[0], .hwAttrs = &cryptoCC26XXHWAttrs[0] },
                                                    { NULL, NULL } };
/*
 *  ========================== Crypto end =========================================
 */

/*
 *  ========================= RF driver begin ==============================================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(RFCC26XX_hwAttrs, ".const:RFCC26XX_hwAttrs")
#endif
/* Include drivers */
#include <ti/drivers/rf/RF.h>

/* RF hwi and swi priority */
const RFCC26XX_HWAttrs RFCC26XX_hwAttrs = { .hwiCpe0Priority = ~0,
                                            .hwiHwPriority = ~0,
                                            .swiCpe0Priority = 0,
                                            .swiHwPriority = 0, };

/*
 *  ========================== RF driver end =========================================
 */

/*
 *  ========================= Display begin ====================================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(Display_config, ".const:Display_config")
#pragma DATA_SECTION(displayUartHWAttrs, ".const:displayUartHWAttrs")
#endif

//#include <ti/mw/display/Display.h>
//#include <ti/mw/display/DisplayUart.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>

/* Structures for UartPlain Blocking */
DisplayUart_Object displayUartObject;

#ifndef BOARD_DISPLAY_UART_STRBUF_SIZE
#define BOARD_DISPLAY_UART_STRBUF_SIZE    128
#endif
static char uartStringBuf[BOARD_DISPLAY_UART_STRBUF_SIZE];

const DisplayUart_HWAttrs displayUartHWAttrs = {
        .uartIdx = Board_UART, .baudRate = 115200, .mutexTimeout =
                BIOS_WAIT_FOREVER,
        .strBuf = uartStringBuf, .strBufLen = BOARD_DISPLAY_UART_STRBUF_SIZE, };

/* Array of displays */
const Display_Config Display_config[] = {
#if !defined(BOARD_DISPLAY_EXCLUDE_UART)
        { .fxnTablePtr = &DisplayUart_fxnTable, .object = &displayUartObject,
          .hwAttrs = &displayUartHWAttrs, },
#endif
        { NULL, NULL, NULL } // Terminator
};

/*
 *  ========================= Display end ======================================
 */

/*
 *  ============================ GPTimer begin =================================
 *  Remove unused entries to reduce flash usage both in Board.c and Board.h
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(GPTimerCC26XX_config, ".const:GPTimerCC26XX_config")
#pragma DATA_SECTION(gptimerCC26xxHWAttrs, ".const:gptimerCC26xxHWAttrs")
#endif

/* GPTimer hardware attributes, one per timer part (Timer 0A, 0B, 1A, 1B..) */
const GPTimerCC26XX_HWAttrs gptimerCC26xxHWAttrs[CC1310DK_5XD_GPTIMERPARTSCOUNT] =
        { { .baseAddr = GPT0_BASE, .intNum = INT_GPT0A, .intPriority = (~0),
            .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0A, },
          { .baseAddr = GPT0_BASE, .intNum = INT_GPT0B, .intPriority = (~0),
            .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0B, },
          { .baseAddr = GPT1_BASE, .intNum = INT_GPT1A, .intPriority = (~0),
            .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1A, },
          { .baseAddr = GPT1_BASE, .intNum = INT_GPT1B, .intPriority = (~0),
            .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1B, },
          { .baseAddr = GPT2_BASE, .intNum = INT_GPT2A, .intPriority = (~0),
            .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2A, },
          { .baseAddr = GPT2_BASE, .intNum = INT_GPT2B, .intPriority = (~0),
            .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2B, },
          { .baseAddr = GPT3_BASE, .intNum = INT_GPT3A, .intPriority = (~0),
            .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3A, },
          { .baseAddr = GPT3_BASE, .intNum = INT_GPT3B, .intPriority = (~0),
            .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3B, }, };

/*  GPTimer objects, one per full-width timer (A+B) (Timer 0, Timer 1..) */
GPTimerCC26XX_Object gptimerCC26XXObjects[CC1310DK_5XD_GPTIMERCOUNT];

/* GPTimer configuration (used as GPTimer_Handle by driver and application) */
const GPTimerCC26XX_Config GPTimerCC26XX_config[CC1310DK_5XD_GPTIMERPARTSCOUNT] =
        { { &gptimerCC26XXObjects[0], &gptimerCC26xxHWAttrs[0], GPT_A }, {
                &gptimerCC26XXObjects[0], &gptimerCC26xxHWAttrs[1], GPT_B },
          { &gptimerCC26XXObjects[1], &gptimerCC26xxHWAttrs[2], GPT_A }, {
                  &gptimerCC26XXObjects[1], &gptimerCC26xxHWAttrs[3], GPT_B },
          { &gptimerCC26XXObjects[2], &gptimerCC26xxHWAttrs[4], GPT_A }, {
                  &gptimerCC26XXObjects[2], &gptimerCC26xxHWAttrs[5], GPT_B },
          { &gptimerCC26XXObjects[3], &gptimerCC26xxHWAttrs[6], GPT_A }, {
                  &gptimerCC26XXObjects[3], &gptimerCC26xxHWAttrs[7], GPT_B }, };

/*
 *  ============================ GPTimer end ===================================
 */

/*
 *  =============================== Watchdog ===============================
 */
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogCC26XX.h>

WatchdogCC26XX_Object watchdogCC26XXObjects[CC1310_5XD_WATCHDOGCOUNT];

const WatchdogCC26XX_HWAttrs watchdogCC26XXHWAttrs[CC1310_5XD_WATCHDOGCOUNT] = {
        { .baseAddr = WDT_BASE, .reloadValue = 45000 /* Reload value in milliseconds */
        }, };

const Watchdog_Config Watchdog_config[CC1310_5XD_WATCHDOGCOUNT] = { {
        .fxnTablePtr = &WatchdogCC26XX_fxnTable, .object =
                &watchdogCC26XXObjects[CC1310_5XD_WATCHDOG0],
        .hwAttrs = &watchdogCC26XXHWAttrs[CC1310_5XD_WATCHDOG0] }, };

const uint_least8_t Watchdog_count = CC1310_5XD_WATCHDOGCOUNT;
