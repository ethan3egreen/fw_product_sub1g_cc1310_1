#include <stdlib.h>
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/PIN.h>
#include "Board.h"
#include "smartrf_settings/smartrf_settings.h"
#include "Uart.h"
#include <unistd.h>
#include <xdc/std.h>
#include <ti/sysbios/knl/Event.h>
#include "RF_receive.h"
#include "watchdog.h"

#define RFEASYLINKECHO_TASK_STACK_SIZE    2048
#define RFEASYLINKECHO_TASK_PRIORITY      3

extern PIN_Handle pinHandle;
static uint8_t echoTaskStack[RFEASYLINKECHO_TASK_STACK_SIZE];
UART_Handle uart;
Event_Struct radioOperationUartEvent;
static Task_Params concentratorRadioTaskParams;
Task_Struct concentratorRadioUartTask; /* not static so you can see in ROV */
static Semaphore_Handle echoDoneSem;
bool isTxDone = true;

/* Pin driver handles */
static PIN_Handle buttonPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State buttonPinState;

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] =
{
    IOID_14  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

UART_Handle uartHandle;
uint8_t uartReadBuffer[10];
volatile uint8_t goToStandby = 0;

Semaphore_Struct semStruct;
Semaphore_Handle semHandle;

void startUART();

void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
    if (pinId == IOID_14)
    {
        /* Release the RX pin */
        PIN_remove(handle, pinId);

        /* Start the UART for TX/RX operation */
        startUART();
    }
}

/*
 *  ======== uartReadCallback ========
 */
void uartReadCallback (UART_Handle handle, void *buf, size_t count)
{
    /* Mark that we want to close the UART after the write completes */
    goToStandby = 1;

    /* Wake the mainTask */
    Semaphore_post(semHandle);
}

/*
 *  ======== startUART ========
 */
void startUART()
{
    UART_Params uartParams;

    /* Call driver init functions */
    UART_init();

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = uartReadCallback;

    uartHandle = UART_open(Board_UART0, &uartParams);

    /* Start a read */
    UART_read(uartHandle, uartReadBuffer, 1);
}

typedef enum
{
    rpcSteSOF,
    rpcSteLen,
    rpcSteData,
    rpcSteFcs
} rpcSte_t;

static void UartFnx(UArg arg0, UArg arg1)
{
    Semaphore_Params params;
    Error_Block      eb;

    Semaphore_Params_init(&params);
    Error_init(&eb);

    echoDoneSem = Semaphore_create(0, &params, &eb);
    if(echoDoneSem == NULL)
    {
        System_abort("Semaphore creation failed");
    }

    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if(!buttonPinHandle)
    {
        /* Error initializing button pins */
        while(1);
    }

    /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0)
    {
        /* Error registering button callback function */
        while(1);
    }

    /* Loop forever */
    while(1)
    {
        /* Wait until it is time to wake-up */
        Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);

        /* If we are to go into standby, close the UART and reconfigure the pin */
        if(goToStandby)
        {
            /* Go back to sleep */
            UART_close(uartHandle);

            /* Reconfigure the PIN interrupt */
            /* Add RX pin back to the PIN driver */
            PIN_add(buttonPinHandle, buttonPinTable[0]);

            /* Register callback */
            PIN_registerIntCb(buttonPinHandle, buttonCallbackFxn);

            goToStandby = 0;
        }
    }
}

int inttochar(int input)
{
    if(input >= 0 && input <= 9)
    {
        return input + '0';
    }
    else if(input >= 10 && input <= 15)
    {
        return input - 10 + 'A';
    }
    else
    {
        return 0;
    }
}

void hextochar(int input,char *result)
{
     char output[3];
     char temp;
     int count;
     count = 1;

     output[0] = '0';
     output[1] = '0';
     output[2] = '\0';

     do
     {
        temp = input & 0xF;
        output[count] = inttochar(temp);
        input = input >> 4;
        count--;
     } while (input != 0 && count < 1);

     output[2] = '\0';
     memcpy(result,&output,sizeof(output));
}

void send_Uart(uint8_t *buffer, size_t size)
{
    buffer[buffer[1]+1] = 0xE3;
    UART_write(uart, buffer, size);
}

void Uart_init()
{
    Task_Params_init(&concentratorRadioTaskParams);
    concentratorRadioTaskParams.stackSize = RFEASYLINKECHO_TASK_STACK_SIZE;
    concentratorRadioTaskParams.priority = RFEASYLINKECHO_TASK_PRIORITY;
    concentratorRadioTaskParams.stack = &echoTaskStack;
    concentratorRadioTaskParams.arg0 = (UInt)1000000;
    Task_construct(&concentratorRadioUartTask, UartFnx, &concentratorRadioTaskParams, NULL);
}
