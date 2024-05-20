#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/display/Display.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/ADCBuf.h>

/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "RF_receive.h"
#include "watchdog.h"
#include "Uart2.h"
//#include "Uart.h"
/*
 *  ======== main ========
 */
int main(void)
{
    Board_initGeneral();
    Board_initADCBuf();
    echoTask_init();
    //Uart_init();
    WatchdogFn_init();
    Uart2_createTask();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
