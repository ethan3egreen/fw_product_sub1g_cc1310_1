#include "scif.h"
#include <driverlib/prcm.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/PIN/PINCC26XX.h>
#include "Board.h"
#include "Uart2.h"
#include "Uart.h"
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <unistd.h>
#include "RF_receive.h"
#include <driverlib/sys_ctrl.h>
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <string.h>

#define BV(n)                   (1 << (n))

//uint8_t FW_VERSION = 0x14; // 20220812 by Ethan
// michael
//#if 0
//uint8_t FW_VERSION = 0x15; // 20220824 by Ethan
//#else
//uint8_t FW_VERSION = 0x16;
//uint8_t FW_VERSION = 0x51;
//#endif

#define TASK_STACK_SIZE 1024

static uint8_t taskStack[TASK_STACK_SIZE];
Task_Struct taskStruct; /* not static so you can see in ROV */
static Semaphore_Struct sem;
static Semaphore_Handle echoDoneSem;
bool isDumpStr = false;
bool isDump = false;

extern char TEG_Device_MAC_ASC[16];
extern bool isAllow;
extern uint8_t pid;
extern uint8_t ieee[8];

// MATT MODIFIED 20221117
extern uint8_t offset_percent;
extern uint8_t offset_value;
extern uint8_t offset_sign;
extern float currentPercent;
extern float currentOffset;
extern float currentPercent00;
extern float currentOffset0;
extern float currentPercent10;
extern float currentOffset1;
extern float freqCorrection;
extern int16_t currentMin;
extern int16_t currentMax;

extern uint16_t frq;
extern uint8_t sampleTime;
extern uint8_t tx_power;
extern uint8_t sampleV_Len;
extern uint8_t hw_version;
extern uint8_t isDemo;
extern char  TEG_Device_MAC_ASC[16];
extern uint32_t CurrentValue;
extern uint8_t Range;

/* Pin driver handles */
static Semaphore_Handle semHandle;

static PIN_Config keyPinsCfg[] =
{
    IOID_1    | PIN_INPUT_EN | PIN_PULLUP,
    PIN_TERMINATE
};

static PIN_State  keyPins;
static PIN_Handle hKeyPins;

static void sleepms(float ms)
{
    Semaphore_pend(echoDoneSem, (ms*1000 / Clock_tickPeriod));
}

static void startUART2()
{
    // Initialize and start the Sensor Controller
    scifInit(&scifDriverSetup);

    // Start the UART emulator
    scifExecuteTasksOnceNbl(BV(SCIF_UART_EMULATOR_TASK_ID));

    // Enable baud rate generation
    scifUartSetBaudRate(9600);

    // Enable RX
    scifUartSetRxTimeout(20);
    scifUartSetRxEnableReqIdleCount(1);
    scifUartRxEnable(1);

    // Enable events
    scifUartSetEventMask(0xF);
}

static void buttonCallbackFxn2(PIN_Handle handle, PIN_Id pinId)
{
    if (pinId == IOID_1)
    {
        /* Release the RX pin */
        PIN_remove(handle, pinId);

        /* Start the UART for TX/RX operation */
        Semaphore_post(semHandle);
    }
}

static void taskFxn(UArg a0, UArg a1);

void Uart2_createTask()
{
    //Initialize task
    Task_Params params;
    Task_Params_init(&params);
    params.priority = 5;
    params.stackSize = TASK_STACK_SIZE;
    params.stack = taskStack;
    params.arg0 = (UInt)1000000;
    Task_construct(&taskStruct, taskFxn, &params, NULL);
    
    // Construct semaphore used for pending in task
    Semaphore_Params sParams;
    Error_Block      eb;

    Semaphore_Params_init(&sParams);
    
    echoDoneSem = Semaphore_create(0, &sParams, &eb);
    if(echoDoneSem == NULL)
    {
        System_abort("Semaphore creation failed");
    }

    Semaphore_construct(&sem, 0, &sParams);
    semHandle = Semaphore_handle(&sem);
}

void taskFxn(UArg a0, UArg a1)
{
    Semaphore_Params Sparams;
    Error_Block      eb;

    /* Init params */
    Semaphore_Params_init(&Sparams);
    Error_init(&eb);

    /* Create semaphore instance */
    echoDoneSem = Semaphore_create(0, &Sparams, &eb);
    if(echoDoneSem == NULL)
    {
        System_abort("Semaphore creation failed");
    }

    // Enable power domains
    PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
    while (PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON);

    // Enable peripheral clocks
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
    PRCMLoadSet();
    while (!PRCMLoadGet());

    sleepms(2000);

    hKeyPins = PIN_open(&keyPins, keyPinsCfg);
    PIN_registerIntCb(hKeyPins, buttonCallbackFxn2);
    PIN_setConfig(hKeyPins, PIN_BM_IRQ, IOID_1  | PIN_IRQ_NEGEDGE);

    Semaphore_pend(semHandle, BIOS_WAIT_FOREVER);

    sleepms(200);

    startUART2();

    uint8_t head = 0;
    uint8_t len = 0;
    uint8_t _len = 0;
    uint8_t rx_buf[10] = {0};
    uint16_t count = 0;

    //uint8_t txping[] = {0x3E,0x02,PING_ACK,0xE3};
    //scifUartTxPutChars((char*)txping, 4);
    uint8_t txHi[12] = {0x3E,0x0A,GET_MAC,0,0,0,0,0,0,0,0,0xE3},i;
    for(i = 0;i < 8;i++){
        txHi[3+i] = ieee[i];
    }
    //txHi[11] = 0xE3;
    scifUartTxPutChars((char*)txHi,12);

    // Main loop
    while(1){
        len = 0;
        do{
            sleepms(100);
            len = scifUartGetRxFifoCount();
            if(count++ > 500){
                if(!isDump && !isDumpStr){
                    SysCtrlSystemReset();
                }
            }
        }while(len < 4);
        count = 0;

        //_len = 0;
        //count = 0;
        //do{
        //    sleepms(20);
        //    len = scifUartGetRxFifoCount();
        //    if(len > _len){
        //        _len = len;
        //        count = 0;
        //    }
        //    if(count++ > 25000){
        //        if(!isDump && !isDumpStr){
        //            SysCtrlSystemReset();
        //        }
        //    }
        //}while(count < 10 || len == 0);

        /*head = (char)scifUartRxGetChar();
        if(head == 0x3e){
            _len = (char)scifUartRxGetChar();
            if(_len < 9){
                scifUartRxGetChars((char*)rx_buf[2], _len);
            }else{
                do{
                    len = scifUartGetRxFifoCount();
                    if(len > 0){
                        if(len > 10)
                            len = 10;
                        scifUartRxGetChars((char*)rx_buf, len);
                    }else{
                        _len = 1;
                        rx_buf[0] = 0;
                    }
                }while(len);
            }
            if(rx_buf[_len+1] == 0xe3)*/
        if(len > 10){
            len = 10;
        }
        scifUartRxGetChars((char*)rx_buf, len);
        if(rx_buf[0] == 0x3e){
            if(rx_buf[len-1] == 0xe3)
            {
                switch(rx_buf[2])
                {
                    case PING_ACK:{
                        uint8_t ack[] = {0x3E,0x02,PING_ACK,0xE3};
                        //ack[0] = 0x3E;
                        //ack[1] = 1+1;
                        //ack[2] = PING_ACK;
                        //ack[3] = 0xE3;
                        scifUartTxPutChars((char*)ack, 4);
                        break;
                    }
                    case GET_MAC:
                    {
                        uint8_t resp[12] = {0},i;
                        resp[0] = 0x3E;
                        resp[1] = 8+1+1;
                        resp[2] = GET_MAC;
                        for(i = 0;i < 8;i++)
                        {
                            resp[3+i] = ieee[i];
                        }
                        resp[11] = 0xe3;
                        scifUartTxPutChars((char*)resp,12);
                        break;
                    }
                    case GET_FIRMWARE_VERSION:
                    {
                        uint8_t tx[6] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+2;
                        tx[2] = GET_FIRMWARE_VERSION;
                        tx[3] = FW_VERSION;
                        tx[4] = hw_version;
                        tx[5] = 0xE3;
                        scifUartTxPutChars((char*)tx,6);
                        break;
                    }
                    case SET_FRQ:
                    {
                        uint16_t f = BUILD_UINT16(rx_buf[3],rx_buf[4]);
                        if(f != 0 && f < 1000)
                        {
                           if(f != frq)
                           {
                               uint8_t data_backup[100] = {0};
                                OADTarget_readFlash(20,0,data_backup,100);
                                OADTarget_eraseFlash(20);
                                data_backup[FRQ_L] = rx_buf[3];//frq L
                                data_backup[FRQ_H] = rx_buf[4];//frq H
                                OADTarget_writeFlash(20,0,data_backup,100);
                                frq = f;

                                uint8_t tx[6] = {0};
                                tx[0] = 0x3E;
                                tx[1] = 1+1+2;
                                tx[2] = SET_FRQ;
                                tx[3] = rx_buf[3];
                                tx[4] = rx_buf[4];
                                tx[5] = 0xE3;
                                scifUartTxPutChars((char*)tx,6);
                           }
                           else
                           {
                               uint8_t tx[6] = {0};
                               tx[0] = 0x3E;
                               tx[1] = 1+1+2;
                               tx[2] = SET_FRQ;
                               tx[3] = rx_buf[3];
                               tx[4] = rx_buf[4];
                               tx[5] = 0xE3;
                               scifUartTxPutChars((char*)tx,6);
                           }
                        }
                        else
                        {
                            uint8_t tx[6] = {0};
                            tx[0] = 0x3E;
                            tx[1] = 1+1+2;
                            tx[2] = SET_FRQ;
                            tx[3] = 0xfe;
                            tx[4] = 0xfe;
                            tx[5] = 0xE3;
                            scifUartTxPutChars((char*)tx,6);
                        }
                        break;
                    }
                    case GET_FRQ:
                    {
                        uint8_t tx[6] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+2;
                        tx[2] = GET_FRQ;
                        tx[3] = LO_UINT16(frq);
                        tx[4] = HI_UINT16(frq);
                        tx[5] = 0xE3;
                        scifUartTxPutChars((char*)tx,6);
                        break;
                    }
                    case SET_PID:
                    {
                        if(pid != rx_buf[3])
                        {
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            data_backup[PID] = rx_buf[3];//pid
                            OADTarget_writeFlash(20,0,data_backup,100);
                            pid = rx_buf[3];
                        }
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = SET_PID;
                        tx[3] = rx_buf[3];
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case GET_PID:
                    {
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = GET_PID;
                        tx[3] = pid;
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case SET_BORADCAST_INTERVAL:
                    {
                        if(sampleTime != rx_buf[3])
                        {
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            data_backup[BROADCAST_INTERVAL] = rx_buf[3];//pid
                            OADTarget_writeFlash(20,0,data_backup,100);
                            sampleTime = rx_buf[3];
                        }
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = SET_BORADCAST_INTERVAL;
                        tx[3] = rx_buf[3];
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case GET_BORADCAST_INTERVAL:
                    {
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = GET_BORADCAST_INTERVAL;
                        tx[3] = sampleTime;
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case SET_TX_POWER:
                    {
                        if(tx_power != rx_buf[3])
                        {
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            data_backup[TX_POWER] = rx_buf[3];
                            OADTarget_writeFlash(20,0,data_backup,100);
                            tx_power = rx_buf[3];
                        }
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = SET_TX_POWER;
                        tx[3] = rx_buf[3];
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case GET_TX_POWER:
                    {
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = GET_TX_POWER;
                        tx[3] = tx_power;
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case SET_HZ:
                    {
                        if(sampleV_Len != rx_buf[3])
                        {
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            data_backup[HZ] = rx_buf[3];
                            OADTarget_writeFlash(20,0,data_backup,100);
                            sampleV_Len = rx_buf[3];
                        }
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = SET_HZ;
                        tx[3] = rx_buf[3];
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case GET_HZ:
                    {
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = GET_HZ;
                        tx[3] = sampleV_Len;
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case CLEAR_RESET:
                    {
                        OADTarget_eraseFlash(21);
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = CLEAR_RESET;
                        tx[3] = 1;
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case RESET_CMD:
                    {
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = RESET_CMD;
                        tx[3] = 0xbb;
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        Semaphore_pend(echoDoneSem, (200000 / Clock_tickPeriod));
                        SysCtrlSystemReset();
                        break;
                    }

                    // MATT MODIFIED 20221117
                    case SET_OFFSET_PERCENT:
                    {
                        if(offset_percent != rx_buf[3])
                        {
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            data_backup[OFFSET_PERCENT] = rx_buf[3];
                            OADTarget_writeFlash(20,0,data_backup,100);
                            offset_percent = rx_buf[3];
                        }
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = SET_OFFSET_PERCENT;
                        tx[3] = rx_buf[3];
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case GET_OFFSET_PERCENT:
                    {
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = GET_OFFSET_PERCENT;
                        tx[3] = offset_percent;
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case SET_OFFSET_RAW:
                    {
                        uint8_t data_backup[100] = {0};
                        OADTarget_readFlash(20,0,data_backup,100);
                        OADTarget_eraseFlash(20);
                        data_backup[OFFSET_RAW_SIGN] = rx_buf[3];
                        data_backup[OFFSET_RAW_VALUE] = rx_buf[4];
                        OADTarget_writeFlash(20,0,data_backup,100);
                        offset_sign = rx_buf[3];
                        offset_value = rx_buf[4];

                        uint8_t tx[6] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+2;
                        tx[2] = SET_OFFSET_RAW;
                        tx[3] = rx_buf[3];
                        tx[4] = rx_buf[4];
                        tx[5] = 0xE3;
                        scifUartTxPutChars((char*)tx,6);
                        break;
                    }
                    case GET_OFFSET_RAW:
                    {
                        uint8_t tx[6] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+2;
                        tx[2] = GET_OFFSET_RAW;
                        tx[3] = offset_sign;
                        tx[4] = offset_value;
                        tx[5] = 0xE3;
                        scifUartTxPutChars((char*)tx,6);
                        break;
                    }
                    case SET_CURRENT_PERCENT:
                    {
                        if(currentPercent != *((float *)(&rx_buf[3])))
                        {
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            memcpy(&data_backup[CURRENT_PERCENT_B0],&rx_buf[3],sizeof(float));
                            OADTarget_writeFlash(20,0,data_backup,100);
                            currentPercent = *((float *)(&rx_buf[3]));
                        }
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = SET_CURRENT_PERCENT;
                        memcpy(&tx[3],&rx_buf[3],sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }
                    case GET_CURRENT_PERCENT:
                    {
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = GET_CURRENT_PERCENT;
                        memcpy(&tx[3],(uint8_t *)(&currentPercent),sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }
                    case SET_CURRENT_OFFSET:
                    {
                        if(currentOffset != *((float *)(&rx_buf[3])))
                        {
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            memcpy(&data_backup[CURRENT_OFFSET_B0],&rx_buf[3],sizeof(float));
                            OADTarget_writeFlash(20,0,data_backup,100);
                            currentOffset = *((float *)(&rx_buf[3]));
                        }
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = SET_CURRENT_OFFSET;
                        memcpy(&tx[3],&rx_buf[3],sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }
                    case GET_CURRENT_OFFSET:
                    {
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = GET_CURRENT_OFFSET;
                        memcpy(&tx[3],(uint8_t *)(&currentOffset),sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }

                    case SET_CURRENT_PERCENT00: {
                        if(currentPercent00 != *((float *)(&rx_buf[3]))){
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            memcpy(&data_backup[CURRENT_PERCENT00_B0],&rx_buf[3],sizeof(float));
                            OADTarget_writeFlash(20,0,data_backup,100);
                            currentPercent00 = *((float *)(&rx_buf[3]));
                        }
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = SET_CURRENT_PERCENT00;
                        memcpy(&tx[3],&rx_buf[3],sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }

                    case GET_CURRENT_PERCENT00: {
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = GET_CURRENT_PERCENT00;
                        memcpy(&tx[3],(uint8_t *)(&currentPercent00),sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }

                    case SET_CURRENT_OFFSET0: {
                        if(currentOffset0 != *((float *)(&rx_buf[3]))){
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            memcpy(&data_backup[CURRENT_OFFSET0_B0],&rx_buf[3],sizeof(float));
                            OADTarget_writeFlash(20,0,data_backup,100);
                            currentOffset0 = *((float *)(&rx_buf[3]));
                        }
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = SET_CURRENT_OFFSET0;
                        memcpy(&tx[3],&rx_buf[3],sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }

                    case GET_CURRENT_OFFSET0: {
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = GET_CURRENT_OFFSET0;
                        memcpy(&tx[3],(uint8_t *)(&currentOffset0),sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }

                    case SET_CURRENT_PERCENT10: {
                        if(currentPercent10 != *((float *)(&rx_buf[3]))){
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            memcpy(&data_backup[CURRENT_PERCENT10_B0],&rx_buf[3],sizeof(float));
                            OADTarget_writeFlash(20,0,data_backup,100);
                            currentPercent10 = *((float *)(&rx_buf[3]));
                        }
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = SET_CURRENT_PERCENT10;
                        memcpy(&tx[3],&rx_buf[3],sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }

                    case GET_CURRENT_PERCENT10: {
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = GET_CURRENT_PERCENT10;
                        memcpy(&tx[3],(uint8_t *)(&currentPercent10),sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }

                    case SET_CURRENT_OFFSET1: {
                        if(currentOffset1 != *((float *)(&rx_buf[3]))){
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            memcpy(&data_backup[CURRENT_OFFSET1_B0],&rx_buf[3],sizeof(float));
                            OADTarget_writeFlash(20,0,data_backup,100);
                            currentOffset1 = *((float *)(&rx_buf[3]));
                        }
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = SET_CURRENT_OFFSET1;
                        memcpy(&tx[3],&rx_buf[3],sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }

                    case GET_CURRENT_OFFSET1: {
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = GET_CURRENT_OFFSET1;
                        memcpy(&tx[3],(uint8_t *)(&currentOffset1),sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }

                    case SET_FREQ_CORRECTION: {
                        if(freqCorrection != *((float *)(&rx_buf[3]))){
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            memcpy(&data_backup[FREQ_CORRECTION_B0],&rx_buf[3],sizeof(float));
                            OADTarget_writeFlash(20,0,data_backup,100);
                            freqCorrection = *((float *)(&rx_buf[3]));
                        }
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = SET_FREQ_CORRECTION;
                        memcpy(&tx[3],&rx_buf[3],sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }

                    case GET_FREQ_CORRECTION: {
                        uint8_t tx[8] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+4+1;
                        tx[2] = GET_FREQ_CORRECTION;
                        memcpy(&tx[3],(uint8_t *)(&freqCorrection),sizeof(float));
                        tx[7] = 0xE3;
                        scifUartTxPutChars((char*)tx,8);
                        break;
                    }

                    case SET_CURRENT_MIN:
                    {
                        if(currentMin != *((int16_t *)(&rx_buf[3])));
                        {
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            memcpy(&data_backup[CURRENT_MIN_B0],&rx_buf[3],sizeof(int16_t));
                            OADTarget_writeFlash(20,0,data_backup,100);
                            currentMin = *((int16_t *)(&rx_buf[3]));
                        }
                        uint8_t tx[6] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+2+1;
                        tx[2] = SET_CURRENT_MIN;
                        memcpy(&tx[3],&rx_buf[3],sizeof(int16_t));
                        tx[5] = 0xE3;
                        scifUartTxPutChars((char*)tx,6);
                        break;
                    }
                    case GET_CURRENT_MIN:
                    {
                        uint8_t tx[6] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+2+1;
                        tx[2] = GET_CURRENT_MIN;
                        memcpy(&tx[3],(uint8_t *)(&currentMin),sizeof(int16_t));
                        tx[5] = 0xE3;
                        scifUartTxPutChars((char*)tx,6);
                        break;
                    }
                    case SET_CURRENT_MAX:
                    {
                        if(currentMax != *((int16_t *)(&rx_buf[3])));
                        {
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            memcpy(&data_backup[CURRENT_MAX_B0],&rx_buf[3],sizeof(int16_t));
                            OADTarget_writeFlash(20,0,data_backup,100);
                            currentMax = *((int16_t *)(&rx_buf[3]));
                        }
                        uint8_t tx[6] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+2+1;
                        tx[2] = SET_CURRENT_MAX;
                        memcpy(&tx[3],&rx_buf[3],sizeof(int16_t));
                        tx[5] = 0xE3;
                        scifUartTxPutChars((char*)tx,6);
                        break;
                    }
                    case GET_CURRENT_MAX:
                    {
                        uint8_t tx[6] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+2+1;
                        tx[2] = GET_CURRENT_MAX;
                        memcpy(&tx[3],(uint8_t *)(&currentMax),sizeof(int16_t));
                        tx[5] = 0xE3;
                        scifUartTxPutChars((char*)tx,6);
                        break;
                    }

                    case SET_HARDWARE_ID:
                    {
                        if(hw_version != rx_buf[3])
                        {
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            data_backup[HARDWARE_ID] = rx_buf[3];
                            OADTarget_writeFlash(20,0,data_backup,100);
                            hw_version = rx_buf[3];
                        }
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = SET_HARDWARE_ID;
                        tx[3] = rx_buf[3];
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case GET_HARDWARE_ID:
                    {
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = GET_HARDWARE_ID;
                        tx[3] = hw_version;
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case SET_ENABLE_MAC:
                    {
                        uint8_t tx[5] = {0},i;
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = SET_ENABLE_MAC;
                        tx[3] = rx_buf[3];
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        if(rx_buf[3] != isAllow)
                        {
                            OADTarget_eraseFlash(22);
                            if(rx_buf[3] == 1)
                            {
                                uint8_t device_mac[16] = {0};
                                for(i = 0;i < 16;i++)
                                {
                                    device_mac[i] = TEG_Device_MAC_ASC[i] + MAC_DECODE;
                                }
                                OADTarget_writeFlash(22,0,device_mac,16);
                            }
                        }
                        break;
                    }
                    case GET_ENABLE_MAC:
                    {
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = GET_ENABLE_MAC;
                        tx[3] = isAllow;
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case SET_DEMO:
                    {
                        if(isDemo != rx_buf[3])
                        {
                            uint8_t data_backup[100] = {0};
                            OADTarget_readFlash(20,0,data_backup,100);
                            OADTarget_eraseFlash(20);
                            data_backup[DEMO] = rx_buf[3];
                            OADTarget_writeFlash(20,0,data_backup,100);
                            isDemo = rx_buf[3];
                        }
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = SET_DEMO;
                        tx[3] = rx_buf[3];
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case GET_DEMO:
                    {
                        uint8_t tx[5] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+1;
                        tx[2] = GET_DEMO;
                        tx[3] = isDemo;
                        tx[4] = 0xE3;
                        scifUartTxPutChars((char*)tx,5);
                        break;
                    }
                    case GET_CURRENTSTR:{
                        char str[] = "";
                        System_sprintf(str,"%u\t(%d)\r\n", (long)CurrentValue, Range);
                        scifUartTxPutChars((char*)str,strlen(str));
                        break;
                    }
                    case CURRENT_DUMPSTR:{
                        if(rx_buf[3] == 0){
                            isDumpStr = false;
                        }else{
                            isDumpStr = true;
                        }
                        break;
                    }
                    case GET_CURRENT:{
                        uint8_t tx[7] = {0};
                        tx[0] = 0x3E;
                        tx[1] = 1+1+3;
                        tx[2] = GET_CURRENT;
                        tx[3] = ((uint8_t *) &CurrentValue)[1];
                        tx[4] = ((uint8_t *) &CurrentValue)[0];
                        tx[5] = Range;
                        tx[6] = 0xE3;
                        scifUartTxPutChars((char*)tx,7);
                        break;
                    }
                    case CURRENT_DUMP:{
                        if(rx_buf[3] == 0){
                            isDump = false;
                        }else{
                            isDump = true;
                        }
                        break;
                    }
                }
            }
        }
    }
}
