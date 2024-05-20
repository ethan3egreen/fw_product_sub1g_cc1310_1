/* Standard C Libraries */
#include <stdlib.h>
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <unistd.h>
/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
/* EasyLink API Header files */
#include "easylink/EasyLink.h"
#include "Uart2.h"
#include "RF_receive.h"

#include <ti/drivers/NVS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/vims.h)
#include DeviceFamily_constructPath(driverlib/flash.h)
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

#define FLASH_ADDRESS(page, offset) (((page) << 12) + (offset))

/* Undefine to not use async mode */
#define RFEASYLINKECHO_ASYNC
#define RFEASYLINKECHO_ADDR_FILTER

#define RFEASYLINKECHO_TASK_STACK_SIZE    4096
#define RFEASYLINKECHO_TASK_PRIORITY      2

#define RFEASYLINKECHO_PAYLOAD_LENGTH     64
#define RFEASYLINKECHO_RF_POWER           20

Task_Struct echoTask;    /* not static so you can see in ROV */
static Task_Params echoTaskParams;
static uint8_t echoTaskStack[RFEASYLINKECHO_TASK_STACK_SIZE];

static Semaphore_Handle echoDoneSem;

static EasyLink_TxPacket txPacket = {{0}, 0, 0, {0}};

uint16_t frq = 915;
uint8_t toSubG[SUBG_PACKET_MAX] = {0};
uint8_t tx_power = 0;

static void OADTarget_enableCache(uint8_t state)
{
  if (state != VIMS_MODE_DISABLED)
  {
    // Enable the Cache.
    VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);
  }
}

/*********************************************************************
 * @fn      OADTarget_disableCache
 *
 * @brief   Resumes system after a write to flash, if necessary.
 *
 * @param   None.
 *
 * @return  VIMS_MODE_ENABLED if cache was in use before this operation,
 *          VIMS_MODE_DISABLED otherwise.
 */
static uint8_t OADTarget_disableCache(void)
{
  uint8_t state = VIMSModeGet(VIMS_BASE);

  // Check VIMS state
  if (state != VIMS_MODE_DISABLED)
  {
    // Invalidate cache
    VIMSModeSet(VIMS_BASE, VIMS_MODE_DISABLED);

    // Wait for disabling to be complete
    while (VIMSModeGet(VIMS_BASE) != VIMS_MODE_DISABLED);
  }

  return state;
}

void OADTarget_readFlash(uint8_t page, uint32_t offset, uint8_t *pBuf, uint16_t len)
{

  uint8_t *ptr = (uint8_t *)FLASH_ADDRESS(page, offset);

  // Enter critical section.
  uint32_t key = Hwi_disable();

  // Read from pointer into buffer.
  while (len--)
  {
    *pBuf++ = *ptr++;
  }

  // Exit critical section.
  Hwi_restore(key);
}

/*********************************************************************
 * @fn      OADTarget_writeFlash
 *
 * @brief   Write data to flash.
 *
 * @param   page   - page to write to in flash
 * @param   offset - offset into flash page to begin writing
 * @param   pBuf   - pointer to buffer of data to write
 * @param   len    - length of data to write in bytes
 *
 * @return  None.
 */
void OADTarget_writeFlash(uint8_t page, uint32_t offset, uint8_t *pBuf, uint16_t len)
{
  uint8_t cacheState;

  cacheState = OADTarget_disableCache();

  FlashProgram(pBuf, (uint32_t)FLASH_ADDRESS(page, offset), len);

  OADTarget_enableCache(cacheState);
}

/*********************************************************************
 * @fn      OADTarget_eraseFlash
 *
 * @brief   Erase selected flash page.
 *
 * @param   page - the page to erase.
 *
 * @return  None.
 */
void OADTarget_eraseFlash(uint8_t page)
{
  uint8_t cacheState;

  cacheState = OADTarget_disableCache();

  FlashSectorErase((uint32_t)FLASH_ADDRESS(page, 0));

  OADTarget_enableCache(cacheState);
}

static void echoTxDoneCb(EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
        /* Toggle LED2 to indicate Echo TX, clear LED1 */
        //PIN_setOutputValue(pinHandle, IOID_2,!PIN_getOutputValue(IOID_2));
    }
    else
    {
        /* Set LED1 and clear LED2 to indicate error */
        //PIN_setOutputValue(pinHandle, IOID_2, 1);
        //PIN_setOutputValue(pinHandle, IOID_2,!PIN_getOutputValue(IOID_2));
    }
}

static void sendRF(const uint8_t data[],const uint8_t len)
{
    uint8_t i = 0;
    uint8_t crc = data[0];

    txPacket.len = len;
    txPacket.dstAddr[0] = '2'; //to GW
    txPacket.dstAddr[1] = 'G';
    txPacket.dstAddr[2] = 'W';

    txPacket.payload[0] = data[0];
    for(i = 1;i < len-1;i++)
    {
        txPacket.payload[i] = data[i];
        crc ^= data[i];
    }
    txPacket.payload[len-1] = crc;
    txPacket.absTime = 0;
    //20220818 by Ethan
    //EasyLink_transmitAsync(&txPacket, echoTxDoneCb);
    EasyLink_transmit(&txPacket);
}

//uint8_t CRC_checksum(uint8_t data[])
//{
//  uint8_t crc = 0,i;
//
//  for(i = 1;i <= data[1];i++) crc ^= data[i];
//
//  return crc;
//}

void sendRFTx()
{
    sendRF(toSubG,SUBG_PACKET_MAX);
}

static void rfEasyLinkEchoRxFnx(UArg arg0, UArg arg1)
{
    /* Create a semaphore for Async */
    Semaphore_Params params;
    Error_Block      eb;

    /* Init params */
    Semaphore_Params_init(&params);
    Error_init(&eb);

    /* Create semaphore instance */
    echoDoneSem = Semaphore_create(0, &params, &eb);
    if(echoDoneSem == NULL)
    {
        System_abort("Semaphore creation failed");
    }

    EasyLink_Status status;
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);
    easyLink_params.ui32ModType = EasyLink_Phy_50kbps2gfsk;

    /* Initialize EasyLink */
    if(EasyLink_init(&easyLink_params) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_init failed");
    }

    /*
     * If you wish to use a frequency other than the default, use
     * the following API:
     * EasyLink_setFrequency(868000000);
     */
    uint8_t data[100] = {0};
    OADTarget_readFlash(20,0,data,100);
    if(data[FRQ_L] != 0 && data[FRQ_H] != 0)
    {
        if(data[FRQ_L] != 0xff && data[FRQ_H] != 0xff)
        {
            frq = BUILD_UINT16(data[FRQ_L],data[FRQ_H]);
        }
    }
    status = EasyLink_setFrequency(frq * 1000000);


    // MATT MODIFIED 20210308
    if(data[TX_POWER] != 0xff)
    {
        tx_power = data[TX_POWER];
    }
    else
    {
        tx_power = 0;
    }
    status = EasyLink_setRfPower(tx_power);

    while(1)
    {
        Semaphore_pend(echoDoneSem, BIOS_WAIT_FOREVER);
    }
}

void echoTask_init(PIN_Handle inPinHandle)
{
    Task_Params_init(&echoTaskParams);
    echoTaskParams.stackSize = RFEASYLINKECHO_TASK_STACK_SIZE;
    echoTaskParams.priority = RFEASYLINKECHO_TASK_PRIORITY;
    echoTaskParams.stack = &echoTaskStack;
    echoTaskParams.arg0 = (UInt)1000000;

    Task_construct(&echoTask, rfEasyLinkEchoRxFnx, &echoTaskParams, NULL);
}
