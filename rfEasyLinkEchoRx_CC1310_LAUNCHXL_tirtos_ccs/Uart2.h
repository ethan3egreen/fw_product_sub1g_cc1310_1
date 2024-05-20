#ifndef UART2_H
#define UART2_H
#include "Board.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MAC_DECODE                            2

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

typedef enum
{
    //general subg command
    GET_MAC = 0x01,//1
    GET_FIRMWARE_VERSION,//2
    SET_FRQ,//3
    GET_FRQ,//4
    SET_PID,//5
    GET_PID,//6
    SET_TX_POWER,//7
    GET_TX_POWER,//8
    SET_HARDWARE_ID,//9
    GET_HARDWARE_ID,//0A
    GET_CURRENTSTR,//0B
    CURRENT_DUMPSTR,//0C
    GET_CURRENT,//0D
    CURRENT_DUMP,//0E
    //subg meter command
    SET_HZ = 0x40,//40
    GET_HZ,//41
    CLEAR_RESET,//42
    RESET_CMD,//43

    // MATT MODIFIED 20221117
#if defined( TEG_EDGE )
    SET_CURRENT_PERCENT,//44
    GET_CURRENT_PERCENT,//45
    SET_CURRENT_OFFSET,//46
    GET_CURRENT_OFFSET,//47
#else
    SET_OFFSET_PERCENT,//44
    GET_OFFSET_PERCENT,//45
    SET_OFFSET_RAW,//46
    GET_OFFSET_RAW,//47
#endif

    SET_BORADCAST_INTERVAL,//48
    GET_BORADCAST_INTERVAL,//49
    SET_ENABLE_MAC,//4A
    GET_ENABLE_MAC,//4B
    SET_DEMO,//4C
    GET_DEMO,//4D

    // MATT ADDED 20221117
    SET_CURRENT_MIN,//4E
    GET_CURRENT_MIN,//4F
    SET_CURRENT_MAX,//50
    GET_CURRENT_MAX,//51
#if defined( TEG_EDGE )
    SET_OFFSET_PERCENT,//52
    GET_OFFSET_PERCENT,//53
    SET_OFFSET_RAW,//54
    GET_OFFSET_RAW,//55
#else
    SET_CURRENT_PERCENT,//52
    GET_CURRENT_PERCENT,//53
    SET_CURRENT_OFFSET,//54
    GET_CURRENT_OFFSET,//55
#endif
    SET_CURRENT_PERCENT00,//56
    GET_CURRENT_PERCENT00,//57
    SET_CURRENT_OFFSET0,//58
    GET_CURRENT_OFFSET0,//59
    SET_CURRENT_PERCENT10,//5A
    GET_CURRENT_PERCENT10,//5B
    SET_CURRENT_OFFSET1,//5C
    GET_CURRENT_OFFSET1,//5D
    SET_FREQ_CORRECTION,//5E
    GET_FREQ_CORRECTION,//5F
    //subg repeater white list 
	ENABLE_FILTER = 0x80,
    PING_ACK = 0xAA,
	
} UART_COMMAND;

typedef enum
{
    FRQ_L = 0,
    FRQ_H,
    PID,
    BROADCAST_INTERVAL,
    TX_POWER,

    // MATT MODIFIED 20221117
    OFFSET_PERCENT,
    OFFSET_RAW_SIGN,
    OFFSET_RAW_VALUE,
    CURRENT_PERCENT_B0,
    CURRENT_PERCENT_B1,
    CURRENT_PERCENT_B2,
    CURRENT_PERCENT_B3,
    CURRENT_OFFSET_B0,
    CURRENT_OFFSET_B1,
    CURRENT_OFFSET_B2,
    CURRENT_OFFSET_B3,
    CURRENT_PERCENT00_B0,
    CURRENT_PERCENT00_B1,
    CURRENT_PERCENT00_B2,
    CURRENT_PERCENT00_B3,
    CURRENT_OFFSET0_B0,
    CURRENT_OFFSET0_B1,
    CURRENT_OFFSET0_B2,
    CURRENT_OFFSET0_B3,
    CURRENT_PERCENT10_B0,
    CURRENT_PERCENT10_B1,
    CURRENT_PERCENT10_B2,
    CURRENT_PERCENT10_B3,
    CURRENT_OFFSET1_B0,
    CURRENT_OFFSET1_B1,
    CURRENT_OFFSET1_B2,
    CURRENT_OFFSET1_B3,
    CURRENT_MIN_B0,
    CURRENT_MIN_B1,
    CURRENT_MAX_B0,
    CURRENT_MAX_B1,
    FREQ_CORRECTION_B0,
    FREQ_CORRECTION_B1,
    FREQ_CORRECTION_B2,
    FREQ_CORRECTION_B3,

    DEMO,
    HZ,
    ADC_INTERVAL,
    HARDWARE_ID,
    REBOOT_STATUS,
    CHARGIN_STATUS,
    NAME1,
    NAME2,
    NAME3,
    NAME4,
    NAME5,
    NAME6,
    NAME7,
    NAME8,
    NAME9,
    NAME10,
    NAME11,
    NAME12,
} FLASH_MAP;

/*
 * Task creation function 
 */
void Uart2_createTask();

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif 
