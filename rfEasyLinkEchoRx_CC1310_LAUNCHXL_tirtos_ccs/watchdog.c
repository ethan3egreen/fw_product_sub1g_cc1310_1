#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/PIN.h>
#include <ti/sysbios/knl/Event.h>
#include <xdc/runtime/System.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include "RF_receive.h"
#include <ti/drivers/ADC.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <inc/hw_ints.h>
#include "easylink/EasyLink.h"
#include <driverlib/aon_batmon.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <math.h>
#include "Board.h"
#include "watchdog.h"
#include "Uart2.h"
#include "Uart.h"
#include <driverlib/trng.h>
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufCC26XX.h>
#include <driverlib/sys_ctrl.h>
#include "scif.h"
//#include <xdc/runtime/Types.h>

static PIN_Config keyPinsCfg[] =
{
    IOID_11 | PIN_INPUT_EN | PIN_NOPULL,
    PIN_TERMINATE
};

static PIN_State keyPins;
static PIN_Handle hKeyPins;

#define RFEASYLINKECHO_TASK_STACK_SIZE 1024
#define RFEASYLINKECHO_TASK_PRIORITY 4
#define BATT_MAX_VOLTAGE 3000
#define ASM_NOP asm(" NOP ")
#define BREAK_UINT32(var, ByteNum)\
    (uint8_t)((uint32_t)(((var) >> ((ByteNum) *8)) &0x00FF))
#define TASKSTACKSIZE    (768)
#if defined( TEG_TRUE_RMS )
#define ADCBUFFERSIZE    (120)
#define CAPTURESIZE 120
#else
#define ADCBUFFERSIZE    (240)
#define CAPTURESIZE 240
#endif
#define NODE_EVENT_ALL             0xFFFFFFFF
#define NODE_EVENT_NEW_ADC_VALUE    (uint32_t)(1 << 0)
#define NODE_EVENT_ADC_CONVERSION_START (uint32_t)(1 << 1)
//Task_Struct task0Struct;
//Char task0Stack[TASKSTACKSIZE];

//uint16_t sampleBufferOne[ADCBUFFERSIZE];
//uint16_t sampleBufferTwo[ADCBUFFERSIZE];

static Semaphore_Struct sem;
static uint8_t echoTaskStack[RFEASYLINKECHO_TASK_STACK_SIZE];
static Event_Struct radioOperationWatchdogEvent;
static Task_Params concentratorRadioTaskParams;
Task_Struct concentratorRadioWatchdogTask; /* not static so you can see in ROV */
static PIN_Handle pinHandle;
static Semaphore_Handle echoDoneSem;
//static ADC_Handle adc;
//static ADC_Handle adc_temp;
//static ADC_Params adc_params;

static ADCBuf_Handle adcBuf;
static ADCBuf_Params adcBufParams;
static ADCBuf_Conversion singleConversion;
//static ADCBufCC26XX_ParamsExtension customParams;
static bool battgetting = false;
Event_Struct nodeEvent;  /* Not static so you can see in ROV */
static Event_Handle nodeEventHandle;

//static bool isCompleted = false; //Ethan 20230501
uint8_t sampleV_Len = 83;
uint8_t pid = 0;
uint8_t ieee[8];
uint8_t sampleTime = 4;
uint8_t reset_count = 0;

// MATT MODIFIED 20221117
uint8_t offset_percent = 100;
uint8_t offset_value = 0;
uint8_t offset_sign = 0;
float currentPercent = 1.0;
float currentOffset = 0.0;
float currentPercent00 = 1.0;
float currentOffset0 = 0.0;
float currentPercent10 = 1.0;
float currentOffset1 = 0.0;
float freqCorrection = 1.0;
int16_t currentMax = 32767;
int16_t currentMin = 0;

uint8_t hw_version = 0;
uint8_t isDemo = false;
extern uint8_t toSubG[SUBG_PACKET_MAX];
extern uint16_t frq;
//extern uint8_t FW_VERSION;
extern uint8_t tx_power;
extern bool isDump;
extern bool isDumpStr;

static bool isReset = false;
static uint8_t batt = 0;
uint8_t scale = 0x01;
static MODE mode = NOT_ALLOW;
bool isAllow = FALSE;
//bool isAllow = TRUE;
//static bool isInit = FALSE;
static uint16_t sampleV[CAPTURESIZE] = { 0 };
//static uint16_t trgADC[4] = { 0 };
static uint32_t reboot_timer = 0;
extern char TEG_Device_MAC_ASC[16] = { 0 };
//static bool isCharging = false; //20220812 by Ethan
static bool isCharging = true; //20220812 by Ethan
uint16_t CurrentMin_Index;
uint8_t sine_cycle;
static uint8_t wakeCount = 0;
static uint8_t toNormalCount = 0;
static uint8_t toNoCurrentCount = 0;
static uint8_t toSleepCount = 0;
static uint16_t normalSlot = 0;
static uint32_t adcsample = 0;
static uint32_t current_sum = 0;
static uint32_t total_current = 0;
static uint16_t total_minutes = 0;
uint32_t CurrentValue;
uint8_t Range;

//#define SEND_TIMER_INTERVAL 3000 //0.02
//#define SEND_TIMER_INTERVAL 1200 //0.05
//#define SEND_TIMER_INTERVAL 600 //0.10
#define SEND_TIMER_INTERVAL 300 //0.20
static uint16_t DataSendCount = 0;
static uint16_t DataCount = 0;
static uint8_t SwitchCounter = 0;
static uint16_t Broadcast_times = 0;
static uint8_t adc_range = 1;
static uint8_t PacketCount = 0;

//#pragma DATA_SECTION(adcsample, ".section_noinit")
//uint16_t adcsample;
//#pragma DATA_SECTION(total_current, ".section_noinit")
//uint32_t total_current;
//#pragma DATA_SECTION(total_minutes, ".section_noinit")
//uint32_t total_minutes;

static void delayus(uint32_t us)
{
    uint32_t i = 0;
    for (i = 0; i < us; i++)
    {
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;

        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;

        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;

        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;

        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
        ASM_NOP;
    }
}

static void sleepms(float ms)
{
    Semaphore_pend(echoDoneSem, (ms * 1000 / Clock_tickPeriod));
}

static void sleeps(float s)
{
    Semaphore_pend(echoDoneSem, (s * 1000000 / Clock_tickPeriod));
}

static void battValueGet(void){
    ADCBuf_close(adcBuf);
    adcHandle(4, true);
    battgetting = true;
    singleConversion.samplesRequestedCount = 10;
    ADCBuf_convert(adcBuf, &singleConversion, 1);
    uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);
    battgetting = false;
}

static uint8_t battMeasure(void)
{
	double percent;
	
	// Read the battery voltage (V), only the first 12 bits
	percent = AONBatMonBatteryVoltageGet();
	
	// Convert to from V to mV to avoid fractions.
	// Fractional part is in the lower 8 bits thus converting is done as follows:
	// (1/256)/(1/1000) = 1000/256 = 125/32
	// This is done most effectively by multiplying by 125 and then shifting
	// 5 bits to the right.
	percent = (percent * 125) / 32;
	// Convert to percentage of maximum voltage.
	percent = ((percent *100) / BATT_MAX_VOLTAGE);
	
	return ((uint8_t )percent);
}

static void Board_keyCallback(PIN_Handle hPin, PIN_Id pinId)
{
    if (!isReset)
    {
        PIN_setOutputValue(pinHandle, IOID_12, 0x01);
        delayus(1);
        PIN_setOutputValue(pinHandle, IOID_12, 0x00);
    }
}

//void timerCallback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask) {
    // interrupt callback code goes here. Minimize processing in interrupt.
    /* Post event */
//    singleConversion.samplesRequestedCount = ADCBUFFERSIZE;
//    ADCBuf_convert(adcBuf, &singleConversion, 1);
//
//    Event_post(nodeEventHandle, NODE_EVENT_ADC_CONVERSION_START);
//}

static void oneMimUpdate(){
//    static uint8_t count = 0;

//    if(!isInit){
//        if(++count > 3){
//            isInit = true;
//        }
//    }

    static uint8_t five = 0;

    if(five++ > 4){
        five = 0;
        if(reboot_timer++ > 286){
        //if(reboot_timer++ > 1){
            uint8_t data_backup[2] = { 0 };
            //OADTarget_readFlash(20,0,data_backup,100);
            //OADTarget_eraseFlash(20);
            //data_backup[REBOOT_STATUS] = mode;
            //OADTarget_writeFlash(20,0,data_backup,100);
            OADTarget_readFlash(21,0,data_backup,2);
            OADTarget_eraseFlash(21);
            data_backup[1] = mode; //REBOOT_STATUS
            OADTarget_writeFlash(21,0,data_backup,2);

            isReset = true;
            //SysCtrlSystemReset();
        }
    }

    //batt = battMeasure();
    battValueGet();

}

#if defined( TEG_CT_CHARGE )
//20220812 by Ethan
static void CharingHandle(void){
    //batt = battMeasure();

    if (isCharging) {
        if (batt >= 95) // 95%->3.11v
            isCharging = false;
    }else {
        if (batt <= 88) // 88%->2.9v
            isCharging = true;
    }
}

static void ChargingStop(void)
{
    PIN_setOutputValue(pinHandle, IOID_2, 0x01);    //High : Stop Charging
}

static void ChargingStart(void)
{
    if (isCharging) //20220812 by Ethan
        PIN_setOutputValue(pinHandle, IOID_2, 0x00);    //Low : Start Charging
}
#endif

static uint16_t detectCurrent(){
    uint16_t maxV = 0;

#if defined( TEG_TRUE_RMS )
    PIN_setOutputValue(NULL, IOID_10, 0x00);
    Task_sleep( (560*1000) / Clock_tickPeriod);

    // Start ADCBuf conversion
    singleConversion.samplesRequestedCount = 1;
    ADCBuf_convert(adcBuf, &singleConversion, 1);
    // Adjust raw ADC values
    ADCBuf_adjustRawValues(adcBuf, sampleV, 1, 0);
    maxV = sampleV[0];

    PIN_setOutputValue(NULL, IOID_10, 0x01);
#else
    uint8_t i = 0;
    // Start ADCBuf conversion
    singleConversion.samplesRequestedCount = ADCBUFFERSIZE/2;
    ADCBuf_convert(adcBuf, &singleConversion, 1);
    // Adjust raw ADC values
    ADCBuf_adjustRawValues(adcBuf, sampleV, ADCBUFFERSIZE/2, 0);

    //for (i = 0; i < ADCBUFFERSIZE/2; i++)
    //{
    //    if (sampleV[i] > maxV)
    //    {
    //        maxV = sampleV[i];
    //    }
    //}
    for(i = 0; i < ADCBUFFERSIZE/2; i++){
        maxV += sampleV[i];
    }
    maxV/=ADCBUFFERSIZE/2;
#endif

#if defined( TEG_EDGE )

    float current = 0.0;

    current =  maxV;
    current = currentPercent*current+currentOffset;
    current *= 10;

    maxV = (uint16_t )current;
    if(maxV < currentMin) {
        adcsample = 0;
        return 0;
    } else if(maxV > currentMax) {
        return currentMax;
    } else {
        return maxV;
    }

#else
    // MATT ADDED 20210309
    //maxV = (maxV * offset_percent) / 100;
    //if(offset_sign == 1)
    // {
    //    maxV += offset_value;
    //}
    //else if(offset_sign == 2)
    //{
    //    maxV -= offset_value;
    //}

    if(maxV <= 6)
    {
        maxV = 0;
        adcsample = 0;
    }

    return maxV;
#endif

}

static uint16_t getCurrent(uint16_t *adcbuffer) {
    uint8_t i = 0;
    uint16_t max = adcbuffer[0];
    uint16_t min = adcbuffer[0];
    //uint16_t min_i = 0;
    uint16_t vph = 0;
    uint8_t wls = 0;
    uint16_t wl = 0;
    //uint16_t ac = 0;
    uint32_t total = 0;
    //uint16_t adcsample = 0;

    // 量測頻週期
    for(i = 0; i < CAPTURESIZE; i++){
      if(adcbuffer[i] > max){
        max = adcbuffer[i];
      }
      if(adcbuffer[i] < min){
        min = adcbuffer[i];
      }
    }
    vph = (max - min)/2+min;
    uint16_t k =  0;
    for(i = 0; i < CAPTURESIZE; i++){
      if(adcbuffer[i] != vph){
        k = i;
        break;
      }
    }
    if(adcbuffer[k] < vph){
      for(i = k; i < CAPTURESIZE; i++){
        if(adcbuffer[i] < vph){
          if(wls == 0) wls = 1;
          if(wls == 2) wls = 3;
          if(wls == 3) wl++;
        }
        if(adcbuffer[i] > vph){
          if(wls == 1) wls = 2;
          if(wls == 2) wl++;
          if(wls == 3) break;
        }
      }
    }else{
      for(i = k; i < CAPTURESIZE; i++){
        if(adcbuffer[i] > vph){
          if(wls == 0) wls = 1;
          if(wls == 2) wls = 3;
          if(wls == 3) wl++;
        }
        if(adcbuffer[i] < vph){
          if(wls == 1) wls = 2;
          if(wls == 2) wl++;
          if(wls == 3) break;
        }
      }
    }

    sine_cycle = wl;
    if(wl > CAPTURESIZE-(CAPTURESIZE/12)) {
      wl = CAPTURESIZE; //50Hz的週期長度
    }else {
      wl = CAPTURESIZE-(CAPTURESIZE/6); //60Hz的週期長度
    }
    //

    //min = sampleV[0];
    //for(i = 0; i < wl; i++){
    //  if(sampleV[i] < min){
    //    min = sampleV[i];
    //    min_i = i;
    //  }
    //}
    //CurrentMin_Index = min_i;

    //if((CAPTURESIZE-CurrentMin_Index)<wl) CurrentMin_Index = CAPTURESIZE-wl;
    //for(i = 0; i < wl; i++){
    //  ac = sampleV[CurrentMin_Index+i];
    //  total += ac;
    //}


    //計算有效值(均方根值)
    uint32_t sumx2 = 0;
    uint64_t sumx2n = 0;
    //uint64_t sumxn2 = 0;
    //uint32_t sumxn = 0;
    for(i = 0; i < wl; i++){
        sumx2 = (uint32_t)sampleV[i]*sampleV[i];
        sumx2n += sumx2;
        //sumxn += sampleV[i];
    }
    //sumxn2 = (uint64_t)sumxn*sumxn;
    //sumxn2 = sumxn2/wl;
    //if(sumx2n >= sumxn2){
    //    sumx2n -= sumxn2;
    //}else{
    //    sumx2n =0;
    //}
    sumx2n = sumx2n/wl;
    total = (uint32_t)sumx2n;

    float current = 0.0;
    current = sqrt(total);

    //線性參數調整
    current = currentPercent * current + currentOffset;
    current *= 10;
    scale = 0x01; //x10

    //adcsample = (uint16_t)current;
    if(current < currentMin) {
        //adcsample = 0;
        return 0;
    } else if(current > (currentMax*1.1)) { //超過最大值10%不更新電流回報值
        //adcsample = 0;
        return currentMax*1.1;
    } else {
        //return adcsample;
        return (uint16_t)current;
    }
}

static void readCurrent(){
    do{
        singleConversion.samplesRequestedCount = 1;
        ADCBuf_convert(adcBuf, &singleConversion, 1);
        /* Wait for event */
        uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);
        if(SwitchCounter > 0){
            sleepms((60000/SEND_TIMER_INTERVAL)-1);
        }
    }while(SwitchCounter > 0);
}

static void updateBroadcast()
{
    //Battery
    toSubG[15] = batt;

    //value of Current byte
    toSubG[19] = ((uint8_t *) &adcsample)[1];
    toSubG[20] = ((uint8_t *) &adcsample)[0];

    //status
    if (mode == NOT_ALLOW) toSubG[18] = TEG_MAC_NOT_ALLOW;
    else if (mode == DEMO_MODE) toSubG[18] = TEG_DEMO_MODE;
//    else if (!isInit) toSubG[18] = TEG_METER_INIT;
    else if (mode == INIT_MODE) toSubG[18] = TEG_METER_INIT;
    else if (mode == NORMAL_MODE){
//        if(isCharging){
//            toSubG[18] = TEG_NormalPower_Charging;
//        }else{
            toSubG[18] = TEG_NormalPower_Standby;
//        }
    }else if (mode == NO_CURRENT_MODE) toSubG[18] = TEG_NormalPower_Standby_NoCurrent;
    else if (mode == LOW_CURRENT_MODE) toSubG[18] = TEG_LowPower_Charging;
    else if (mode == SLEEP_MODE) toSubG[18] = TEG_NormalPower_SLEEP;
    else toSubG[18] = TEG_NormalPower_Standby;

#if defined( TEG_PROTOCOL_30 )
    toSubG[21] = scale;
    toSubG[23] = PacketCount;
    toSubG[24] = ((uint8_t *) &total_current)[3];
    toSubG[25] = ((uint8_t *) &total_current)[2];
    toSubG[26] = ((uint8_t *) &total_current)[1];
    toSubG[27] = ((uint8_t *) &total_current)[0];
    //toSubG[28] = ((uint8_t *) &total_minutes)[3];
    //toSubG[29] = ((uint8_t *) &total_minutes)[2];
    toSubG[28] = ((uint8_t *) &total_minutes)[1];
    toSubG[29] = ((uint8_t *) &total_minutes)[0];
#endif
    sendRFTx();
}

void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion,
    void *completedADCBuffer, uint32_t completedChannel){
    //uint_fast16_t i=0;
    uint16_t *completedBuffer = (uint16_t *) completedADCBuffer;
    uint16_t adc_raw;
    uint32_t current_value = 0;
    float adc_f = 0.0;

    /* Adjust raw adc values */
    ADCBuf_adjustRawValues(handle, completedADCBuffer, ADCBUFFERSIZE,
        completedChannel);

    if(battgetting){
        adc_raw = completedBuffer[9];
        batt = (uint8_t)(((adc_raw*(4.3/4096))/2.7397)*100);
        ADCBuf_close(adcBuf);
#if defined( TEG_REF_V148 )
        adcHandle(3, false);
#else
        adcHandle(3, true);
#endif
        battgetting = false;
        //return;
    }else{
        adc_raw = completedBuffer[0];
    //}

    if(SwitchCounter == 0){
        if(adc_range == 0){
            if(adc_raw > TEG_RANGE0_MAX){
#if defined( TEG_CM0X )
                PIN_setOutputValue(NULL, IOID_3, 0x01);
#else
                PIN_setOutputValue(NULL, IOID_10, 0x01);//TRMS Off
                PIN_setOutputValue(NULL, IOID_14, 0x00); //OPA Off
                PIN_setOutputValue(NULL, IOID_2, 0x01);//Range mode
#endif
                adc_range=1;
                SwitchCounter = SEND_TIMER_INTERVAL/20;
            }else{
#if defined( TEG_FM00_0B )
                if(adc_raw <= 380){
                    float _adc = (float)adc_raw;
                    static double _total;
                    //static double x6 = 0.000000000000000374449130220870;
                    //static double x5 = -0.000000000002244532816390960000;
                    static double x4 = -0.000000010478910581596600000000;
                    static double x3 = +0.000009817291766769670000000000;
                    static double x2 = -0.003286338313756200000000000000;
                    static double x1 = +1.472176536513310000000000000000;
                    static double x0 = -25.082522702395000000000000000000;
                    _total = x4 * pow(_adc, 4)
                             + x3 * pow(_adc, 3)
                             + x2 * pow(_adc, 2)
                             + x1 * pow(_adc, 1)
                             + x0 * pow(_adc, 0);
                    adc_raw = (uint16_t)_total;
                }
#endif
                adc_f = (float)adc_raw*currentPercent00+currentOffset0;
            }
        }else if(adc_range == 1){
            if(adc_raw < TEG_RANGE1_MIN){
#if defined( TEG_CM0X )
                PIN_setOutputValue(NULL, IOID_3, 0x00);
                adc_range=0;
                SwitchCounter = SEND_TIMER_INTERVAL/100;
#else
                PIN_setOutputValue(NULL, IOID_2, 0x00);//Range mode
                adc_range=0;
                SwitchCounter = SEND_TIMER_INTERVAL/100;
            }else if(adc_raw > 3300){
                PIN_setOutputValue(NULL, IOID_10, 0x01);//TRMS Off
                PIN_setOutputValue(NULL, IOID_14, 0x00); //OPA Off
                SwitchCounter = SEND_TIMER_INTERVAL/20;
#endif
            }else{
                //adc_f *= TEG_RANGE_MULTIPLE;
                adc_f = (float)adc_raw*currentPercent10+currentOffset1;
            }
        }

        //if(adc_f > (float)((uint32_t)currentMax*11)){
        //    adc_f = (float)((uint32_t)currentMax*11);
        //}

        if(adc_raw < currentMin){
            adc_f = 0.0;
        }

        current_value = (uint32_t)(adc_f*freqCorrection);
        current_sum += current_value;
        DataCount++;

        CurrentValue = current_value;
        if(adc_range == 0){
            Range = TEG_RANGE_LOW;
        }else{
            Range = TEG_RANGE_HIGHT;
        }

    }else{
        SwitchCounter--;
        if(SwitchCounter == SEND_TIMER_INTERVAL/60){
            PIN_setOutputValue(NULL, IOID_14, 0x01); //OPA On
            PIN_setOutputValue(NULL, IOID_10, 0x00);//TRMS On
        }
    }

    if(mode == NORMAL_MODE){
        if(++DataSendCount > SEND_TIMER_INTERVAL-1){
            current_value = current_sum/DataCount;
#if defined( TEG_CT_CHARGE )
            current_value = current_value*11/10;
#endif
            if(current_value < (uint32_t)currentMax*11){

                if(current_value < TEG_RANGE0_ZERO){
                    current_value = 0;
                }

                if(current_value < TEG_RANGELOW_MAX){
                    adcsample = current_value;
                    scale = TEG_RANGE_LOW;
                }else{
                    adcsample = current_value/10;
                    scale = TEG_RANGE_HIGHT;
                }

                if((total_current > 0xefffffff) || (total_minutes == 0xffff)){
                    total_current = 0;
                    total_minutes = 0;
                }
                total_current += current_value;
                total_minutes ++;
            }
            DataSendCount = 0;
            DataCount = 0;
            current_sum = 0;
        }
    }else if(mode == LOW_CURRENT_MODE){
        if(++DataSendCount > (SEND_TIMER_INTERVAL/5)-1){
            current_value = current_sum/DataCount;
#if defined( TEG_CT_CHARGE )
            current_value = current_value*11/10;
            current_value = current_value*102/100; //Compensation correction for simple rate 1sec.
#endif
            if(current_value < (uint32_t)currentMax*11){

                if(current_value < TEG_RANGE0_ZERO){
                    current_value = 0;
                }

                if(current_value < TEG_RANGELOW_MAX){
                    adcsample = current_value;
                    scale = TEG_RANGE_LOW;
                }else{
                    adcsample = current_value/10;
                    scale = TEG_RANGE_HIGHT;
                }

                if((total_current > 0xefffffff) || (total_minutes == 0xffff)){
                    total_current = 0;
                    total_minutes = 0;
                }
                total_current += current_value;
                total_minutes ++;
            }
            DataSendCount = 0;
            DataCount = 0;
            current_sum = 0;
        }
    }else if(mode == NOT_ALLOW){
        if(++DataSendCount > (SEND_TIMER_INTERVAL/30)-1){
            current_value = current_sum/DataCount;
#if defined( TEG_CT_CHARGE )
            adcsample = current_value*11/10;
#else
            adcsample = current_value;
#endif
            if(adc_range == 0){
                scale = TEG_RANGE_LOW;
            }else{
                scale = TEG_RANGE_HIGHT;
            }
            DataSendCount = 0;
            DataCount = 0;
            current_sum = 0;
        }
    }else{
        if(current_value < TEG_RANGELOW_MAX){
            adcsample = current_value;
            scale = TEG_RANGE_LOW;
        }else{
            adcsample = current_value/10;
            scale = TEG_RANGE_HIGHT;
        }
        DataCount = 0;
        current_sum = 0;
    }
    }

    /* Post event */
    Event_post(nodeEventHandle, NODE_EVENT_NEW_ADC_VALUE);
}

static void adcHandle(char channel, bool scaling){
    ADCBufCC26XX_ParamsExtension customParams;

    customParams.samplingDuration= ADCBufCC26XX_SAMPLING_DURATION_2P7_US;
    customParams.refSource= ADCBufCC26XX_FIXED_REFERENCE;
    customParams.samplingMode= ADCBufCC26XX_SAMPING_MODE_SYNCHRONOUS;
    customParams.inputScalingEnabled = scaling;
    adcBufParams.custom = &customParams;
    adcBuf = ADCBuf_open(CC1310_5XD_ADCBuf0, &adcBufParams);

    /* Configure the conversion struct */
    //continuousConversion.arg = NULL;
    //continuousConversion.adcChannel = Board_ADCBufChannel0;
    //continuousConversion.sampleBuffer = sampleBufferOne;
    //continuousConversion.sampleBufferTwo = sampleBufferTwo;
    //continuousConversion.samplesRequestedCount = ADCBUFFERSIZE;
    singleConversion.arg = NULL;
    singleConversion.adcChannel = channel;
    singleConversion.sampleBuffer = sampleV;
    singleConversion.sampleBufferTwo = NULL;
    //singleConversion.samplesRequestedCount = ADCBUFFERSIZE;


    //if (!adcBuf){
    //    System_abort("adcBuf did not open correctly\n");
    //}
}

static void WatchdogFnx(UArg arg0, UArg arg1)
{
    Semaphore_Params Sparams;
    Error_Block eb;
    //ADCBuf_Handle adcBuf;
    //ADCBuf_Params adcBufParams;
    //ADCBuf_Conversion continuousConversion;
    //ADCBuf_Conversion singleConversion;
    //ADCBufCC26XX_ParamsExtension customParams;
    //GPTimerCC26XX_Params GPTimerParams;
    //GPTimerCC26XX_Handle GPTimer_Handle;

    /*Init params */
    Semaphore_Params_init(&Sparams);
    Error_init(&eb);

    /*Create semaphore instance */
    echoDoneSem = Semaphore_create(0, &Sparams, &eb);
    if (echoDoneSem == NULL)
    {
        System_abort("Semaphore creation failed");
    }

    // 重置WDT
    PIN_setOutputValue(NULL, IOID_12, 0x01);
    delayus(1);
    PIN_setOutputValue(NULL, IOID_12, 0x00);

    // 設置PIN中斷
    hKeyPins = PIN_open(&keyPins, keyPinsCfg);
    PIN_registerIntCb(hKeyPins, Board_keyCallback);
    PIN_setConfig(hKeyPins, PIN_BM_IRQ, IOID_11 | PIN_IRQ_NEGEDGE);

    // 初始化ADC
    //ADC_init();
    //ADC_Params_init(&adc_params);
    //adc = ADC_open(CC1310_5XD_ADC0, &adc_params);

    //ADCBuf_init();
    /* Set up an ADCBuf peripheral in ADCBuf_RECURRENCE_MODE_CONTINUOUS */
    ADCBuf_Params_init(&adcBufParams);
    adcBufParams.callbackFxn = adcBufCallback;
    //adcBufParams.recurrenceMode = ADCBuf_RECURRENCE_MODE_CONTINUOUS;
    adcBufParams.recurrenceMode = ADCBuf_RECURRENCE_MODE_ONE_SHOT;
    adcBufParams.returnMode = ADCBuf_RETURN_MODE_CALLBACK;
    //adcBufParams.returnMode = ADCBuf_RETURN_MODE_BLOCKING;
    adcBufParams.samplingFrequency = 12000; //12KHz, 83.33us
#if defined( TEG_REF_V148 )
    adcHandle(3, false);
#else
    adcHandle(3, true);
#endif

    // GPTimer initialization
//    GPTimerCC26XX_Params_init(&GPTimerParams);
//    GPTimerParams.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;
//    GPTimerParams.mode = GPT_MODE_PERIODIC_UP;
//    GPTimerParams.width = GPT_CONFIG_32BIT;
//    GPTimer_Handle = GPTimerCC26XX_open(Board_GPTIMER1A, &GPTimerParams);
    //PG_Assert(GPTimer_Handle);

//    Types_FreqHz freq;
//    BIOS_getCpuFreq(&freq);
//    GPTimerCC26XX_Value loadVal = freq.lo - 1; //47999999 1秒
//    GPTimerCC26XX_setLoadValue(GPTimer_Handle, loadVal);
//    GPTimerCC26XX_registerInterrupt(GPTimer_Handle, timerCallback, GPT_INT_TIMEOUT);

    // 讀取RESET COUNT
    uint8_t data2[1] = { 0 };
    OADTarget_readFlash(21,0,data2,2);
    if (data2[0] != 0xff)
    {
        if (data2[0] < 0xfd)
        {
            reset_count = data2[0] + 1;
            data2[0] = reset_count;
            OADTarget_eraseFlash(21);
            OADTarget_writeFlash(21,0,data2,2);
        }
        else
        {
            reset_count = 0xfd;
        }
    }
    else
    {
        OADTarget_eraseFlash(21);
        data2[0] = 1;
        OADTarget_writeFlash(21,0,data2,2);
        reset_count = 1;
    }

    if (data2[1] != 0xff){
        mode = (MODE) data2[1]; //REBOOT_STATUS
        //mode = NORMAL_MODE;
    }else{
        mode = NOT_ALLOW;
    }

    // 讀取設定值
    uint8_t data[100] = { 0 };
    OADTarget_readFlash(20,0,data,100);
    if (data[PID] != 0xff)
    {
        pid = data[PID];
    }
    else
    {
        pid = 0;
    }
    if (data[BROADCAST_INTERVAL] != 0xff)
    {
        sampleTime = data[BROADCAST_INTERVAL];
    }
    else
    {
        sampleTime = 4;
    }
    if (data[HZ] != 0xff)
    {
        sampleV_Len = data[HZ];
    }
    else
    {
        sampleV_Len = 83;
    }

    // MATT MODIFIED 20221117
    if (data[OFFSET_PERCENT] != 0xff)
    {
        offset_percent = data[OFFSET_PERCENT];
    }
    else
    {
        offset_percent = 100;
    }
    if (data[OFFSET_RAW_SIGN] != 0xff)
    {
        offset_sign = data[OFFSET_RAW_SIGN];
    }
    else
    {
        offset_sign = 0;
    }
    if (data[OFFSET_RAW_VALUE] != 0xff)
    {
        offset_value = data[OFFSET_RAW_VALUE];
    }
    else
    {
        offset_value = 0;
    }
    if (data[CURRENT_PERCENT_B0] == 0xff &&
            data[CURRENT_PERCENT_B1] == 0xff &&
            data[CURRENT_PERCENT_B2] == 0xff &&
            data[CURRENT_PERCENT_B3] == 0xff)
    {
        currentPercent = 1.0;
    }
    else
    {
        currentPercent = *((float *)(&data[CURRENT_PERCENT_B0]));
    }
    if (data[CURRENT_OFFSET_B0] == 0xff &&
            data[CURRENT_OFFSET_B1] == 0xff &&
            data[CURRENT_OFFSET_B2] == 0xff &&
            data[CURRENT_OFFSET_B3] == 0xff)
    {
        currentOffset = 0.0;
    }
    else
    {
        currentOffset = *((float *)(&data[CURRENT_OFFSET_B0]));
    }

    if (data[CURRENT_PERCENT00_B0] == 0xff &&
            data[CURRENT_PERCENT00_B1] == 0xff &&
            data[CURRENT_PERCENT00_B2] == 0xff &&
            data[CURRENT_PERCENT00_B3] == 0xff){
        currentPercent00 = 1.0;
    }else{
        currentPercent00 = *((float *)(&data[CURRENT_PERCENT00_B0]));
    }

    if (data[CURRENT_OFFSET0_B0] == 0xff &&
            data[CURRENT_OFFSET0_B1] == 0xff &&
            data[CURRENT_OFFSET0_B2] == 0xff &&
            data[CURRENT_OFFSET0_B3] == 0xff){
        currentOffset0 = 0.0;
    }else{
        currentOffset0 = *((float *)(&data[CURRENT_OFFSET0_B0]));
    }

    if (data[CURRENT_PERCENT10_B0] == 0xff &&
            data[CURRENT_PERCENT10_B1] == 0xff &&
            data[CURRENT_PERCENT10_B2] == 0xff &&
            data[CURRENT_PERCENT10_B3] == 0xff){
        currentPercent10 = 1.0;
    }else{
        currentPercent10 = *((float *)(&data[CURRENT_PERCENT10_B0]));
    }

    if (data[CURRENT_OFFSET1_B0] == 0xff &&
            data[CURRENT_OFFSET1_B1] == 0xff &&
            data[CURRENT_OFFSET1_B2] == 0xff &&
            data[CURRENT_OFFSET1_B3] == 0xff){
        currentOffset1 = 0.0;
    }else{
        currentOffset1 = *((float *)(&data[CURRENT_OFFSET1_B0]));
    }

    if (data[FREQ_CORRECTION_B0] == 0xff &&
            data[FREQ_CORRECTION_B1] == 0xff &&
            data[FREQ_CORRECTION_B2] == 0xff &&
            data[FREQ_CORRECTION_B3] == 0xff){
        freqCorrection = 1.0;
    }else{
        freqCorrection = *((float *)(&data[FREQ_CORRECTION_B0]));
    }

    if (data[CURRENT_MIN_B0] == 0xff &&
            data[CURRENT_MIN_B1] == 0xff)
    {
        currentMin = 0;
    }
    else
    {
        currentMin = *((int16_t *)(&data[CURRENT_MIN_B0]));
    }
    if (data[CURRENT_MAX_B0] == 0xff &&
            data[CURRENT_MAX_B1] == 0xff)
    {
        currentMax = 32767;
    }
    else
    {
        currentMax = *((int16_t *)(&data[CURRENT_MAX_B0]));
    }

    if (data[HARDWARE_ID] != 0xff)
    {
        hw_version = data[HARDWARE_ID];
    }
    else
    {
        hw_version = 0;
    }
//    if (data[REBOOT_STATUS] != 0xff)
//    {
//        mode = (MODE) data[REBOOT_STATUS];
        //mode = NORMAL_MODE;
//    }
//    else
//    {
//        mode = NOT_ALLOW;
//    }
    if (data[DEMO] != 0xff)
    {
        isDemo = data[DEMO];
    }
    else
    {
        isDemo = FALSE;
    }

    // 讀取MAC
    EasyLink_getIeeeAddr(ieee);
    uint8_t Device_MAC[16] = { 0 };
    OADTarget_readFlash(22,0,Device_MAC,16);
    hextochar(ieee[0], &TEG_Device_MAC_ASC[0]);
    hextochar(ieee[1], &TEG_Device_MAC_ASC[2]);
    hextochar(ieee[2], &TEG_Device_MAC_ASC[4]);
    hextochar(ieee[3], &TEG_Device_MAC_ASC[6]);
    hextochar(ieee[4], &TEG_Device_MAC_ASC[8]);
    hextochar(ieee[5], &TEG_Device_MAC_ASC[10]);
    hextochar(ieee[6], &TEG_Device_MAC_ASC[12]);
    hextochar(ieee[7], &TEG_Device_MAC_ASC[14]);

    // 驗證MAC
    uint8_t i = 0;
    uint8_t countM = 0;
    for (i = 0; i < 16; i++)
    {
        if (TEG_Device_MAC_ASC[i] == Device_MAC[i] - MAC_DECODE)
        {
            countM++;
        }
    }
    if (countM == 16)
    {
        isAllow = TRUE;
    }
#if defined( TEST_ALLOW )
    isAllow = TRUE;
#endif

    // 初始化封包
    memset(toSubG, 0, SUBG_PACKET_MAX);
    toSubG[0] = 0x3E;
#if defined( TEG_PROTOCOL_30 )
    toSubG[1] = 0x30;
    toSubG[2] = 0;  //RSSI
    toSubG[3] = ieee[0];
    toSubG[4] = ieee[1];
    toSubG[5] = ieee[2];
    toSubG[6] = ieee[3];
    toSubG[7] = ieee[4];
    toSubG[8] = ieee[5];
    toSubG[9] = ieee[6];
    toSubG[10] = ieee[7];
    toSubG[11] = 0x00;  //Company ID : 3Egreen
    toSubG[12] = pid;   //Product ID
    toSubG[13] = FW_VERSION;    //FW Version
    toSubG[14] = hw_version;    //HW Version
    toSubG[15] = 0; //Battery
    toSubG[16] = 1; //Packet Type:1     //F1 = not show,01 = show
    toSubG[17] = reset_count;
    toSubG[18] = 0; //Status
    toSubG[19] = 0; //Current
    toSubG[20] = 0; //Current
    toSubG[21] = 2; //Output value '1'x0.1, '2'x0.01
    toSubG[22] = 9; //Length
    toSubG[23] = 0; //Packet Counter
    toSubG[24] = 0; //Total Current
    toSubG[25] = 0; //Total Current
    toSubG[26] = 0; //Total Current
    toSubG[27] = 0; //Total Current
    toSubG[28] = 0; //Total Minutes
    toSubG[29] = 0; //Total Minutes
    //toSubG[30] = 0; //Total Minutes
    //toSubG[31] = 0; //Total Minutes
    toSubG[30] = 2; //Output value '1'x0.1, '2'x0.01
    toSubG[SUBG_PACKET_MAX-1] = 0; //CRC
#elif defined( TEG_PROTOCOL_20 )
    toSubG[1] = 0x20;
    toSubG[2] = 0;  //RSSI
    toSubG[3] = ieee[0];
    toSubG[4] = ieee[1];
    toSubG[5] = ieee[2];
    toSubG[6] = ieee[3];
    toSubG[7] = ieee[4];
    toSubG[8] = ieee[5];
    toSubG[9] = ieee[6];
    toSubG[10] = ieee[7];
    toSubG[11] = 0x00;  //Company ID : 3Egreen
    toSubG[12] = pid;   //Product ID
    toSubG[13] = FW_VERSION;    //FW Version
    toSubG[14] = hw_version;    //HW Version
    toSubG[15] = 0; //Battery
    toSubG[16] = 1; //Packet Type:1     //F1 = not show,01 = show
    toSubG[17] = reset_count;
    toSubG[18] = 0; //Status
    toSubG[19] = 0; //Current
    toSubG[20] = 0; //Current
    toSubG[21] = 0; //CRC
#else
    toSubG[1] = 0x10;
    toSubG[2] = 0;  //RSSI
    toSubG[3] = ieee[0];
    toSubG[4] = ieee[1];
    toSubG[5] = ieee[2];
    toSubG[6] = ieee[3];
    toSubG[7] = ieee[4];
    toSubG[8] = ieee[5];
    toSubG[9] = ieee[6];
    toSubG[10] = ieee[7];
    toSubG[11] = 0x00;  //Company ID : 3Egreen
    toSubG[12] = pid;   //Product ID
    toSubG[13] = FW_VERSION;    //FW Version
    toSubG[14] = hw_version;    //HW Version
    toSubG[15] = 0; //Battery
    toSubG[16] = 1; //Packet Type:1     //F1 = not show,01 = show
    toSubG[17] = reset_count;
    toSubG[18] = 0; //Status
    toSubG[19] = 0; //Current
    toSubG[20] = 0; //Current
    toSubG[21] = 0x7F; //temperature, no temperature sensor
    toSubG[22] = 0xFF; //temperature, no temperature sensor
    toSubG[23] = 0; //CRC
#endif

    // 產生亂數種子
    Power_setDependency(PowerCC26XX_PERIPH_TRNG);
    TRNGEnable();
    while (!(TRNGStatusGet() & TRNG_NUMBER_READY));
    srand(TRNGNumberGet(TRNG_LOW_WORD));//RAND_MAX=32767
    TRNGDisable();
    Power_releaseDependency(PowerCC26XX_PERIPH_TRNG);

    oneMimUpdate();

    uint32_t rstsrc = SysCtrlResetSourceGet();
    if(rstsrc == RSTSRC_PWR_ON){
        total_current = 0;
        total_minutes = 0;
        adcsample = 0;

        //PIN_setOutputValue(NULL, IOID_14, 0x01); //OPA On
        //sleeps(3);
        //PIN_setOutputValue(NULL, IOID_10, 0x00); //TRMS On
        //sleeps(4);
    }

    PIN_setOutputValue(NULL, IOID_14, 0x01); //OPA On
    PIN_setOutputValue(NULL, IOID_10, 0x00); //TRMS On

    sleeps(4);
    if(mode == NORMAL_MODE){
        //Broadcast_times = 2;
        //DataSendCount = SEND_TIMER_INTERVAL/3;
        //normalSlot = 2*(SEND_TIMER_INTERVAL/3)+(rand()%50);
        mode = INIT_MODE;
        wakeCount = 1;
    //}else if(mode == NO_CURRENT_MODE){
    //    wakeCount = 2;
    //}else if(mode == SLEEP_MODE){
    //    wakeCount = 2;
    }else{
        wakeCount = 2;
    }

    while(true){
        switch (mode){
            case NOT_ALLOW:{
                if(isAllow){
                    if(isDemo){
                        mode = DEMO_MODE;
                    }else{
                        mode = INIT_MODE;
                        //mode = NORMAL_MODE;
                    }
                }else{
#if defined( TEG_TRUE_RMS )
                    singleConversion.samplesRequestedCount = 1;
                    ADCBuf_convert(adcBuf, &singleConversion, 1);
                    /* Wait for event */
                    uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);

                    if(DataSendCount == 0){
//                      updateBroadcast();
                        if(isDumpStr){
                            char str[] = "";
                            System_sprintf(str,"%u\t(%d)\r\n", (long)adcsample, scale);
                            //char len = strlen(str);
                            scifUartTxPutChars((char*)str,strlen(str));
                        }

                        if(isDump){
                            uint8_t tx[7] = {0};
                            tx[0] = 0x3E;
                            tx[1] = 1+1+3;
                            tx[2] = GET_CURRENT;
                            tx[3] = ((uint8_t *) &adcsample)[1];
                            tx[4] = ((uint8_t *) &adcsample)[0];
                            tx[5] = scale;
                            tx[6] = 0xE3;
                            scifUartTxPutChars((char*)tx,7);
                        }

                        if(++Broadcast_times > 29) {
                            Broadcast_times = 0;
                            PacketCount++;
                            oneMimUpdate();
                        }
                    }
    #if defined( TEG_CT_CHARGE )
                    ChargingStart();
                    sleepms((30000/SEND_TIMER_INTERVAL));
                    ChargingStop();
                    sleepms((30000/SEND_TIMER_INTERVAL)-1);
    #else
                    sleepms((60000/SEND_TIMER_INTERVAL)-1);
    #endif
#else
#endif
                }break;
            }
            case DEMO_MODE:{
                singleConversion.samplesRequestedCount = 1;
                ADCBuf_convert(adcBuf, &singleConversion, 1);
                /* Wait for event */
                uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);
                if(DataSendCount == 0){
                    updateBroadcast();
                    if(++Broadcast_times > 29) {
                        Broadcast_times = 0;
                        PacketCount++;
                        oneMimUpdate();
                    }
                }
                sleepms((60000/SEND_TIMER_INTERVAL)-1);
                break;
            }
            case INIT_MODE:{
                //static uint8_t initCount = 0;
                //static uint8_t init2NormalCount = 0;
                //static uint8_t init2SleepCount = 0;

                if(++wakeCount > 2){
                    wakeCount = 0;
#if defined( TEG_TRUE_RMS )
                    PIN_setOutputValue(NULL, IOID_10, 0x00); //TRMS On
                    sleepms(3998);
//                    do{
//                        singleConversion.samplesRequestedCount = 1;
//                        ADCBuf_convert(adcBuf, &singleConversion, 1);
                        /* Wait for event */
//                        uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);
//                        if(SwitchCounter > 0){
//                            sleepms((60000/SEND_TIMER_INTERVAL)-1);
//                        }
//                    }while(SwitchCounter > 0);
                    readCurrent();
                    //adcsample = adcsample*9/10;

                    uint8_t slot = {0};
                    slot = rand() % 100;
                    sleepms((slot*40)+39);
                    updateBroadcast();
                    sleepms(3961-(slot*40));

                    if(++Broadcast_times > 2) {
                        Broadcast_times = 0;
                        PacketCount++;
                        oneMimUpdate();
                    }

                    if(toNormalCount > 2){
                        toNormalCount = 0;
                        toSleepCount = 0;
                        DataSendCount = Broadcast_times*(SEND_TIMER_INTERVAL/3); //Inital DataSend for Normal status
                        uint8_t slot = {0};
                        slot = rand() % 50;
                        normalSlot = DataSendCount+slot;
                        if(DataSendCount > 0){
                            DataSendCount -= SEND_TIMER_INTERVAL/3;
                        }else{
                            DataSendCount = 2*(SEND_TIMER_INTERVAL/3);
                        }
                        //normalSlot = (DataSendCount+(SEND_TIMER_INTERVAL/6))+(rand()%50);
                        mode = NORMAL_MODE;
                        break;
                    }

                    if(adcsample > 0){
                        toNormalCount++;
                    }else{
                        toNormalCount = 0;
                        PIN_setOutputValue(NULL, IOID_10, 0x01); //TRMS Off
                    }

                    if(++toSleepCount > 8){
                        toNormalCount = 0;
                        toSleepCount = 0;
                        mode = SLEEP_MODE;
                        PIN_setOutputValue(NULL, IOID_10, 0x01); //TRMS Off
                    }

#else
#endif
                }else{
                    if(toNormalCount > 0){
//                        do{
//                            singleConversion.samplesRequestedCount = 1;
//                            ADCBuf_convert(adcBuf, &singleConversion, 1);
                            /* Wait for event */
//                            uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);
//                            if(SwitchCounter > 0){
//                                sleepms((60000/SEND_TIMER_INTERVAL)-1);
//                            }
//                        }while(SwitchCounter > 0);
                        readCurrent();
                        if(adcsample > 0){
                            toNormalCount++;
                        }else{
                            toNormalCount = 0;
                            PIN_setOutputValue(NULL, IOID_10, 0x01); //TRMS Off
                        }
                    }
                    sleeps(4);//Wake up every 5 seconds to done the watchdog
                }
                break;
            }
            case NORMAL_MODE:{
                //uint8_t slot = {0};
                //slot = rand() % (sampleTime*1000/40);

#if defined( TEG_TRUE_RMS )
                singleConversion.samplesRequestedCount = 1;
                ADCBuf_convert(adcBuf, &singleConversion, 1);
                /* Wait for event */
                uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);

                if(DataSendCount == normalSlot){
                    updateBroadcast();
                    if(++Broadcast_times > 2) {
                        Broadcast_times = 0;
                        PacketCount++;
                        oneMimUpdate();

                        if(adcsample > 0){
    #if defined( TEG_CT_CHARGE )
                            if(adcsample < 1000){//Change the sampling rate below 9.99A to 1s
                                toNoCurrentCount = 0;
                                //DataSendCount = Broadcast_times*(SEND_TIMER_INTERVAL/15); //Inital DataSend for Normal status
                                //normalSlot = DataSendCount+(rand()%10);
                                normalSlot = rand()%10;
                                DataSendCount = 2*(SEND_TIMER_INTERVAL/15);
                                mode = LOW_CURRENT_MODE;
                                break;
                            }else{
                                toNoCurrentCount = 0;
                            }
    #else
                            toNoCurrentCount = 0;
    #endif
                        }else{
                            if(++toNoCurrentCount > 2){
                                toNoCurrentCount = 0;
                                isCharging = false;
                                mode = NO_CURRENT_MODE;
                                break;
                            }
                        }
    #if defined( TEG_CT_CHARGE )
                        //CharingHandle();
    #endif
                    }
                    normalSlot = Broadcast_times*(SEND_TIMER_INTERVAL/3)+(rand()%50);
                }

    #if defined( TEG_CT_CHARGE )
                ChargingStart();
                sleepms((30000/SEND_TIMER_INTERVAL));
                ChargingStop();
                sleepms((30000/SEND_TIMER_INTERVAL)-1);
    #else
                sleepms((60000/SEND_TIMER_INTERVAL)-1);
    #endif
#else
                singleConversion.samplesRequestedCount = ADCBUFFERSIZE;
                ADCBuf_convert(adcBuf, &singleConversion, 1);
                /* Wait for event */
                uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);
                /* If new ADC value, send this data */
                //if (events & NODE_EVENT_NEW_ADC_VALUE) {

//                }
                CharingHandle();
                ChargingStart();
                sleepms(68);
                ChargingStop();

                sleepms(10);
#endif
                break;
            }
            case LOW_CURRENT_MODE:{
#if defined( TEG_TRUE_RMS )
//                singleConversion.samplesRequestedCount = 1;
//                ADCBuf_convert(adcBuf, &singleConversion, 1);
                /* Wait for event */
//                uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);
                readCurrent();

                if(DataSendCount == normalSlot){
                    updateBroadcast();
                    if(++Broadcast_times > 2) {
                        Broadcast_times = 0;
                        PacketCount++;
                        oneMimUpdate();

                        if(adcsample > 0){
                            if(adcsample > 1000){//Change the sampling rate above 10.01A to 0.2s
                                toNoCurrentCount = 0;
                                //DataSendCount = Broadcast_times*(SEND_TIMER_INTERVAL/3); //Inital DataSend for Normal status
                                //normalSlot = DataSendCount+(rand()%50);
                                normalSlot = rand()%50;
                                DataSendCount = 2*(SEND_TIMER_INTERVAL/3);
                                mode = NORMAL_MODE;
                                break;
                            }else{
                                toNoCurrentCount = 0;
                            }
                        }else{
                            if(++toNoCurrentCount > 2){
                                toNoCurrentCount = 0;
                                isCharging = false;
                                mode = NO_CURRENT_MODE;
                                break;
                            }
                        }
    #if defined( TEG_CT_CHARGE )
                        //CharingHandle();
    #endif
                    }
                    normalSlot = Broadcast_times*(SEND_TIMER_INTERVAL/15)+(rand()%10);
                }

    #if defined( TEG_CT_CHARGE )
                ChargingStart();
                sleepms((30000/SEND_TIMER_INTERVAL)*9);//900ms
                ChargingStop();
                sleepms((30000/SEND_TIMER_INTERVAL)-1);//98ms
    #else
                sleepms((60000/SEND_TIMER_INTERVAL)-1);
    #endif
#else
#endif
                break;
            }
            case NO_CURRENT_MODE:{
                //static uint8_t toSleepCount = 0;
                //static uint8_t toNormalCount = 0;

                if(++wakeCount > 2){
                    wakeCount = 0;
#if defined( TEG_TRUE_RMS )
                    PIN_setOutputValue(NULL, IOID_10, 0x00); //TRMS On
                    sleepms(3998);
//                    do{
//                        singleConversion.samplesRequestedCount = 1;
//                        ADCBuf_convert(adcBuf, &singleConversion, 1);
                        /* Wait for event */
//                        uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);
//                        if(SwitchCounter > 0){
//                            sleepms((60000/SEND_TIMER_INTERVAL)-1);
//                        }
//                    }while(SwitchCounter > 0);
                    readCurrent();
                    //adcsample = adcsample*9/10;

                    uint8_t slot = {0};
                    slot = rand() % 100;
                    sleepms((slot*40)+39);
                    updateBroadcast();
                    sleepms(3961-(slot*40));

                    if(++Broadcast_times > 2) {
                        Broadcast_times = 0;
                        PacketCount++;
                        oneMimUpdate();
                    }

                    if(toNormalCount > 2){
                        toNormalCount = 0;
                        toSleepCount = 0;
                        DataSendCount = Broadcast_times*(SEND_TIMER_INTERVAL/3); //Inital DataSend for Normal status
                        normalSlot = DataSendCount+(rand()%50);
                        if(DataSendCount > 0){
                            DataSendCount -= SEND_TIMER_INTERVAL/3;
                        }else{
                            DataSendCount = 2*(SEND_TIMER_INTERVAL/3);
                        }
                        mode = NORMAL_MODE;
                        break;
                    }

                    if(adcsample > 0){
                        toNormalCount++;
                    }else{
                        toNormalCount = 0;
                        PIN_setOutputValue(NULL, IOID_10, 0x01); //TRMS Off
                    }

                    if(++toSleepCount > 179){
                        toNormalCount = 0;
                        toSleepCount = 0;
                        mode = SLEEP_MODE;
                        PIN_setOutputValue(NULL, IOID_10, 0x01); //TRMS Off
                    }

#else
#endif
                }else{
                    if(toNormalCount > 0){
//                        do{
//                            singleConversion.samplesRequestedCount = 1;
//                            ADCBuf_convert(adcBuf, &singleConversion, 1);
                            /* Wait for event */
//                            uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);
//                            if(SwitchCounter > 0){
//                                sleepms((60000/SEND_TIMER_INTERVAL)-1);
//                            }
//                        }while(SwitchCounter > 0);
                        readCurrent();
                        if(adcsample > 0){
                            toNormalCount++;
                        }else{
                            toNormalCount = 0;
                            PIN_setOutputValue(NULL, IOID_10, 0x01); //TRMS Off
                        }
                    }
                    sleeps(4);//Wake up every 5 seconds to done the watchdog
                }
                break;
            }
            case SLEEP_MODE:{
                //static uint8_t triggerCount = 0;

#if defined( TEG_TRUE_RMS )
                if(++wakeCount > 2){
                    wakeCount = 0;

                    uint8_t slot = {0};
                    slot = rand() % 100;
                    sleepms((slot*40)+39);
                    updateBroadcast();
                    sleepms(3961-(slot*40));

                    if(++Broadcast_times > 2) {
                        Broadcast_times = 0;
                        PacketCount++;
                        oneMimUpdate();

                        PIN_setOutputValue(NULL, IOID_10, 0x00); //TRMS On
                        sleepms(3998);
//                        do{
//                            singleConversion.samplesRequestedCount = 1;
//                            ADCBuf_convert(adcBuf, &singleConversion, 1);
                            /* Wait for event */
//                            uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);
//                            if(SwitchCounter > 0){
//                                sleepms((60000/SEND_TIMER_INTERVAL)-1);
//                            }
//                        }while(SwitchCounter > 0);
                        readCurrent();
                        //adcsample = adcsample*9/10;

                        if(adcsample > 0){
                            toNormalCount = 1;
                        }
                        PIN_setOutputValue(NULL, IOID_10, 0x01); //TRMS Off
                        //sleeps(4);
                        //break;
                    }else if(toNormalCount > 0){
                        PIN_setOutputValue(NULL, IOID_10, 0x00); //TRMS On
                        sleepms(3998);

//                        singleConversion.samplesRequestedCount = 1;
//                        ADCBuf_convert(adcBuf, &singleConversion, 1);
                        /* Wait for event */
//                        uint32_t events = Event_pend(nodeEventHandle, 0, NODE_EVENT_ALL, BIOS_WAIT_FOREVER);
                        readCurrent();

                        if(adcsample > 0){
                            if(++toNormalCount > 2){
                                toNormalCount = 0;
                                DataSendCount = Broadcast_times*(SEND_TIMER_INTERVAL/3); //Inital DataSend for Normal status
                                normalSlot = DataSendCount+(rand()%50);
                                if(DataSendCount > 0){
                                    DataSendCount -= SEND_TIMER_INTERVAL/3;
                                }else{
                                    DataSendCount = 2*(SEND_TIMER_INTERVAL/3);
                                }
                                mode = NORMAL_MODE;
                                break;
                            }
                        }else{
                            toNormalCount = 0;
                            PIN_setOutputValue(NULL, IOID_10, 0x01); //TRMS Off
                        }
                    }else{
                        sleeps(4);
                    }
#else
#endif
                }else{
                    sleeps(4);
                }break;
            }
            default:{
                break;
            }
        }
    }
}

void WatchdogFn_init()
{
#if defined( TEG_CM0X )
    PIN_setOutputValue(NULL, IOID_2, 0x01);
    PIN_setOutputValue(NULL, IOID_3, 0x00); //Range mode
#else
    PIN_setOutputValue(NULL, IOID_2, 0x01); //Range mode
#endif
    PIN_setOutputValue(NULL, IOID_14, 0x00); //OPA Off
    PIN_setOutputValue(NULL, IOID_10, 0x01); //TRMS Off

    Task_Params_init(&concentratorRadioTaskParams);
    concentratorRadioTaskParams.stackSize = RFEASYLINKECHO_TASK_STACK_SIZE;
    concentratorRadioTaskParams.priority = RFEASYLINKECHO_TASK_PRIORITY;
    concentratorRadioTaskParams.stack = &echoTaskStack;
    concentratorRadioTaskParams.arg0 = (UInt) 1000000;
    Task_construct(&concentratorRadioWatchdogTask, WatchdogFnx, &concentratorRadioTaskParams, NULL);

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&nodeEvent, &eventParam);
    nodeEventHandle = Event_handle(&nodeEvent);
}
