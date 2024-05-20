static void adcHandle(char channel, bool scaling);
void WatchdogFn_init();
typedef enum
{
    NOT_ALLOW,
    INIT_MODE,
    DEMO_MODE,
    NORMAL_MODE,
    CHARGING_MODE,
    NO_CURRENT_MODE,
    LOW_CURRENT_MODE,
    //SLOW_MODE,
    //SLOW_NO_CURRENT_MODE,
    SLEEP_MODE,
    //FAST_MODE,
    //FAST_CHARGING_MODE,
    //FAST_NO_CURRENT_MODE,

} MODE;

typedef enum
{
    TEG_NormalPower_Standby            = 0x01,
    TEG_NormalPower_Charging           = 0x02,
    TEG_LowPower_Charging              = 0x03,
    TEG_NormalPower_Standby_NoCurrent  = 0x04,
    //TEG_NormalPower_Charging_NoCurrent = 0x05,
    //TEG_LowPower_Charging_NoCurrent    = 0x06,
    TEG_NormalPower_SLEEP              = 0x07,
    TEG_MAC_NOT_ALLOW                  = 0x08,
    TEG_METER_INIT                     = 0x09,
    TEG_DEMO_MODE                      = 0x0A,
    //TEG_FAST_MODE_Standby              = 0x0B,
    //TEG_FAST_MODE_Charging             = 0x0C,
    //TEG_FAST_MODE_NoCurrent            = 0x0D,
    //TEG_NormalPower_Slow               = 0x0E,

} TEG_ChargState;
