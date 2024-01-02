
#include "main.hpp"
#include "config.hpp"

enum InterfaceVariables{
//HEX FORMAT 16bit
    /*  Commands variables            = 0x0000,*/
    CMD_NONE                          = 0x0000,
    CMD_START                         = 0x0001,
    CMD_STOP                          = 0x0002,
    CMD_LINE_CALIBRATE                = 0x0003,
    CMD_IMU_CALIBRATE                 = 0x0004,
    CMD_NVM_ERASE                     = 0x0005,
    CMD_NVM_LOAD                      = 0x0006,
    CMD_NVM_SAVE                      = 0x0007,
    CMD_SLEEP                         = 0x0008,
    CMD_WAKEUP                        = 0x0009,
    CMD_NVM_RELOAD                    = 0x000A,
    CMD_ESC_START                     = 0x000B,
    CMD_ESC_STOP                      = 0x000C,
    VAR_BAT_VOLTAGE                   = 0x000D,
    /*  Common variables              = 0x0100,*/
    VAR_SPEED                         = 0x0100,
    VAR_ESC_SPEED                     = 0x0101,
    VAR_STEERING_GAIN                 = 0x0102,
    VAR_STEERING_GAIN2                = 0x0103,
    VAR_STEERING_GAIN3                = 0x0104,
    VAR_STEERING_GAIN4                = 0x0105,
    VAR_STEERING_GAIN5                = 0x0106,
    VAR_STEERING_GAIN6                = 0x0107,
    VAR_CRUISE_GAIN                   = 0x0108,
    VAR_CRUISE_GAIN2                  = 0x0109,
    VAR_CRUISE_GAIN3                  = 0x010A,
    VAR_CRUISE_GAIN4                  = 0x010B,
    VAR_CRUISE_GAIN5                  = 0x010C,
    VAR_CRUISE_GAIN6                  = 0x010D,
    VAR_BRAKE_TIMEOUT                 = 0x010E,
    VAR_KP                            = 0X010F,
    VAR_KD                            = 0X0110,
    VAR_MAX_SPEED                     = 0X0111,
    VAR_RAMP_SPEED                    = 0X0112,
    VAR_RAMP_SPEED_DOWN               = 0X0113,
    VAR_WALL_SPEED                    = 0X0114,
    VAR_WALL_ANGLE                    = 0X0115,
    VAR_WALL_TH                       = 0X0116,
    VAR_WALL_TIME1                    = 0X0117,
    VAR_WALL_TIME2                    = 0X0118,
    VAR_DISTANCE                      = 0X0119,
    /*  IMU variables                 = 0x0200,*/
    /*  Line variables                = 0x0220,*/
    VAR_LINE_EMITTER_POWER            = 0x0220,
    VAR_LINE_LED_BRIGHTNESS           = 0x0221,
    VAR_LINE_TURN_GAIN_1              = 0x0222,
    VAR_LINE_TURN_GAIN_2              = 0x0223,
    CMD_LINE_TOGGLE_DISPLAY           = 0x0224,
    CMD_LINE_TOGGLE_LIVE              = 0x0225,
    VAR_LINE_OFFSET_DECAY             = 0x0226,
    VAR_LINE_AVERAGE_DECAY            = 0x0227,
    VAR_LINE_ANALOG_WIDTH             = 0x0228,
    VAR_LINE_OFFSET_THRESHOLD         = 0x0229,
    VAR_LINE_LEFT_THRESHOLD           = 0x022A,
    VAR_LINE_RIGHT_THRESHOLD          = 0x022B,
    VAR_LINE_CENTER_TIMEOUT           = 0x022C,
    /*  Drive variables               = 0x0240,*/
    VAR_DRIVE0_PWM_WRAP_VALUE         = 0x0240,
    VAR_DRIVE0_PWM_CLK_DIV            = 0x0241,
    VAR_DRIVE0_BIAS                   = 0x0242,
    VAR_DRIVE0_DEADZONE               = 0x0243,
    VAR_DRIVE1_PWM_WRAP_VALUE         = 0x0244,
    VAR_DRIVE1_PWM_CLK_DIV            = 0x0245,
    VAR_DRIVE1_BIAS                   = 0x0246,
    VAR_DRIVE1_DEADZONE               = 0x0247,
    /*  PID0 variables                = 0x0260,*/
    VAR_PID0_GAIN                     = 0x0260,
    VAR_PID0_INTEGRAL_TIME            = 0x0261,
    VAR_PID0_INTEGRAL_LIMIT           = 0x0262,
    VAR_PID0_INTEGRAL_RATE_LIMIT      = 0x0263,
    VAR_PID0_DERIVATIVE_TIME          = 0x0264,
    VAR_PID0_DERIVATIVE_CUTOFF        = 0x0265,
    VAR_PID0_DEADZONE                 = 0x0266,
    /*  PID1 variables                = 0x0260,*/
    VAR_PID1_GAIN                     = 0x0280,
    VAR_PID1_INTEGRAL_TIME            = 0x0281,
    VAR_PID1_INTEGRAL_LIMIT           = 0x0282,
    VAR_PID1_INTEGRAL_RATE_LIMIT      = 0x0283,
    VAR_PID1_DERIVATIVE_TIME          = 0x0284,
    VAR_PID1_DERIVATIVE_CUTOFF        = 0x0285,
    VAR_PID1_DEADZONE                 = 0x0286,

    CMD_FIRST                         = CMD_NONE,
    CMD_LAST                          = CMD_WAKEUP,
    VAR_FIRST                         = VAR_SPEED,
    VAR_LAST                          = VAR_PID0_DEADZONE,
    INTERFACE_ENUM_FIRST              = CMD_FIRST,
    INTERFACE_ENUM_LAST               = VAR_LAST + 1,
    INTERFACE_ENUM_SIZE               = VAR_LAST
};

std::tuple<int, float> GetVariable(int _Enum)
{
    switch(_Enum)
    {
    /*Commands variables                    */
    case CMD_START:                         return {INTERFACE_GET_OK, LFSYS.Start};
    case CMD_STOP:                          return {INTERFACE_GET_OK, LFSYS.Stop};
    case CMD_SLEEP:                         return {INTERFACE_GET_OK, LFSYS.Sleep};
    case CMD_WAKEUP:                        return {INTERFACE_GET_OK, !LFSYS.Sleep};
    case CMD_LINE_CALIBRATE:                return {INTERFACE_GET_OK, LineSensor0.IsCalibrated()};
    case CMD_IMU_CALIBRATE:                 return {INTERFACE_GET_OK, IMU0.IsCalibrated()};
    case CMD_NVM_ERASE:                     return {INTERFACE_GET_OK, LFSYS.NVM_Erase_OK};
    case CMD_NVM_LOAD:                      return {INTERFACE_GET_OK, LFSYS.NVM_ReLoad_OK};
    case CMD_NVM_SAVE:                      return {INTERFACE_GET_OK, LFSYS.NVM_Save_OK};
    case CMD_NVM_RELOAD:                    return {INTERFACE_GET_OK, LFSYS.NVM_ReLoad_OK};
    case CMD_ESC_START:                     return {INTERFACE_GET_OK, LFSYS.ESC_Start};
    case CMD_ESC_STOP:                      return {INTERFACE_GET_OK, !LFSYS.ESC_Start};
    /*Common variables                      */
    case VAR_SPEED:                         return {INTERFACE_GET_OK, LFSYS.Config.M_Speed};
    case VAR_ESC_SPEED:                     return {INTERFACE_GET_OK, LFSYS.Config.ESC_Speed};
    case VAR_KP:                            return {INTERFACE_GET_OK, LFSYS.Config.Kp};
    case VAR_KD:                            return {INTERFACE_GET_OK, LFSYS.Config.Kd};
    case VAR_MAX_SPEED:                     return {INTERFACE_GET_OK, LFSYS.Config.Max_Speed};
    case VAR_RAMP_SPEED:                    return {INTERFACE_GET_OK, LFSYS.Config.Ramp_Speed};
    case VAR_RAMP_SPEED_DOWN:               return {INTERFACE_GET_OK, LFSYS.Config.Ramp_SpeedDown};
    case VAR_WALL_SPEED:                    return {INTERFACE_GET_OK, LFSYS.Config.Wall_Speed};
    case VAR_WALL_ANGLE:                    return {INTERFACE_GET_OK, LFSYS.Config.Wall_Angle};
    case VAR_WALL_TH:                       return {INTERFACE_GET_OK, LFSYS.Config.Wall_Th};
    case VAR_WALL_TIME1:                    return {INTERFACE_GET_OK, LFSYS.Config.Wall_time1};
    case VAR_WALL_TIME2:                    return {INTERFACE_GET_OK, LFSYS.Config.Wall_time2};
    case VAR_DISTANCE:                      return {INTERFACE_GET_OK, DistanceSensor0.GetDistance()};
    /*Driver variables                      */
    /*Line variables                        */
    case CMD_LINE_TOGGLE_DISPLAY:           return {INTERFACE_GET_OK, LineSensor0.GetDisplayAnalog()};
    // case CMD_LINE_TOGGLE_LIVE:              return {INTERFACE_GET_OK, LineSensor.getLiveUpdate()};
    // case VAR_LINE_OFFSET_DECAY:             return {INTERFACE_GET_OK, LineSensor.getOffsetDecayCoef()};
    // case VAR_LINE_AVERAGE_DECAY:            return {INTERFACE_GET_OK, LineSensor.getAverageDecayCoef()};
    // case VAR_LINE_ANALOG_WIDTH:             return {INTERFACE_GET_OK, LineSensor.getAnalogWidth()};
    // case VAR_LINE_OFFSET_THRESHOLD:         return {INTERFACE_GET_OK, LineSensor.getOffsetThreshold()};
    // case VAR_LINE_LEFT_THRESHOLD:           return {INTERFACE_GET_OK, LineSensor.getLeftThreshold()};
    // case VAR_LINE_RIGHT_THRESHOLD:          return {INTERFACE_GET_OK, LineSensor.getRightThreshold()};
    // case VAR_LINE_CENTER_TIMEOUT:           return {INTERFACE_GET_OK, LineSensor.getCenterTimeout()};
    /*PID0 variables                        */
    case VAR_PID0_GAIN:                     return {INTERFACE_GET_OK, 0};
    case VAR_PID0_INTEGRAL_TIME:            return {INTERFACE_GET_OK, 0};
    case VAR_PID0_INTEGRAL_LIMIT:           return {INTERFACE_GET_OK, 0};
    case VAR_PID0_INTEGRAL_RATE_LIMIT:      return {INTERFACE_GET_OK, 0};
    case VAR_PID0_DERIVATIVE_TIME:          return {INTERFACE_GET_OK, 0};
    case VAR_PID0_DERIVATIVE_CUTOFF:        return {INTERFACE_GET_OK, 0};
    case VAR_PID0_DEADZONE:                 return {INTERFACE_GET_OK, 0};

    case INTERFACE_ENUM_LAST:               return {INTERFACE_GET_EOF, 0.0f};
    default:                                return {INTERFACE_GET_NOVAL, 0.0f};
    }
}

int SetVariable(int _Enum, float _Value)
{
    switch(_Enum)
    {
    /*Commands variables                                                                                                                         */
    case CMD_START:                             Start();                                                            return INTERFACE_SET_OK;
    case CMD_STOP:                              Stop();                                                             return INTERFACE_SET_OK;
    case CMD_SLEEP:                             Sleep();                                                            return INTERFACE_SET_OK;
    case CMD_WAKEUP:                            Wakeup();                                                           return INTERFACE_SET_OK;
    case CMD_LINE_CALIBRATE:                    Wakeup(); LineSensor0.StartCalibration(); LED0.Set(100,400);        return INTERFACE_SET_OK;
    case CMD_IMU_CALIBRATE:                     Wakeup(); IMU0.StartCalibration();        LED0.Set(100,400);        return INTERFACE_SET_OK;
    case CMD_NVM_ERASE:                         LFSYS.NVM_Erase_OK  = false; EraseConfig();                         return INTERFACE_SET_OK;
    case CMD_NVM_LOAD:                          LFSYS.NVM_ReLoad_OK = false; ReloadConfig();                        return INTERFACE_SET_OK;
    case CMD_NVM_SAVE:                          LFSYS.NVM_Save_OK   = false; SaveConfig();                          return INTERFACE_SET_OK;
    case CMD_NVM_RELOAD:                        LFSYS.NVM_ReLoad_OK = false; ReloadConfig();                        return INTERFACE_SET_OK;
    case CMD_ESC_START:                         ESC_Start();                                                        return INTERFACE_SET_OK;
    case CMD_ESC_STOP:                          ESC_Stop();                                                         return INTERFACE_SET_OK;
    /*Common variables                                                                                                                           */
    case VAR_SPEED:                             LFSYS.Config.M_Speed   = _Value;                                    return INTERFACE_SET_OK;
    case VAR_ESC_SPEED:                         LFSYS.Config.ESC_Speed = _Value;                                    return INTERFACE_SET_OK;
    case VAR_KP:                                LFSYS.Config.Kp = _Value;                                           return INTERFACE_SET_OK;
    case VAR_KD:                                LFSYS.Config.Kd = _Value;                                           return INTERFACE_SET_OK;
    case VAR_MAX_SPEED:                         LFSYS.Config.Max_Speed = _Value;                                    return INTERFACE_SET_OK;
    case VAR_RAMP_SPEED:                        LFSYS.Config.Ramp_Speed = _Value;                                   return INTERFACE_SET_OK;
    case VAR_RAMP_SPEED_DOWN:                   LFSYS.Config.Ramp_SpeedDown = _Value;                               return INTERFACE_SET_OK;
    case VAR_WALL_SPEED:                        LFSYS.Config.Wall_Speed = _Value;                                   return INTERFACE_SET_OK;
    case VAR_WALL_ANGLE:                        LFSYS.Config.Wall_Angle = _Value;                                   return INTERFACE_SET_OK;
    case VAR_WALL_TH:                           LFSYS.Config.Wall_Th = _Value;                                      return INTERFACE_SET_OK;
    case VAR_WALL_TIME1:                        LFSYS.Config.Wall_time1 = _Value;                                   return INTERFACE_SET_OK;
    case VAR_WALL_TIME2:                        LFSYS.Config.Wall_time2 = _Value;                                   return INTERFACE_SET_OK;
    case VAR_DISTANCE:                                                                                              return INTERFACE_SET_OK;
    /*Driver variables                                                                                                                           */
    /*Line variables                                                                                                                             */
    case CMD_LINE_TOGGLE_DISPLAY:               LineSensor0.SetDisplayAnalog(_Value);                               return INTERFACE_SET_OK;
    // case CMD_LINE_TOGGLE_LIVE:              LineSensor.setLiveUpdate(!LineSensor.getLiveUpdate());                                          return INTERFACE_SET_OK;
    // case VAR_LINE_OFFSET_DECAY:             LineSensor.setOffsetDecayCoef(Config.LineSensor.OffsetDecayCoef = _Value);                      return INTERFACE_SET_OK;
    // case VAR_LINE_AVERAGE_DECAY:            LineSensor.setAverageDecayCoef(Config.LineSensor.AverageDecayCoef = _Value);                    return INTERFACE_SET_OK;
    // case VAR_LINE_ANALOG_WIDTH:             LineSensor.setAnalogWidth(Config.LineSensor.AnalogWidth = _Value);                              return INTERFACE_SET_OK;
    // case VAR_LINE_OFFSET_THRESHOLD:         LineSensor.setOffsetThreshold(Config.LineSensor.OffsetThreshold = _Value);                      return INTERFACE_SET_OK;
    // case VAR_LINE_LEFT_THRESHOLD:           LineSensor.setLeftThreshold(Config.LineSensor.LeftThreshold = _Value);                          return INTERFACE_SET_OK;
    // case VAR_LINE_RIGHT_THRESHOLD:          LineSensor.setRightThreshold(Config.LineSensor.RightThreshold = _Value);                        return INTERFACE_SET_OK;
    // case VAR_LINE_CENTER_TIMEOUT:           LineSensor.setCenterTimeout(Config.LineSensor.CenterTimeout = _Value);                          return INTERFACE_SET_OK;
    /*PID0 variables                                                                                                                             */
    case VAR_PID0_GAIN:                                 return INTERFACE_SET_OK;
    case VAR_PID0_INTEGRAL_TIME:                        return INTERFACE_SET_OK;
    case VAR_PID0_INTEGRAL_LIMIT:                       return INTERFACE_SET_OK;
    case VAR_PID0_INTEGRAL_RATE_LIMIT:                  return INTERFACE_SET_OK;
    case VAR_PID0_DERIVATIVE_TIME:                      return INTERFACE_SET_OK;
    case VAR_PID0_DERIVATIVE_CUTOFF:                    return INTERFACE_SET_OK;
    case VAR_PID0_DEADZONE:                             return INTERFACE_SET_OK;
    default:                                            return INTERFACE_SET_NOVAL;
    }
}

void SaveConfig(void)
{
    Config0.MotorDriverA = MotorDriverA.GetConfig();
    Config0.MotorDriverB = MotorDriverB.GetConfig();
    Config0.EncoderA     = EncoderA.GetConfig();
    Config0.EncoderB     = EncoderB.GetConfig();
    Config0.ESC0         = ESC0.GetConfig();
    Config0.IMU0         = IMU0.GetConfig();
    Config0.LineSensor0  = LineSensor0.GetConfig();
    Config0.LFCFG        = LFSYS.Config;
    

    Config0.LockCode = NVM_CONFG_LOCK_CODE;
    NVM.program();

    printf("save ok\n");

    LFSYS.NVM_Erase_OK  = false;
    LFSYS.NVM_Save_OK   = true;
}

void LoadConfig(void)
{
    NVM.load();

    if (Config0.LockCode != NVM_CONFG_LOCK_CODE) {
        printf("ld default\n");
        Config0 = DefaultConfig;
    }
    else
    {
        printf("ld ok\n");
    }

    LFSYS.NVM_Load_OK   = true;
    LFSYS.NVM_ReLoad_OK = false;
    LFSYS.NVM_Save_OK   = false;
}

void ReloadConfig(void)
{
    NVM.load();

    if (Config0.LockCode != NVM_CONFG_LOCK_CODE)
    {
        printf("rld default\n");
        Config0 = DefaultConfig;
    }
    else
    {
        printf("rld ok\n");
    }

    MotorDriverA.LoadConfig(Config0.MotorDriverA);
    MotorDriverB.LoadConfig(Config0.MotorDriverB);
    EncoderA.LoadConfig(Config0.EncoderA);
    EncoderB.LoadConfig(Config0.EncoderB);
    ESC0.LoadConfig(Config0.ESC0);
    IMU0.LoadConfig(Config0.IMU0);
    LineSensor0.LoadConfig(Config0.LineSensor0);
    LFSYS.Config = Config0.LFCFG;

    LFSYS.NVM_Load_OK   = false;
    LFSYS.NVM_ReLoad_OK = true;
    LFSYS.NVM_Save_OK   = false;
}

void EraseConfig(void)
{
    printf("erase ok\n");
    NVM.erase();

    Config0 = DefaultConfig;
    
    LFSYS.NVM_Erase_OK  = false;
    LFSYS.NVM_Load_OK   = false;
    LFSYS.NVM_ReLoad_OK = false;
    LFSYS.NVM_Save_OK   = false;
}
