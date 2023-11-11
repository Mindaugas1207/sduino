
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
    CMD_LINE_SLEEP                    = 0x0008,
    CMD_LINE_WAKEUP                   = 0x0009,
    CMD_ESC_ARM                       = 0x000A,
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
    CMD_LAST                          = CMD_LINE_WAKEUP,
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
    case CMD_START:                         return {INTERFACE_GET_OK, 0};
    case CMD_STOP:                          return {INTERFACE_GET_OK, 0};
    //case CMD_LINE_CALIBRATE:                return {INTERFACE_GET_OK, LineSensor.isCalibrated()};
    case CMD_IMU_CALIBRATE:                 return {INTERFACE_GET_OK, IMU0.IsCalibrated()};
    case CMD_NVM_ERASE:                     return {INTERFACE_GET_OK, 0};
    case CMD_NVM_LOAD:                      return {INTERFACE_GET_OK, 0};
    case CMD_NVM_SAVE:                      return {INTERFACE_GET_OK, 0};
    case CMD_LINE_SLEEP:                    return {INTERFACE_GET_OK, 0};
    case CMD_LINE_WAKEUP:                   return {INTERFACE_GET_OK, 0};
    case CMD_ESC_START:                     return {INTERFACE_GET_OK, 0};
    case CMD_ESC_STOP:                      return {INTERFACE_GET_OK, 0};
    case VAR_BAT_VOLTAGE:                   return {INTERFACE_GET_OK, 0};
    /*Common variables                      */
    case VAR_SPEED:                         return {INTERFACE_GET_OK, 0};
    case VAR_ESC_SPEED:                     return {INTERFACE_GET_OK, 0};
    case VAR_STEERING_GAIN:                 return {INTERFACE_GET_OK, 0};
    case VAR_STEERING_GAIN2:                return {INTERFACE_GET_OK, 0};
    case VAR_STEERING_GAIN3:                return {INTERFACE_GET_OK, 0};
    case VAR_STEERING_GAIN4:                return {INTERFACE_GET_OK, 0};
    case VAR_STEERING_GAIN5:                return {INTERFACE_GET_OK, 0};
    case VAR_STEERING_GAIN6:                return {INTERFACE_GET_OK, 0};
    case VAR_CRUISE_GAIN:                   return {INTERFACE_GET_OK, 0};
    case VAR_CRUISE_GAIN2:                  return {INTERFACE_GET_OK, 0};
    case VAR_CRUISE_GAIN3:                  return {INTERFACE_GET_OK, 0};
    case VAR_CRUISE_GAIN4:                  return {INTERFACE_GET_OK, 0};
    case VAR_CRUISE_GAIN5:                  return {INTERFACE_GET_OK, 0};
    case VAR_CRUISE_GAIN6:                  return {INTERFACE_GET_OK, 0};
    case VAR_BRAKE_TIMEOUT:                 return {INTERFACE_GET_OK, 0};
    /*Driver variables                      */
    // case VAR_DRIVE0_PWM_WRAP_VALUE:         return {INTERFACE_GET_OK, DriveA.getPWM_WRAP_VALUE()};
    // case VAR_DRIVE0_PWM_CLK_DIV:            return {INTERFACE_GET_OK, DriveA.getPWM_CLK_DIV()};
    // case VAR_DRIVE0_BIAS:                   return {INTERFACE_GET_OK, DriveA.getBias()};
    // case VAR_DRIVE0_DEADZONE:               return {INTERFACE_GET_OK, DriveA.getDeadZone()};
    // case VAR_DRIVE1_PWM_WRAP_VALUE:         return {INTERFACE_GET_OK, DriveB.getPWM_WRAP_VALUE()};
    // case VAR_DRIVE1_PWM_CLK_DIV:            return {INTERFACE_GET_OK, DriveB.getPWM_CLK_DIV()};
    // case VAR_DRIVE1_BIAS:                   return {INTERFACE_GET_OK, DriveB.getBias()};
    // case VAR_DRIVE1_DEADZONE:               return {INTERFACE_GET_OK, DriveB.getDeadZone()};
    /*Line variables                        */
    // case VAR_LINE_EMITTER_POWER:            return {INTERFACE_GET_OK, LineSensor.getEmittersPower()};
    // case VAR_LINE_LED_BRIGHTNESS:           return {INTERFACE_GET_OK, LineSensor.getLedsBrightness()};
    // case VAR_LINE_TURN_GAIN_1:              return {INTERFACE_GET_OK, LineSensor.getTurnGain1()};
    // case VAR_LINE_TURN_GAIN_2:              return {INTERFACE_GET_OK, LineSensor.getTurnGain2()};
    // case CMD_LINE_TOGGLE_DISPLAY:           return {INTERFACE_GET_OK, LineSensor.getDisplayAnalog()};
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
    /*PID1 variables                        */
    case VAR_PID1_GAIN:                     return {INTERFACE_GET_OK, 0};
    case VAR_PID1_INTEGRAL_TIME:            return {INTERFACE_GET_OK, 0};
    case VAR_PID1_INTEGRAL_LIMIT:           return {INTERFACE_GET_OK, 0};
    case VAR_PID1_INTEGRAL_RATE_LIMIT:      return {INTERFACE_GET_OK, 0};
    case VAR_PID1_DERIVATIVE_TIME:          return {INTERFACE_GET_OK, 0};
    case VAR_PID1_DERIVATIVE_CUTOFF:        return {INTERFACE_GET_OK, 0};
    case VAR_PID1_DEADZONE:                 return {INTERFACE_GET_OK, 0};

    case INTERFACE_ENUM_LAST:               return {INTERFACE_GET_EOF, 0.0f};
    default:                                return {INTERFACE_GET_NOVAL, 0.0f};
    }
}

int SetVariable(int _Enum, float _Value)
{
    switch(_Enum)
    {
    /*Commands variables                                                                                                                         */
    case CMD_START:                                                                                                                return INTERFACE_SET_OK;
    case CMD_STOP:                                                                                                                 return INTERFACE_SET_OK;
    //case CMD_LINE_CALIBRATE:                wakeup(); LineSensor.startCalibration(); LED0.start(100,400);                                   return INTERFACE_SET_OK;
    case CMD_IMU_CALIBRATE:                                                           return INTERFACE_SET_OK;
    case CMD_NVM_ERASE:                           return INTERFACE_SET_OK;
    case CMD_NVM_LOAD:                            return INTERFACE_SET_OK;
    case CMD_NVM_SAVE:                            return INTERFACE_SET_OK;
    case CMD_LINE_SLEEP:                          return INTERFACE_SET_OK;
    case CMD_LINE_WAKEUP:                         return INTERFACE_SET_OK;
    case CMD_ESC_ARM:                                                                                                                       return INTERFACE_SET_OK;
    case CMD_ESC_START:                                                                                                         return INTERFACE_SET_OK;
    case CMD_ESC_STOP:                                                                                                          return INTERFACE_SET_OK;
    /*Common variables                                                                                                                           */
    case VAR_SPEED:                                                                               return INTERFACE_SET_OK;
    case VAR_ESC_SPEED:                                                                            return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN:                                                                       return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN2:                                                                      return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN3:                                                                      return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN4:                                                                      return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN5:                                                                      return INTERFACE_SET_OK;
    case VAR_STEERING_GAIN6:                                                                      return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN:                                                                         return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN2:                                                                        return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN3:                                                                        return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN4:                                                                        return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN5:                                                                        return INTERFACE_SET_OK;
    case VAR_CRUISE_GAIN6:                                                                        return INTERFACE_SET_OK;
    case VAR_BRAKE_TIMEOUT:                                                                       return INTERFACE_SET_OK;
    /*Driver variables                                                                                                                           */
    // case VAR_DRIVE0_PWM_WRAP_VALUE:         DriveA.setPWM_WRAP_VALUE(Config.DriveA.PWM_WRAP_VALUE = _Value);                                return INTERFACE_SET_OK;
    // case VAR_DRIVE0_PWM_CLK_DIV:            DriveA.setPWM_CLK_DIV(Config.DriveA.PWM_CLK_DIV = _Value);                                      return INTERFACE_SET_OK;
    // case VAR_DRIVE0_BIAS:                   DriveA.setBias(Config.DriveA.Bias = _Value);                                                    return INTERFACE_SET_OK;
    // case VAR_DRIVE0_DEADZONE:               DriveA.setDeadZone(Config.DriveA.DeadZone = _Value);                                            return INTERFACE_SET_OK;
    // case VAR_DRIVE1_PWM_WRAP_VALUE:         DriveB.setPWM_WRAP_VALUE(Config.DriveB.PWM_WRAP_VALUE = _Value);                                return INTERFACE_SET_OK;
    // case VAR_DRIVE1_PWM_CLK_DIV:            DriveB.setPWM_CLK_DIV(Config.DriveB.PWM_CLK_DIV = _Value);                                      return INTERFACE_SET_OK;
    // case VAR_DRIVE1_BIAS:                   DriveB.setBias(Config.DriveB.Bias = _Value);                                                    return INTERFACE_SET_OK;
    // case VAR_DRIVE1_DEADZONE:               DriveB.setDeadZone(Config.DriveB.DeadZone = _Value);                                            return INTERFACE_SET_OK;
    /*Line variables                                                                                                                             */
    // case VAR_LINE_EMITTER_POWER:            LineSensor.setEmittersPower(Config.LineSensor.EmittersPower = _Value);                          return INTERFACE_SET_OK;
    // case VAR_LINE_LED_BRIGHTNESS:           LineSensor.setLedsBrightness(Config.LineSensor.LedBrightness = _Value);                         return INTERFACE_SET_OK;
    // case VAR_LINE_TURN_GAIN_1:              LineSensor.setTurnGain1(Config.LineSensor.TurnGain1 = _Value);                                  return INTERFACE_SET_OK;
    // case VAR_LINE_TURN_GAIN_2:              LineSensor.setTurnGain2(Config.LineSensor.TurnGain2 = _Value);                                  return INTERFACE_SET_OK;
    // case CMD_LINE_TOGGLE_DISPLAY:           LineSensor.setDisplayAnalog(!LineSensor.getDisplayAnalog());                                    return INTERFACE_SET_OK;
    // case CMD_LINE_TOGGLE_LIVE:              LineSensor.setLiveUpdate(!LineSensor.getLiveUpdate());                                          return INTERFACE_SET_OK;
    // case VAR_LINE_OFFSET_DECAY:             LineSensor.setOffsetDecayCoef(Config.LineSensor.OffsetDecayCoef = _Value);                      return INTERFACE_SET_OK;
    // case VAR_LINE_AVERAGE_DECAY:            LineSensor.setAverageDecayCoef(Config.LineSensor.AverageDecayCoef = _Value);                    return INTERFACE_SET_OK;
    // case VAR_LINE_ANALOG_WIDTH:             LineSensor.setAnalogWidth(Config.LineSensor.AnalogWidth = _Value);                              return INTERFACE_SET_OK;
    // case VAR_LINE_OFFSET_THRESHOLD:         LineSensor.setOffsetThreshold(Config.LineSensor.OffsetThreshold = _Value);                      return INTERFACE_SET_OK;
    // case VAR_LINE_LEFT_THRESHOLD:           LineSensor.setLeftThreshold(Config.LineSensor.LeftThreshold = _Value);                          return INTERFACE_SET_OK;
    // case VAR_LINE_RIGHT_THRESHOLD:          LineSensor.setRightThreshold(Config.LineSensor.RightThreshold = _Value);                        return INTERFACE_SET_OK;
    // case VAR_LINE_CENTER_TIMEOUT:           LineSensor.setCenterTimeout(Config.LineSensor.CenterTimeout = _Value);                          return INTERFACE_SET_OK;
    /*PID0 variables                                                                                                                             */
    case VAR_PID0_GAIN:                     
                                                         return INTERFACE_SET_OK;
    case VAR_PID0_INTEGRAL_TIME:            
                                                         return INTERFACE_SET_OK;
    case VAR_PID0_INTEGRAL_LIMIT:           
                                                         return INTERFACE_SET_OK;
    case VAR_PID0_INTEGRAL_RATE_LIMIT:      
                                                         return INTERFACE_SET_OK;
    case VAR_PID0_DERIVATIVE_TIME:          
                                                         return INTERFACE_SET_OK;
    case VAR_PID0_DERIVATIVE_CUTOFF:        
                                                         return INTERFACE_SET_OK;
    case VAR_PID0_DEADZONE:                 
                                                         return INTERFACE_SET_OK;
    /*PID1 variables                                                                                                                             */
    case VAR_PID1_GAIN:                   return INTERFACE_SET_OK;
    case VAR_PID1_INTEGRAL_TIME:          return INTERFACE_SET_OK;
    case VAR_PID1_INTEGRAL_LIMIT:         return INTERFACE_SET_OK;
    case VAR_PID1_INTEGRAL_RATE_LIMIT:    return INTERFACE_SET_OK;
    case VAR_PID1_DERIVATIVE_TIME:        return INTERFACE_SET_OK;
    case VAR_PID1_DERIVATIVE_CUTOFF:      return INTERFACE_SET_OK;
    case VAR_PID1_DEADZONE:               return INTERFACE_SET_OK;
    default:                                                                                                                                return INTERFACE_SET_NOVAL;
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
    

    Config0.LockCode = NVM_CONFG_LOCK_CODE;
    NVM.program();
}

void LoadConfig(void)
{
    NVM.load();

    if (Config0.LockCode != NVM_CONFG_LOCK_CODE) {
        printf("DBG:LOAD DEFAULT\n");
        Config0 = DefaultConfig;
    }
}

void ReloadConfig(void)
{
    NVM.load();

    if (Config0.LockCode != NVM_CONFG_LOCK_CODE)
    {
        Config0 = DefaultConfig;
        return;
    }

    MotorDriverA.LoadConfig(Config0.MotorDriverA);
    MotorDriverB.LoadConfig(Config0.MotorDriverB);
    EncoderA.LoadConfig(Config0.EncoderA);
    EncoderB.LoadConfig(Config0.EncoderB);
    ESC0.LoadConfig(Config0.ESC0);
    IMU0.LoadConfig(Config0.IMU0);
    LineSensor0.LoadConfig(Config0.LineSensor0);
}

void EraseConfig(void)
{
    NVM.erase();

    Config0 = DefaultConfig;
}
