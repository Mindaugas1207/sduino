
#include "main.hpp"
#include "config.hpp"

const SmartControl button_def = { 0,"","","" };

const SmartControl slider_def = { 1,"0","10","0.1" };

const SmartVariable interface_vars[] = 
{
    {"Start",                    [](){ return LFSYS.Start;                   }, [](float value){ Start();                                                        }, &button_def},
    {"Stop",                     [](){ return LFSYS.Stop;                    }, [](float value){ Stop();                                                         }, &button_def},
    {"Sleep",                    [](){ return LFSYS.Sleep;                   }, [](float value){ Sleep();                                                        }, &button_def},
    {"Wakeup",                   [](){ return !LFSYS.Sleep;                  }, [](float value){ Wakeup();                                                       }, &button_def},
    {"Line Calibrate",           [](){ return LineSensor0.IsCalibrated();    }, [](float value){ Wakeup(); LineSensor0.StartCalibration(); LED0.Set(100,400);    }, &button_def},
    {"IMU Calibrate",            [](){ return IMU0.IsCalibrated();           }, [](float value){ Wakeup(); IMU0.StartCalibration();        LED0.Set(100,400);    }, &button_def},
    {"NVM Erase",                [](){ return LFSYS.NVM_Erase_OK;            }, [](float value){ LFSYS.NVM_Erase_OK  = false; EraseConfig();                     }, &button_def},
    {"NVM Load",                 [](){ return LFSYS.NVM_ReLoad_OK;           }, [](float value){ LFSYS.NVM_ReLoad_OK = false; ReloadConfig();                    }, &button_def},
    {"NVM Save",                 [](){ return LFSYS.NVM_Save_OK;             }, [](float value){ LFSYS.NVM_Save_OK   = false; SaveConfig();                      }, &button_def},
    {"NVM Reload",               [](){ return LFSYS.NVM_ReLoad_OK;           }, [](float value){ LFSYS.NVM_ReLoad_OK = false; ReloadConfig();                    }, &button_def},
    {"ESC Start",                [](){ return LFSYS.ESC_Start;               }, [](float value){ ESC_Start();                                                    }, &button_def},
    {"ESC Stop",                 [](){ return !LFSYS.ESC_Start;              }, [](float value){ ESC_Stop();                                                     }, &button_def},
    {"Line Display Analog",      [](){ return LFSYS.LINE_ANALOG;             }, [](float value){ LineSensor0.SetDisplayAnalog(value > 0);                        }, &button_def},
    {"Speed",                    [](){ return LFSYS.Config.M_Speed;          }, [](float value){ LFSYS.Config.M_Speed = value;                                   }, &slider_def},
    {"ESC Speed",                [](){ return LFSYS.Config.ESC_Speed;        }, [](float value){ LFSYS.Config.ESC_Speed = value;                                 }, &slider_def},
    {"KP",                       [](){ return LFSYS.Config.Kp;               }, [](float value){ LFSYS.Config.Kp = value;                                        }, &slider_def},
    {"KD",                       [](){ return LFSYS.Config.Kd;               }, [](float value){ LFSYS.Config.Kd = value;                                        }, &slider_def},
    {"Max Speed",                [](){ return LFSYS.Config.Max_Speed;        }, [](float value){ LFSYS.Config.Max_Speed = value;                                 }, &slider_def},
    {"Loop time",                [](){ return LFSYS.Config.loop_time;        }, [](float value){ LFSYS.Config.loop_time = value;                                 }, &slider_def},
    {"SpeedL",                   [](){ return LFSYS.SpeedL;                  }, [](float value){                                                                 }, &slider_def},
    {"SpeedR",                   [](){ return LFSYS.SpeedR;                  }, [](float value){                                                                 }, &slider_def},
    {"SpeedV",                   [](){ return LFSYS.SpeedV;                  }, [](float value){                                                                 }, &slider_def},
    {"SpeedW",                   [](){ return LFSYS.SpeedW;                  }, [](float value){                                                                 }, &slider_def},
    {"INTERFACE END",            nullptr,                                       nullptr,                                                                            nullptr    }
};
constexpr int interface_variable_count = sizeof(interface_vars) / sizeof(SmartVariable);


int CreateSlider(char* file, uint id, const char* name, const char* min, const char* max, const char* step)
{
    return sprintf(file, "<div class=\"range-wrap\" id=\"%X\">%s"
                         "<input type=\"range\" class=\"range\" min=\"%s\" max=\"%s\" value=\"0\" step=\"%s\">"
                         "<output class=\"bubble\"></output></div>",
                         id, name, min, max, step);
}

int CreateButton(char* file, uint id, const char* name)
{
    return sprintf(file, "<button class=\"button\" id = \"%X\">%s</button>", id, name);
}

int CreateControl(char* file, uint id, const char* name, const SmartControl& control)
{
    CreateSlider(file, id, name, control.min, control.max, control.step);

    if (control.control_type == 0)
    {
        return CreateButton(file, id, name);
    }
    else
    {
        return CreateSlider(file, id, name, control.min, control.max, control.step);
    }
}

int CreateForm(char* file)
{
    int length = 0;

    for (uint i = 0; i < interface_variable_count;i++)
    {
        length += CreateControl(file + length, i, interface_vars[i].name, *(interface_vars[i].control));
    }

    return length;
}

std::tuple<int, float> GetVariable(int _Enum)
{
    if (_Enum >= interface_variable_count) return {INTERFACE_GET_NOVAL, 0.0f};
    if (_Enum == (interface_variable_count - 1)) return {INTERFACE_GET_EOF, 0.0f};
    return {INTERFACE_GET_OK, interface_vars[_Enum].Get()};
}

int SetVariable(int _Enum, float _Value)
{
    if (_Enum >= interface_variable_count) return INTERFACE_GET_NOVAL;
    if (_Enum == (interface_variable_count - 1)) return INTERFACE_GET_EOF;
    interface_vars[_Enum].Set(_Value);
    return INTERFACE_SET_OK;
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
