
// #ifndef INC_LINE_FOLLOWER_HPP_
// #define INC_LINE_FOLLOWER_HPP_



// class LineFollower_s
// {
    // bool NVM_CONFIG_OK = false;
    // bool Start = false;
    // bool Stop = false;
    // bool Awake = false;
    // bool NVM_LOAD_OK = false;
    // bool NVM_SAVE_OK = false;
    // bool NVM_ERASE_OK = false;
    // bool Start_ESC = false;
    // bool Stop_ESC = false;
    // bool SleepAllowed = false;

    // int iX = 0;
    // int iY = 0;

    // float iX_f = 0.0f;
    // float iY_f = 0.0f;
    // float iYaw = 0.0f;

    // int iPulseR = 0;
    // int iPulseL = 0;
    // float iSpeedR = 0.0f;
    // float iSpeedL = 0.0f;
    // bool ReturningOnLine = false;
    // bool BrakeStart = false;
    // bool BrakeEnd = false;
//public:

    //std::tuple<int, float> get(int _Enum);
    //int set(int _Enum, float _Value);

    //const std::function<std::tuple<int, float>(int)> f_getCallback = [this](int _Enum) { return this->get(_Enum); };
    //const std::function<int(int, float)> f_setCallback = [this](int _Enum, float _Value) { return this->set(_Enum, _Value); };
    // float ph = 0, LastFrameHeading = 0, pathHeading = 0;
    // bool doManuver = false;
    // int manuverStep = 0;
    // absolute_time_t PathTime;
    // absolute_time_t vlxTime;
    // bool vlx_start = false;
    // long step_counterA = 0;
    // long step_counterB = 0;
    // bool manuver_done = false;
    // #define xs(x) __XSTRING(x)

    // #define STRINGIFY_IMPL(s) #s
    // #define STRINGIFY(s) STRINGIFY_IMPL(s)
    // #define ARG1_IMPL(a, ...) a
    // #define ARG1(...) ARG1_IMPL(__VA_ARGS__, 0)
    // #define DumpStr(...) DumpString(STRINGIFY(ARG1(__VA_ARGS__)),__VA_ARGS__)
    
    // void DumpString(const char* varname, char* var, int optionalvar=0) {
    //     printf("%s : '%s'\n", varname, var);
    //     printf("blah: %d", optionalvar);
    // }

    // int rangeAdd(char * File, const char * VarName, uint VarId, const char * Min, const char * Max, const char * Step)
    // {
    //     return sprintf(File, "<div class=\"range-wrap\" id=\"%X\">"
    //                          "<span class =\"label\">%s</span>"
	// 	                     "<input type=\"range\" class=\"range\" min=\"%s\" max=\"%s\" value=\"0\" step=\"%s\">"
	// 	                     "<output class=\"bubble\">"
    //                          "</output></div>",
    //                          VarId, VarName, Min, Max, Step);
    // }

    // #define RANGE_ADD(File, VarName, Min, Max, Step) sprintf(File, "<div class=\"range-wrap\" id=\"%X\">"\
    //                                                                "<span class =\"label\">" xs(VarName) "</span>"\
    //                                                                "<input type=\"range\" class=\"range\" min=\"" xs(Min) "\" max=\"" xs(Max) "\" value=\"0\" step=\"" xs(Step) "\">"\
    //                                                                "<output class=\"bubble\">"\
    //                                                                "</output></div>", VarName)

    //#define RANGE_STRING()

    // int tabBegin(char * File, const char * TabName)
    // {
    //     return sprintf(File, "<div id=\"%s\" class=\"tabcontent\">", TabName);
    // }

    // int tabEnd(char * File)
    // {
    //     return sprintf(File, "</div>");
    // }

    // void interfaceFileConstruct()
    // {
    //     char buffer[255];
    //     //tabBegin(buffer, "PID0");
    //     RANGE_ADD(buffer, 0, 0, 1, 0.001);
    //     //rangeAdd(buffer, __XSTRING(VAR_PID0_SETPOINT), VAR_PID0_SETPOINT, "0", "1", "0.001");
    //     tabEnd(buffer);
    // }
    // constexpr auto ccatc(std::string, const char s2[])
    // {
    //     return cat;
    // }

    
    
    // void fileNewTabBegin(char file[], char TabName[])
    // {
    //     constexpr const char ff[] = "help";
    //     constexpr char stc[100];
    //     stc += ff;

    //     int n = sprintf(file, "<div id=\"%s\" class=\"tabcontent\">", TabName);
    // }
// };

// #endif

