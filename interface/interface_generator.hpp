#ifndef INC_INTERFACE_GENERATOR_HPP_
#define INC_INTERFACE_GENERATOR_HPP_

#include "stdio.h"
#include "string"
#include "vector"
#include "pico/stdlib.h"

// enum if_controls
// {
//     BUTTON,
//     RANGE
// };

// enum if_controlAllign
// {
//     Left,
//     Right
// };

// struct if_rangeButton
// {
//     const char* Value;
//     const if_controlAllign Align;
// };

// struct if_control
// {
//     const char* Name;
//     const char* Id;
//     double Min;
//     double Max;
//     double Step;
//     const if_controls Type;
//     //const std::vector<if_rangeButton> Buttons;
// };

// struct if_tab
// {
//     const char* Name;
//     const std::vector<if_control> Controls;
// };

// static if_tab tabs = {
//     .Name = "TAB1",
//     .Controls = {
//         {
//             .Name = "1",
//             .Id   = "000"
//         }
//     }
// };

// enum IntfCommands
// {
//     INTF_CMD_START = 0,

//     CMD_0          = 0 + INTF_CMD_START,
//     CMD_1          = 1 + INTF_CMD_START,
//     CMD_2          = 2 + INTF_CMD_START,
//     INTF_CMD_LAST  = CMD_2,

//     INTF_VAR_START = INTF_CMD_LAST + 1,

//     VAR_0          = 0 + INTF_VAR_START,
//     VAR_1          = 1 + INTF_VAR_START,
//     VAR_2          = 2 + INTF_VAR_START,
//     INTF_VAR_LAST  = VAR_2,
    
//     INTF_FIRST     = INTF_CMD_START,
//     INTF_LAST      = INTF_VAR_LAST,
//     INTF_SIZE      = INTF_VAR_LAST + 1
// };


// inline void generateRange(const if_control& ctrl)
// {
//     if (ctrl.Type != RANGE) return;

//     printf("<div class=\"range-wrap\" id=\"%s\">\n", ctrl.Id);
//     printf("<div class=\"column left\">");
//     for (uint i = 0; i < ctrl.Buttons)

    
// 		<div class="column left"><button class="btnpm">-0.1</button><button class="btnpm">-0.01</button><button class="btnpm">-0.001</button></div>
// 		<div class="column middle"><span class ="label">SPEED</span></div>
// 		<div class="column right"><button class="btnpm">+0.001</button><button class="btnpm">+0.01</button><button class="btnpm">+0.1</button></div>
// 		<input type="range" class="range" min="-1" max="1" value="0" step="0.001">
// 		<output class="bubble"></output>
// 	</div>

// }

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
//                             "<span class =\"label\">%s</span>"
//                             "<input type=\"range\" class=\"range\" min=\"%s\" max=\"%s\" value=\"0\" step=\"%s\">"
//                             "<output class=\"bubble\">"
//                             "</output></div>",
//                             VarId, VarName, Min, Max, Step);
// }

// #define RANGE_ADD(File, VarName, Min, Max, Step) sprintf(File, "<div class=\"range-wrap\" id=\"%X\">"\
//                                                                 "<span class =\"label\">" xs(VarName) "</span>"\
//                                                                 "<input type=\"range\" class=\"range\" min=\"" xs(Min) "\" max=\"" xs(Max) "\" value=\"0\" step=\"" xs(Step) "\">"\
//                                                                 "<output class=\"bubble\">"\
//                                                                 "</output></div>", VarName)

// #define RANGE_STRING()

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



// void fileNewTabBegin(char file[], char TabName[])
// {
//     constexpr const char ff[] = "help";
//     //constexpr char stc[100];
//     //stc += ff;

//     int n = sprintf(file, "<div id=\"%s\" class=\"tabcontent\">", TabName);
// }

#endif

