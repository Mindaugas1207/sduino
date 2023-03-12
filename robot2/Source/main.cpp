/**
 * 2022-11-01, Minduagas Mikalauskas.
 */

#include "main.hpp"
#include "mini_sumo.hpp"

mini_sumo_s mini_sumo;

int main()
{
    mini_sumo.init();
    
    while (true)
    {
        mini_sumo.run();
    }
    
    return 0;
}
