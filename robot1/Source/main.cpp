/**
 * 2022-11-01, Minduagas Mikalauskas.
 */

#include "main.hpp"
#include "line_follower.hpp"

LineFollower_s LineFollower;

int main()
{
    LineFollower.init();

    while (true)
    {
        LineFollower.run();
    }
    
    return 0;
}
