#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>

#include "monitor/scout_monitor.hpp"

using namespace wescore;

int main(int argc, char **argv)
{
    std::string device_name;

    if (argc == 2)
    {
        device_name = {argv[1]};
        std::cout << "Specified device: " << device_name << std::endl;
    }
    else
    {
        std::cout << "Usage: app_scout_monitor <interface>" << std::endl
                  << "Example: ./app_scout_monitor can1" << std::endl;
        return -1;
    }

    ScoutMonitor monitor;
    monitor.Run(device_name);

    return 0;
}