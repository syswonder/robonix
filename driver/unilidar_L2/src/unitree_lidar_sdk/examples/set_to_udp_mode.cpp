/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "example.h"

int main(int argc, char *argv[])
{

    // Initialize
    UnitreeLidarReader *lreader = createUnitreeLidarReader();

    std::string port = "/dev/ttyACM0";
    uint32_t baudrate = 4000000;

    if (lreader->initializeSerial(port, baudrate))
    {
        printf("Unilidar initialization failed! Exit here!\n");
        exit(-1);
    }
    else
    {
        printf("Unilidar initialization succeed!\n");
    }

    lreader->startLidarRotation();
    sleep(1);
    
    // Set lidar work mode
    std::cout << "set Lidar to udp mode" << std::endl;
    lreader->setLidarWorkMode(0);
    std::cout << "done" << std::endl;
    sleep(1);
    
    return 0;
}