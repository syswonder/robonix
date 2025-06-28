/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "example.h"

int main(int argc, char *argv[])
{

    // Initialize
    UnitreeLidarReader *lreader = createUnitreeLidarReader();

    std::string lidar_ip = "192.168.1.62";
    std::string local_ip = "192.168.1.2";

    unsigned short lidar_port = 6101;
    unsigned short local_port = 6201;

    if (lreader->initializeUDP(lidar_port, lidar_ip, local_port, local_ip))
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
    uint32_t workMode = 0;
    std::cout << "set Lidar work mode to: " << workMode << std::endl;
    lreader->setLidarWorkMode(workMode);
    sleep(1);
    
    // Reset Lidar
    lreader->resetLidar();
    sleep(1);

    // Process
    exampleProcess(lreader);
    
    return 0;
}