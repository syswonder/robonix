/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "example.h"

int main(int argc, char *argv[])
{

    // Initialize
    UnitreeLidarReader *lreader = createUnitreeLidarReader();

    std::string lidar_ip = "192.168.123.110";
    std::string local_ip = "192.168.123.120";

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

    sleep(1);

    // Set lidar work mode
    std::cout << "set Lidar to serial mode" << std::endl;
    lreader->setLidarWorkMode(8);
    std::cout << "done" << std::endl;
    sleep(1);
    

    return 0;
}