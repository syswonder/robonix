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

    sleep(1);

    // Set lidar ip address
    LidarIpAddressConfig config;
    config.lidar_ip[0] = 192;
    config.lidar_ip[1] = 168;
    config.lidar_ip[2] = 123;
    config.lidar_ip[3] = 110;

    config.user_ip[0] = 192;
    config.user_ip[1] = 168;
    config.user_ip[2] = 123;
    config.user_ip[3] = 120;

    config.lidar_port = 6101;
    config.user_port = 6201;

    config.gateway[0] = 0;
    config.gateway[1] = 0;
    config.gateway[2] = 0;
    config.gateway[3] = 0;

    config.subnet_mask[0] = 255;
    config.subnet_mask[1] = 255;
    config.subnet_mask[2] = 255;
    config.subnet_mask[3] = 0;

    lreader->setLidarIpAddressConfig(config);
    std::cout << "Lidar IP is reset! Please reboot lidar!\n";
    sleep(1);
    

    return 0;
}