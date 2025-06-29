/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once
#include "unitree_lidar_utilities.h"

namespace unilidar_sdk2{

/**
 * @brief Unitree Lidar Reader
 */
class UnitreeLidarReader
{

public:
    /**
     * @brief Initialize
     * @return Return 0 if the serial port is opened successfully;
     * return -1 if we failed to open the serial port.
     */
    virtual int initializeSerial(
        std::string port = "/dev/ttyACM0",
        uint32_t baudrate = 4000000,
        uint16_t cloud_scan_num = 18,
        bool use_system_timestamp = true,
        float range_min = 0,
        float range_max = 100
    ) = 0;

    /**
     * @brief Initialize for UDP board
     */
    virtual int initializeUDP(
        unsigned short lidar_port = 6101,
        std::string lidar_ip = "192.168.1.62",
        unsigned short local_port = 6201,
        std::string local_ip = "192.168.1.2",
        uint16_t cloud_scan_num = 18,
        bool use_system_timestamp = true,
        float range_min = 0,
        float range_max = 100
    ) = 0;

    /**
     * @brief Close Serial connection
     */
    virtual bool closeSerial() = 0;

    /**
     * @brief Close UDP connection
     */
    virtual bool closeUDP() = 0;

    /**
     * @brief Try to parse a message from the serial buffer once.
     * @note This is the main entrance of this class
     * @return the data packet type id, eg.
     * 0 if no valid message parsed
     * LIDAR_POINT_DATA_PACKET_TYPE 102
     * LIDAR_2D_POINT_DATA_PACKET_TYPE 103
     * LIDAR_IMU_DATA_PACKET_TYPE 104
     * LIDAR_VERSION_PACKET_TYPE 105
     */
    virtual int runParse() = 0;

    /**
     * @brief Clear buffer
     */
    virtual void clearBuffer() = 0;

    virtual const LidarPointDataPacket &getLidarPointDataPacket() const = 0;

    virtual const Lidar2DPointDataPacket &getLidar2DPointDataPacket() const = 0;

    virtual const LidarImuDataPacket &getLidarImuDataPacket() const = 0;

    virtual const LidarVersionDataPacket &getLidarVersionDataPacket() const = 0;

    /**
     * @brief Get parsed point cloud data
     */
    virtual bool getPointCloud(PointCloudUnitree &cloud) const = 0;

    /**
     * @brief Get IMU data
     */
    virtual bool getImuData(LidarImuData &imu) const = 0;

    /**
     * @brief Get the Version of Lidar SDK
     */
    virtual bool getVersionOfSDK(std::string &version) const = 0;

    /**
     * @brief Get Version of lidar firmware
     */
    virtual bool getVersionOfLidarFirmware(std::string &version) const = 0;

    /**
     * @brief Get Version of lidar hardware
     */
    virtual bool getVersionOfLidarHardware(std::string &version) const = 0;

    /**
     * @brief Get time delay of one-way data transmission in second.
     */
    virtual bool getTimeDelay(double &delay) const = 0;

    /**
     * @brief Get the removal percentage of points due to the dirt on the protection cover
     * @return the percentage of removed dirty points
     */
    virtual bool getDirtyPercentage(float &percentage) const = 0;

    /**
     * @brief Send a user control command to lidar
     * @param cmd 
     */
    virtual void sendUserCtrlCmd(LidarUserCtrlCmd cmd)= 0;

    /**
     * @brief Set lidar work mode
     */
    virtual void setLidarWorkMode(uint32_t mode) = 0;

    /**
     * @brief Sync lidar timestamp with system timestamp
     * @param stamp_sec second
     * @param stamp_nsec nanosecond
     */
    virtual void syncLidarTimeStamp() = 0;

    /**
     * @brief Stop lidar rotation
     */
    virtual void resetLidar() = 0;

    /**
     * @brief Stop lidar rotation
     */
    virtual void stopLidarRotation() = 0;

    /**
     * @brief Start lidar rotation
     */
    virtual void startLidarRotation() = 0;

    /**
     * @brief Set the Lidar Ip Address Config 
     * @param config 
     */
    virtual void setLidarIpAddressConfig(LidarIpAddressConfig config) = 0;

    /**
     * @brief Set the Lidar Mac Address Config
     * @param config 
     */
    virtual void setLidarMacAddressConfig(LidarMacAddressConfig config) = 0;

    /**
     * @brief Get the Buffer Cached Size
     */
    virtual size_t getBufferCachedSize() const = 0;

    /**
     * @brief Get the Buffer Read Size
     */
    virtual size_t getBufferReadSize() const = 0;

    
};

/**
 * @brief Create a Unitree Lidar Reader object
 * @return UnitreeLidarReader*
 */
UnitreeLidarReader *createUnitreeLidarReader();

} // end of namespace unilidar_sdk2