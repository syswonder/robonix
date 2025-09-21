/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/impl/instantiate.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/approximate_voxel_grid.hpp>
#include <pcl/filters/impl/radius_outlier_removal.hpp>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/impl/passthrough.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "unitree_lidar_sdk.h"

using namespace unilidar_sdk2;

/**
 * @brief PCL Point Type
 */
struct PointType
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    std::uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(float, time, time))

PCL_INSTANTIATE(VoxelGrid, PointType)
PCL_INSTANTIATE(KdTree, PointType)
PCL_INSTANTIATE(RadiusOutlierRemoval, PointType)

/**
 * @brief Transform a Unitree cloud to PCL cloud
 *
 * @param cloudIn
 * @param cloudOut
 */
void transformUnitreeCloudToPCL(const PointCloudUnitree &cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut)
{
    cloudOut->clear();
    PointType pt;
    for (size_t i = 0; i < cloudIn.points.size(); i++)
    {
        pt.x = cloudIn.points[i].x;
        pt.y = cloudIn.points[i].y;
        pt.z = cloudIn.points[i].z;
        pt.intensity = cloudIn.points[i].intensity;
        pt.time = cloudIn.points[i].time;
        pt.ring = cloudIn.points[i].ring;
        cloudOut->push_back(pt);
    }
}