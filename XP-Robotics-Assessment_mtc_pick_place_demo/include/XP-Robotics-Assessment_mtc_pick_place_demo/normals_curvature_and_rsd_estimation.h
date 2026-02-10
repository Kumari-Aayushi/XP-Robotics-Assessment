#ifndef NORMALS_CURVATURE_AND_RSD_ESTIMATION_H
#define NORMALS_CURVATURE_AND_RSD_ESTIMATION_H

/**
 * @file normals_curvature_and_rsd_estimation.h
 * @brief Estimate normal vectors, curvature values, and RSD values for each point in the point cloud.
 *
 * This file contains function declarations and type definitions for estimating normal vectors,
 * curvature values, and Radius-based Surface Descriptor (RSD) values for each point in a given point cloud.
 * It uses the PCL library for point cloud processing.
 *
 * Input:
 * - Point cloud (of type pcl::PointCloud<pcl::PointXYZRGB>::Ptr)
 * - Parameters for the estimation process (k_neighbors, max_plane_error, max_iterations, min_boundary_neighbors, rsd_radius)
 *
 * Output:
 * - Point Cloud with Normal Vectors, Curvature Values, and RSD Values (of type pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr)
 *
 * @author XP-Robotics-Assessment
 * @date December 20, 2024
 */

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/rsd.h>
#include <pcl/common/pca.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/mlesac.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/segmentation/region_growing.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <numeric>
#include <vector>

struct PointXYZRGBNormalRSD {
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    PCL_ADD_NORMAL4D;
    float curvature;
    float r_min;
    float r_max;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBNormalRSD,
    (float, x, x)(float, y, y)(float, z, z)
    (float, rgb, rgb)
    (float, normal_x, normal_x)(float, normal_y, normal_y)(float, normal_z, normal_z)
    (float, curvature, curvature)
    (float, r_min, r_min)(float, r_max, r_max)
)

PCL_INSTANTIATE_PRODUCT(KdTree, (pcl::PointXYZRGBNormalRSD))
PCL_INSTANTIATE_PRODUCT(Search, (pcl::PointXYZRGBNormalRSD))
PCL_INSTANTIATE_PRODUCT(RegionGrowing, ((pcl::PointXYZRGBNormalRSD))(pcl::Normal))

void LOG_INFO(const std::string& msg);

pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr
estimateNormalsCurvatureAndRSD(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
    int k_neighbors,
    double max_plane_error,
    int max_iterations,
    int min_boundary_neighbors,
    double rsd_radius);

#endif // NORMALS_CURVATURE_AND_RSD_ESTIMATION_H
