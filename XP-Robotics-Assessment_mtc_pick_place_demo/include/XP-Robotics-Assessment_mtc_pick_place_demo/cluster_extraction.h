#ifndef CLUSTER_EXTRACTION_H
#define CLUSTER_EXTRACTION_H

/**
 * @file cluster_extraction.h
 * @brief Header file for cluster extraction using region growing algorithm.
 *
 * This file contains function declarations for extracting clusters from a point cloud
 * with XYZ, RGB, normal vectors, curvature values, and Radius-based Surface Descriptor (RSD) values.
 * It uses the PCL library for point cloud processing.
 *
 * @author Addison Sears-Collins
 * @date December 20, 2024
 */

// Include the header that defines PointXYZRGBNormalRSD and LOG_INFO
#include "XP-Robotics-Assessment_mtc_pick_place_demo/normals_curvature_and_rsd_estimation.h"

#include <pcl/segmentation/region_growing.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/centroid.h>
#include <limits>
#include <algorithm>

/**
 * @brief Extract clusters from a point cloud using region growing algorithm.
 */
std::vector<pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr>
extractClusters(
    const pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr& input_cloud,
    unsigned int min_cluster_size,
    unsigned int max_cluster_size,
    float smoothness_threshold,
    float curvature_threshold,
    unsigned int nearest_neighbors);

#endif // CLUSTER_EXTRACTION_H
