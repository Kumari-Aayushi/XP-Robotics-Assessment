#ifndef GET_PLANNING_SCENE_CLIENT_H
#define GET_PLANNING_SCENE_CLIENT_H

/**
 * @file get_planning_scene_client.h
 * @brief Header file for the GetPlanningSceneClient class.
 *
 * This file declares the GetPlanningSceneClient class, which implements a ROS2 node
 * that acts as a client for the GetPlanningScene service. The class provides functionality
 * to request and process planning scene information, including collision objects,
 * point cloud data, and RGB image data.
 *
 * @author Addison Sears-Collins
 * @date December 20, 2024
 */

#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "XP-Robotics-Assessment_interfaces/srv/get_planning_scene.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "moveit_msgs/msg/planning_scene_world.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/plane.hpp>

/**
 * @class GetPlanningSceneClient
 * @brief A ROS2 node that acts as a client for the GetPlanningScene service.
 */
class GetPlanningSceneClient : public rclcpp::Node
{
public:
  struct PlanningSceneResponse {
    moveit_msgs::msg::PlanningSceneWorld scene_world;
    sensor_msgs::msg::PointCloud2 full_cloud;
    sensor_msgs::msg::Image rgb_image;
    std::string target_object_id;
    std::string support_surface_id;
    bool success;
  };

  GetPlanningSceneClient();

  PlanningSceneResponse call_service(
      const std::string& target_shape,
      const std::vector<double>& target_dimensions);

private:
  rclcpp::Client<XP-Robotics-Assessment_interfaces::srv::GetPlanningScene>::SharedPtr client_;

  void log_response_info(const PlanningSceneResponse& response);
};

#endif // GET_PLANNING_SCENE_CLIENT_H
