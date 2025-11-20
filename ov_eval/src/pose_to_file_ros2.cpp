/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "utils/RecorderROS2.h"
#include "utils/print.h"

int main(int argc, char **argv) {

  // Create ros node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pose_to_file");

  // Verbosity setting
  std::string verbosity;
  node->declare_parameter<std::string>("verbosity", "INFO");
  node->get_parameter("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Get parameters to subscribe
  std::string topic, topic_type, fileoutput;
  node->declare_parameter<std::string>("topic", "");
  node->declare_parameter<std::string>("topic_type", "");
  node->declare_parameter<std::string>("output", "");
  node->get_parameter("topic", topic);
  node->get_parameter("topic_type", topic_type);
  node->get_parameter("output", fileoutput);

  if (topic.empty() || topic_type.empty() || fileoutput.empty()) {
    PRINT_ERROR("Missing required parameters!");
    PRINT_ERROR("Usage: ros2 run ov_eval pose_to_file --ros-args -p topic:=<topic> -p topic_type:=<type> -p output:=<file>");
    PRINT_ERROR("  topic_type options: PoseWithCovarianceStamped, PoseStamped, TransformStamped, Odometry");
    return EXIT_FAILURE;
  }

  // Debug
  PRINT_DEBUG("Done reading config values");
  PRINT_DEBUG(" - topic = %s", topic.c_str());
  PRINT_DEBUG(" - topic_type = %s", topic_type.c_str());
  PRINT_DEBUG(" - file = %s", fileoutput.c_str());

  // Create the recorder object
  ov_eval::RecorderROS2 recorder(fileoutput);

  // Subscribe to topic based on type
  rclcpp::SubscriptionBase::SharedPtr sub;
  if (topic_type == std::string("PoseWithCovarianceStamped")) {
    sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topic, 10, [&recorder](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          recorder.callback_posecovariance(msg);
        });
  } else if (topic_type == std::string("PoseStamped")) {
    sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, 10, [&recorder](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          recorder.callback_pose(msg);
        });
  } else if (topic_type == std::string("TransformStamped")) {
    sub = node->create_subscription<geometry_msgs::msg::TransformStamped>(
        topic, 10, [&recorder](const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
          recorder.callback_transform(msg);
        });
  } else if (topic_type == std::string("Odometry")) {
    sub = node->create_subscription<nav_msgs::msg::Odometry>(
        topic, 10, [&recorder](const nav_msgs::msg::Odometry::SharedPtr msg) {
          recorder.callback_odometry(msg);
        });
  } else {
    PRINT_ERROR("The specified topic type is not supported");
    PRINT_ERROR("topic_type = %s", topic_type.c_str());
    PRINT_ERROR("please select from: PoseWithCovarianceStamped, PoseStamped, TransformStamped, Odometry");
    return EXIT_FAILURE;
  }

  // Done!
  PRINT_INFO("Spinning and recording to: %s", fileoutput.c_str());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}


