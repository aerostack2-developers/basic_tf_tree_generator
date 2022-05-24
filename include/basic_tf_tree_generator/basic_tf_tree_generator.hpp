#ifndef BASIC_TF_TREE_GENERATOR_HPP_
#define BASIC_TF_TREE_GENERATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include "as2_core/node.hpp"
#include <as2_core/tf_utils.hpp>
#include "as2_core/names/topics.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "nav_msgs/msg/odometry.hpp"

class BasicTFTreeGenerator : public as2::Node
{
public:
  /**
   * @brief Constructor of the BasicTFTreeGenerator object
   *
   */
  BasicTFTreeGenerator();

  /**
   * @brief Initial setup for node Transforms
   * This function generate the Transform tree
   * from Map to Baselink.
   */
  void setupTf();

  /**
   * @brief Publish Transforms tree
   *
   */
  void publishTFs();

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfstatic_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  std::vector<geometry_msgs::msg::TransformStamped> tf2_fix_transforms_;
  geometry_msgs::msg::TransformStamped odom2base_link_tf_;
};

#endif