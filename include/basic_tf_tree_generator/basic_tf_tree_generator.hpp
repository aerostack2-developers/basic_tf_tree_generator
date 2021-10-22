#ifndef BASIC_TF_TREE_GENERATOR_HPP_
#define BASIC_TF_TREE_GENERATOR_HPP_


#include "nav_msgs/msg/odometry.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "aerostack2_core/node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

class BasicTFTreeGenerator:public aerostack2::Node{
    public:
        BasicTFTreeGenerator();
        
        // void runOdom();

        void setupTf();
        void publishTFs();
        
        void publishEstimatedPose();

    private:
        
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfstatic_broadcaster_;
        // std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        std::vector<geometry_msgs::msg::TransformStamped> tf2_fix_transforms_;
        geometry_msgs::msg::TransformStamped odom2base_link_tf_;
        

};

#endif 