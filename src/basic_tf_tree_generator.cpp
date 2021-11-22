#include "basic_tf_tree_generator.hpp"

BasicTFTreeGenerator::BasicTFTreeGenerator():as2::Node("basicTFTreeGenerator"){
    // Initialize the transform broadcaster
    tf_broadcaster_       = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tfstatic_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->generate_global_name("self_localization/odom"),1,
        std::bind(&BasicTFTreeGenerator::odomCallback, this, std::placeholders::_1));

    this->setupTf();
};


void BasicTFTreeGenerator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){

    rclcpp::Time timestamp = this->get_clock()->now();

    odom2base_link_tf_.header.stamp    = timestamp;
    odom2base_link_tf_.transform.translation.x = msg->pose.pose.position.x;
    odom2base_link_tf_.transform.translation.y = msg->pose.pose.position.y;
    odom2base_link_tf_.transform.translation.z = msg->pose.pose.position.z;
    odom2base_link_tf_.transform.rotation.x = msg->pose.pose.orientation.x;
    odom2base_link_tf_.transform.rotation.y = msg->pose.pose.orientation.y;
    odom2base_link_tf_.transform.rotation.z = msg->pose.pose.orientation.z;
    odom2base_link_tf_.transform.rotation.w = msg->pose.pose.orientation.w;

    // Get the current time    
    publishTFs();
}


void BasicTFTreeGenerator::setupTf()
{
	tf2_fix_transforms_.clear();
    // global reference to drone reference
    std::string ns = this->get_namespace();

    tf2_fix_transforms_.emplace_back(getTransformation("map",generateTfName(ns,"odom"),0,0,0,0,0,0));
    
    // Odom_rs to rs_link_own
	odom2base_link_tf_.header.frame_id = generateTfName(ns,"odom");
	odom2base_link_tf_.child_frame_id  = generateTfName(ns,"base_link");
	odom2base_link_tf_.transform.rotation.w = 1.0f;

    RCLCPP_INFO(get_logger(),odom2base_link_tf_.header.frame_id);
    RCLCPP_INFO(get_logger(),odom2base_link_tf_.child_frame_id);

    publishTFs();
}

void BasicTFTreeGenerator::publishTFs(){
    rclcpp::Time timestamp = this->get_clock()->now();
    for (geometry_msgs::msg::TransformStamped& transform:tf2_fix_transforms_){
        transform.header.stamp = timestamp;
        tfstatic_broadcaster_->sendTransform(transform);
    }
    odom2base_link_tf_.header.stamp = timestamp;
    tf_broadcaster_->sendTransform(odom2base_link_tf_);
}