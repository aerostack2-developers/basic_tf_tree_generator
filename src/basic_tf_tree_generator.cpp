#include "basic_tf_tree_generator.hpp"

geometry_msgs::msg::TransformStamped getTransformation(	const std::string& _frame_id,
												  	const std::string& _child_frame_id,
													double _translation_x,
													double _translation_y,
													double _translation_z,
													double _roll,
													double _pitch,
													double _yaw  ){

geometry_msgs::msg::TransformStamped transformation;

	transformation.header.frame_id = _frame_id;
	transformation.child_frame_id  = _child_frame_id;
	transformation.transform.translation.x = _translation_x;
	transformation.transform.translation.y = _translation_y;
	transformation.transform.translation.z = _translation_z;
	tf2::Quaternion q;
	q.setRPY(_roll, _pitch, _yaw);
	transformation.transform.rotation.x = q.x();
	transformation.transform.rotation.y = q.y();
	transformation.transform.rotation.z = q.z();
	transformation.transform.rotation.w = q.w();
	
	return transformation;

}

BasicTFTreeGenerator::BasicTFTreeGenerator():aerostack2::Node("basicTFTreeGenerator"){

    // Initialize the transform broadcaster
    tf_broadcaster_       = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tfstatic_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->generate_global_name("self_localization/odom"),1,
        std::bind(&BasicTFTreeGenerator::odomCallback, this, std::placeholders::_1));
};


void BasicTFTreeGenerator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){

    rclcpp::Time timestamp = this->get_clock()->now();

    // odom2base_link_tf_.header.frame_id = "odom";
    // odom2base_link_tf_.child_frame_id  = "base_link";
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
    tf2_fix_transforms_.emplace_back(getTransformation("map",ns+"/odom",0,0,0,0,0,0));
    
    // Odom_rs to rs_link_own
	odom2base_link_tf_.header.frame_id = ns + "/odom";
	odom2base_link_tf_.child_frame_id  = ns + "/base_link";
	odom2base_link_tf_.transform.rotation.w = 1.0f;

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
