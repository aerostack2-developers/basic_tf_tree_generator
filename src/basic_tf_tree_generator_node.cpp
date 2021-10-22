// "Copyright [year] <Copyright Owner>"

#include "basic_tf_tree_generator.hpp"

int main(int argc, char * argv[])
{
  // find_device_with_streams
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicTFTreeGenerator>());
  rclcpp::shutdown();
  return 0;
}