// "Copyright [year] <Copyright Owner>"

#include "basic_tf_tree_generator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicTFTreeGenerator>());
  rclcpp::shutdown();
  return 0;
}