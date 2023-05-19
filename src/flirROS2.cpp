#include <flir_ros2/flir_ros2.hpp>



int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<flirROS2>();
  RCLCPP_INFO(node->get_logger(), "Starting FLIR ROS2 Drivers");

  rclcpp::spin(node);  // should not return
  rclcpp::shutdown();
  return 0;
}
