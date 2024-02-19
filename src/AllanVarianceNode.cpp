#include "allan_variance_ros/AllanVarianceComputor.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<allan_variance_ros::AllanVarianceComputor>("allan_variance_ros"));
  allan_variance_ros::AllanVarianceComputor allan_variance_ros_node("allan_variance_ros");
  rclcpp::shutdown();
  return 0;
}
