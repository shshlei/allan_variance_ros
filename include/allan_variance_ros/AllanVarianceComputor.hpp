#ifndef ALLAN_VARIANCE_ROS_ALLANVARIANCECOMPUTOR_H
#define ALLAN_VARIANCE_ROS_ALLANVARIANCECOMPUTOR_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rcutils/time.h>
#include <sensor_msgs/msg/imu.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <fstream>

namespace allan_variance_ros
{

class AllanVarianceComputor : public rclcpp::Node
{
public:

  template <class T>
  using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;

  AllanVarianceComputor(const std::string & node_name);

  virtual ~AllanVarianceComputor() = default;

private:

  struct ImuMeasurement
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ROS time message received (nanoseconds).
    uint64_t t;

    // Raw acceleration from the IMU (m/s/s)
    Eigen::Vector3d I_a_WI;

    // Raw angular velocity from the IMU (deg/s)
    Eigen::Vector3d I_w_WI;
  };

  template <typename T>
  double stamp2NanoSec(const T & stamp)
  {
    return RCUTILS_S_TO_NS(rclcpp::Time(stamp).seconds());
  }

  void allanVariance(const EigenVector<ImuMeasurement> & imuBuffer);

  void writeAllanDeviation(const std::vector<double> & variance, double period);

private:

  std::ofstream av_output_;

  rosbag2_cpp::Reader reader_;

  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization_;

  std::string imu_topic_;

  /// Location of the ROS bag we want to read in
  std::string path_to_bag_;

  /// Get our start location and how much of the bag we want to play
  /// Make the bag duration < 0 to just process to the end of the bag
  double bag_start_, bag_durr_;

  // Percent to overlap bins
  double overlap_;

  // Data
  int imu_rate_ = 100;
};
}  // namespace allan_variance_ros

#endif // ALLAN_VARIANCE_ROS_ALLANVARIANCECOMPUTOR_H
