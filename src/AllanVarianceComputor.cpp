#include "allan_variance_ros/AllanVarianceComputor.hpp"

#include <mutex>
#include <map>
#include <memory>
#include <iomanip>
#include <iostream>

namespace allan_variance_ros
{

AllanVarianceComputor::AllanVarianceComputor(const std::string & node_name) : Node(node_name), overlap_(0.0)
{
  imu_topic_ = declare_parameter("imu_topic", "/imu");
  imu_rate_ = declare_parameter("imu_rate", 100);
  RCLCPP_INFO_STREAM(get_logger(), "imu_topic: " << imu_topic_);
  RCLCPP_INFO_STREAM(get_logger(), "imu_rate: " << imu_rate_);

  path_to_bag_ = declare_parameter("path_bag", "");
  bag_start_ = declare_parameter("bag_start_", 0);
  bag_durr_ = declare_parameter("bag_durr_", -1);

  // Read Data
  reader_.open(path_to_bag_);
  const rosbag2_storage::BagMetadata & mdata = reader_.get_metadata();
  rcutils_time_point_value_t start = mdata.starting_time.time_since_epoch().count() + RCUTILS_S_TO_NS(bag_start_);
  reader_.seek(start);
  rcutils_time_point_value_t end = start;
  if (bag_durr_ > 0) end = start + RCUTILS_S_TO_NS(bag_durr_);

  EigenVector<ImuMeasurement> imuBuffer;
  RCLCPP_INFO_STREAM(get_logger(), "Processing " << path_to_bag_ << " ...");
  while (reader_.has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next();
    if (msg->topic_name != imu_topic_) continue;
    if (bag_durr_ > 0 && msg->time_stamp > end) break;

    // Subsample IMU measurements
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    sensor_msgs::msg::Imu::SharedPtr s_imu = std::make_shared<sensor_msgs::msg::Imu>();
    imu_serialization_.deserialize_message(&serialized_msg, s_imu.get());

    double time_imu = stamp2NanoSec(s_imu->header.stamp);
    ImuMeasurement input;
    input.t = time_imu;
    input.I_a_WI = Eigen::Vector3d(s_imu->linear_acceleration.x, s_imu->linear_acceleration.y, s_imu->linear_acceleration.z);
    input.I_w_WI = Eigen::Vector3d(s_imu->angular_velocity.x, s_imu->angular_velocity.y, s_imu->angular_velocity.z);
    imuBuffer.push_back(input);
  }

  RCLCPP_INFO_STREAM(get_logger(), "Finished collecting data. " << imuBuffer.size() << " measurements");

  // Compute Allan Variance here
  if (!imuBuffer.empty()) {
    std::string imu_output_file = declare_parameter("output_file", "");
    av_output_ = std::ofstream(imu_output_file.c_str(), std::ofstream::out);
    allanVariance(imuBuffer);
    av_output_.close();
  }
  else {
    RCLCPP_ERROR(get_logger(), "No IMU messages to process, is your topic right?");
  }
}

void AllanVarianceComputor::allanVariance(const EigenVector<ImuMeasurement> & imuBuffer)
{
  std::mutex mtx;
  std::map<int, std::vector<std::vector<double>>> averages_map;

  // Range we will sample from (0.1s to 1000s)
  int period_min = 1;
  int period_max = 10000;

  // Overlapping method
  #pragma omp parallel for
  for (int period = period_min; period < period_max; period++) {
    double period_time = period * 0.1;  // Sampling periods from 0.1s to 1000s
    int max_bin_size = period_time * imu_rate_;
    int overlap = std::floor(max_bin_size * overlap_);

    // Compute Averages
    std::vector<std::vector<double>> averages;
    for (int j = 0; j < ((int)imuBuffer.size() - max_bin_size); j += (max_bin_size - overlap)) {
      std::vector<double> current_average = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      // get average for current bin
      for (int m = 0; m < max_bin_size; m++) {
        // // Acceleration
        current_average[0] += imuBuffer[j + m].I_a_WI[0];
        current_average[1] += imuBuffer[j + m].I_a_WI[1];
        current_average[2] += imuBuffer[j + m].I_a_WI[2];

        // Gyro - assumes measurements in radians and convert to degrees
        current_average[3] += imuBuffer[j + m].I_w_WI[0] * 180 / M_PI; // TODO
        current_average[4] += imuBuffer[j + m].I_w_WI[1] * 180 / M_PI;
        current_average[5] += imuBuffer[j + m].I_w_WI[2] * 180 / M_PI;
      }

      current_average[0] /= max_bin_size;
      current_average[1] /= max_bin_size;
      current_average[2] /= max_bin_size;
      current_average[3] /= max_bin_size;
      current_average[4] /= max_bin_size;
      current_average[5] /= max_bin_size;
      averages.push_back(current_average);
    }

    {
      std::lock_guard<std::mutex> lck(mtx);
      int num_averages = averages.size();
      RCLCPP_INFO_STREAM(get_logger(), "Computed " << num_averages << " averages for period " << period_time << " (" << (10000 - averages_map.size()) << " left)");
      averages_map[period] = averages;
    }
  }

  for (int period = period_min; period < period_max; period++) {
    const std::vector<std::vector<double>> & averages = averages_map.at(period);
    double period_time = period * 0.1;  // Sampling periods from 0.1s to 1000s
    int num_averages = averages.size();
    RCLCPP_INFO_STREAM(get_logger(), "Computed " << num_averages << " bins for sampling period " << period_time << " out of " << imuBuffer.size() << " measurements.");

    // Compute Allan Variance
    std::vector<double> allan_variance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int k = 0; k < num_averages - 1; k++) {
      allan_variance[0] += std::pow(averages[k + 1][0] - averages[k][0], 2);
      allan_variance[1] += std::pow(averages[k + 1][1] - averages[k][1], 2);
      allan_variance[2] += std::pow(averages[k + 1][2] - averages[k][2], 2);
      allan_variance[3] += std::pow(averages[k + 1][3] - averages[k][3], 2);
      allan_variance[4] += std::pow(averages[k + 1][4] - averages[k][4], 2);
      allan_variance[5] += std::pow(averages[k + 1][5] - averages[k][5], 2);
    }
    std::vector<double> avar = {
      allan_variance[0] / (2 * (num_averages - 1)), allan_variance[1] / (2 * (num_averages - 1)), allan_variance[2] / (2 * (num_averages - 1)),
      allan_variance[3] / (2 * (num_averages - 1)), allan_variance[4] / (2 * (num_averages - 1)), allan_variance[5] / (2 * (num_averages - 1))};

    std::vector<double> allan_deviation = {std::sqrt(avar[0]), std::sqrt(avar[1]), std::sqrt(avar[2]),
      std::sqrt(avar[3]), std::sqrt(avar[4]), std::sqrt(avar[5])};

    writeAllanDeviation(allan_deviation, period_time);
  }
}

void AllanVarianceComputor::writeAllanDeviation(const std::vector<double> & variance, double period)
{
  av_output_ << std::setprecision(19) << period << std::setprecision(7) << " " << variance[0] << " " << variance[1] << " " << variance[2] << " "
    << variance[3] << " " << variance[4] << " " << variance[5] << " " << std::endl;
}

}  // namespace allan_variance_ros
