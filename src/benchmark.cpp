#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include "./smoother.hpp"

#include <matplotlibcpp17/pyplot.h>

#include <filesystem>
#include <iostream>
#include <optional>

namespace
{
const std::string k_trajectory_topic_name =
  "/planning/scenario_planning/scenario_selector/trajectory";
const std::string k_localization_topic_name = "/localization/kinematic_state";

static double toSec(const builtin_interfaces::msg::Time & msg)
{
  return msg.sec + static_cast<double>(msg.nanosec) / 1'000'000'000;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc <= 1) {
    std::cout << "Usage: ros2 run trajectory_optimizer benchmark <path to bagdir (in share)>"
              << std::endl;
    return 1;
  }

  const auto bag_path = ament_index_cpp::get_package_share_directory("trajectory_optimizer") +
                        "/bags/" + std::string(argv[1]);
  if (const auto path = std::filesystem::path{bag_path}; not std::filesystem::exists(path)) {
    std::cout << "Cannot open " << bag_path << std::endl;
    return 1;
  }

  // Reader
  // Following info should be read from yaml ideally
  rosbag2_storage::StorageOptions storage_options{};
  storage_options.uri = bag_path;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::readers::SequentialReader reader;
  reader.open(storage_options, converter_options);

  const auto topic_type_info = reader.get_all_topics_and_types();

  // Fetchall data
  std::vector<double> times;
  std::vector<autoware_auto_planning_msgs::msg::Trajectory> trajectories;
  std::vector<nav_msgs::msg::Odometry> positions;

  // Helper
  rclcpp::Serialization<builtin_interfaces::msg::Time> time_serialized_helper;
  rclcpp::Serialization<autoware_auto_planning_msgs::msg::Trajectory> traj_serialize_helper;
  rclcpp::Serialization<nav_msgs::msg::Odometry> pos_serialize_helper;

  // wait for first data
  std::optional<double> latest_time;
  std::optional<autoware_auto_planning_msgs::msg::Trajectory> latest_traj;
  std::optional<nav_msgs::msg::Odometry> latest_pos;
  auto data_ready = [&]() { return latest_traj.has_value() && latest_pos.has_value(); };
  bool data_ready_prev = false;

  while (reader.has_next()) {
    const auto serialized_message = reader.read_next();
    if (serialized_message->topic_name.compare(::k_trajectory_topic_name) == 0) {
      // trajectory
      const rclcpp::SerializedMessage raw_traj_msg(*serialized_message->serialized_data);
      autoware_auto_planning_msgs::msg::Trajectory traj_msg;
      traj_serialize_helper.deserialize_message(&raw_traj_msg, &traj_msg);
      latest_traj = traj_msg;
      latest_time = ::toSec(traj_msg.header.stamp);
      // all data received
      if (data_ready() && !data_ready_prev) {
        times.push_back(0.0);
        trajectories.push_back(std::move(traj_msg));
        positions.push_back(latest_pos.value());
        data_ready_prev = true;
      }
      // use latest for others
      else if (data_ready() && data_ready_prev) {
        times.push_back(latest_time.value() - times.back());
        trajectories.push_back(std::move(traj_msg));
        positions.push_back(positions.back());
      }
    } else if (serialized_message->topic_name.compare(::k_localization_topic_name) == 0) {
      // localization
      const rclcpp::SerializedMessage raw_pos_msg(*serialized_message->serialized_data);
      nav_msgs::msg::Odometry pos_msg;
      pos_serialize_helper.deserialize_message(&raw_pos_msg, &pos_msg);
      latest_pos = pos_msg;
      latest_time = toSec(pos_msg.header.stamp);
      // all data received
      if (data_ready() && !data_ready_prev) {
        times.push_back(0.0);
        trajectories.push_back(latest_traj.value());
        positions.push_back(std::move(pos_msg));
        data_ready_prev = true;
      }
      // use latest for others
      else if (data_ready() && data_ready_prev) {
        times.push_back(latest_time.value() - times.back());
        trajectories.push_back(trajectories.back());
        positions.push_back(std::move(pos_msg));
      }
    }
  }

  const size_t n_data = trajectories.size();
  if (n_data == 0) {
    std::cout << "featched empty data" << std::endl;
    return 1;
  }

  // plotter
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();

  /*
  SmootherFrontEnd smoother{};
  for (size_t i = 0; i < n_data; ++i) {
    const auto & input_trajectory = trajectories[i];
    const auto & input_odom = positions[i];
    const auto output_trajectory = smoother.onCurrentTrajectory(input_trajectory, input_odom);
    break;
  }
  */

  plt.plot(Args(std::vector<int>({1, 3, 2, 4})), Kwargs("color"_a = "blue", "linewidth"_a = 1.0));
  plt.show();
}
