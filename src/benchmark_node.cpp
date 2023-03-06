#include "./smoother.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <motion_utils/trajectory/tmp_conversion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>

// smoother
#include <motion_velocity_smoother/motion_velocity_smoother_node.hpp>
#include <motion_velocity_smoother/smoother/jerk_filtered_smoother.hpp>

#include <matplotlibcpp17/pyplot.h>

#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <tuple>

const std::string k_in_trajectory_topic_name =
  "/planning/scenario_planning/scenario_selector/trajectory";
const std::string k_localization_topic_name = "/localization/kinematic_state";
const std::string k_max_velocity_topic_name = "/planning/scenario_planning/max_velocity";

[[maybe_unused]] static double toSec(const builtin_interfaces::msg::Time & msg)
{
  return msg.sec + static_cast<double>(msg.nanosec) / 1'000'000'000;
}

static std::tuple<
  std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>>
serialize(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & vec, double init = 0)
{
  if (vec.size() == 0) {
    return {
      std::vector<double>({init}), std::vector<double>({}), std::vector<double>({}),
      std::vector<double>({})};
  }

  std::vector<double> cumsum, lon, lat, accel;
  double sum = init;
  cumsum.push_back(sum);
  for (size_t i = 0; i < vec.size() - 1; ++i) {
    sum += tier4_autoware_utils::calcDistance2d(vec[i], vec[i + 1]);
    cumsum.push_back(sum);
    lon.push_back(vec[i].longitudinal_velocity_mps);
    lat.push_back(vec[i].lateral_velocity_mps);
    accel.push_back(vec[i].acceleration_mps2);
  }
  lon.push_back(vec.back().longitudinal_velocity_mps);
  lat.push_back(vec.back().lateral_velocity_mps);
  accel.push_back(vec.back().acceleration_mps2);

  return {cumsum, lon, lat, accel};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("get_param");

  [[maybe_unused]] auto jerk_filtered_smoother =
    std::make_shared<motion_velocity_smoother::MotionVelocitySmootherNode>(rclcpp::NodeOptions());

  const std::string bag_path = node->declare_parameter("bag_path", "./bags");
  if (not std::filesystem::exists(bag_path)) {
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
  std::vector<autoware_auto_planning_msgs::msg::Trajectory> in_trajectories;
  std::vector<nav_msgs::msg::Odometry> positions;
  std::vector<std::optional<tier4_planning_msgs::msg::VelocityLimit>> max_velocities;

  // Helper
  rclcpp::Serialization<builtin_interfaces::msg::Time> time_serialized_helper;
  rclcpp::Serialization<autoware_auto_planning_msgs::msg::Trajectory> traj_serialize_helper;
  rclcpp::Serialization<nav_msgs::msg::Odometry> pos_serialize_helper;
  rclcpp::Serialization<tier4_planning_msgs::msg::VelocityLimit> max_velocity_helper;

  // wait for first data
  std::optional<autoware_auto_planning_msgs::msg::Trajectory> latest_in_traj;
  std::optional<nav_msgs::msg::Odometry> latest_pos;
  std::optional<tier4_planning_msgs::msg::VelocityLimit> latest_max_velocity;

  auto in_data_ready = [&]() { return latest_in_traj.has_value() && latest_pos.has_value(); };

  while (reader.has_next()) {
    const auto serialized_message = reader.read_next();
    if (serialized_message->topic_name.compare(k_localization_topic_name) == 0) {
      // localization
      const rclcpp::SerializedMessage raw_pos_msg(*serialized_message->serialized_data);
      nav_msgs::msg::Odometry pos_msg;
      pos_serialize_helper.deserialize_message(&raw_pos_msg, &pos_msg);
      latest_pos = pos_msg;
    } else if (serialized_message->topic_name.compare(k_max_velocity_topic_name) == 0) {
      const rclcpp::SerializedMessage raw_max_velocity_msg(*serialized_message->serialized_data);
      tier4_planning_msgs::msg::VelocityLimit max_velocity_msg;
      max_velocity_helper.deserialize_message(&raw_max_velocity_msg, &max_velocity_msg);
      latest_max_velocity = max_velocity_msg;
    } else if (serialized_message->topic_name.compare(k_in_trajectory_topic_name) == 0) {
      // trajectory
      const rclcpp::SerializedMessage raw_traj_msg(*serialized_message->serialized_data);
      autoware_auto_planning_msgs::msg::Trajectory traj_msg;
      traj_serialize_helper.deserialize_message(&raw_traj_msg, &traj_msg);
      latest_in_traj = traj_msg;
      // all data received
      if (in_data_ready()) {
        in_trajectories.push_back(latest_in_traj.value());
        positions.push_back(latest_pos.value());
        max_velocities.push_back(latest_max_velocity);
      }
    }
  }

  const size_t n_data = in_trajectories.size();
  if (n_data == 0) {
    std::cout << "featched empty data" << std::endl;
    return 1;
  }

  // plotter
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();
  auto [fig, axes] = plt.subplots(1, 2);
  auto ax1 = axes[0];
  auto ax2 = axes[1];

  SmootherFrontEnd smoother{};
  for (size_t i = 0; i < n_data; ++i) {
    const auto & in_trajectory = in_trajectories[i];
    const auto & input_odom = positions[i];
    const auto & input_max_velocity = max_velocities[i];

    {
      const auto msg = std::make_shared<nav_msgs::msg::Odometry>(input_odom);
      jerk_filtered_smoother->onCurrentOdometry(msg);
    }
    {
      const auto msg =
        input_max_velocity.has_value()
          ? std::make_shared<tier4_planning_msgs::msg::VelocityLimit>(input_max_velocity.value())
          : nullptr;
      jerk_filtered_smoother->onExternalVelocityLimit(msg);
    }
    {
      const auto msg =
        std::make_shared<autoware_auto_planning_msgs::msg::Trajectory>(in_trajectory);
      jerk_filtered_smoother->onCurrentTrajectory(msg);
    }

    const auto solution_trajectory = smoother.onCurrentTrajectory(in_trajectory, input_odom);
    std::cout << i << "-th iteration" << std::endl;

    // plot
    const auto [out_ds, out_lon, out_lat, out_accel] =
      serialize(jerk_filtered_smoother->getPrevSolution(), 0.0);
    const auto [solu_ds, solu_lon, solu_lat, solu_accel] = serialize(solution_trajectory, 0.0);
    ax1.plot(
      Args(out_ds, out_accel),
      Kwargs("color"_a = "blue", "label"_a = "ground truth", "linestyle"_a = "dashed"));
    ax1.plot(Args(solu_ds, solu_accel), Kwargs("color"_a = "red", "label"_a = "solution"));
    ax1.set_xlabel(Args("distance"));
    ax1.set_ylabel(Args(R"(Acceleration [$m/s^2$])"));

    ax2.plot(
      Args(out_ds, out_lon),
      Kwargs("color"_a = "blue", "label"_a = "ground truth", "linestyle"_a = "dashed"));
    ax2.plot(Args(solu_ds, solu_lon), Kwargs("color"_a = "red", "label"_a = "solution"));
    ax2.set_xlabel(Args("distance"));
    ax2.set_ylabel(Args(R"(longituional velocity [$m/s$])"));

    ax1.legend();
    ax2.legend();

    plt.pause(Args(0.1));
    ax1.cla();
    ax2.cla();
  }

  return 0;
}
