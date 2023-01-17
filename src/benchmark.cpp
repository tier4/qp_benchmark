#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <filesystem>
#include <iostream>

const std::string k_trajectory_topic_name =
  "/planning/scenario_planning/scenario_selector/trajectory";

int main(int argc, char ** argv)
{
  if (argc <= 1) {
    std::cout << "Usage: ros2 run trajectory_optimizer benchmark <path to .db3>" << std::endl;
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

  // Fetchall trajectories
  std::vector<autoware_auto_planning_msgs::msg::Trajectory> trajectories;
  rclcpp::Serialization<autoware_auto_planning_msgs::msg::Trajectory> serialize_helper;

  while (reader.has_next()) {
    const auto serialized_message = reader.read_next();
    if (not serialized_message->topic_name.compare(k_trajectory_topic_name) == 0) continue;

    const rclcpp::SerializedMessage raw_traj_msg(*serialized_message->serialized_data);
    autoware_auto_planning_msgs::msg::Trajectory traj_msg;
    serialize_helper.deserialize_message(&raw_traj_msg, &traj_msg);
    trajectories.push_back(std::move(traj_msg));
  }

  const size_t n_trajectory = trajectories.size();

  // Use
  const auto & sample_trajectory = trajectories[n_trajectory / 2];
  const auto & prev_sample_trajectory = trajectories[n_trajectory / 2 - 1];

  std::cout << "sample_trajectory size is " << sample_trajectory.points.size() << std::endl;
  std::cout << "prev_sample_trajectory size is " << prev_sample_trajectory.points.size()
            << std::endl;
}
