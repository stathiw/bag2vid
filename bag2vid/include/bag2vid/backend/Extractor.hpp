/*
 * @file Extractor.hpp
 * @author Stathi Weir (stathi.weir@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-09
 */

#pragma once

#include "bag2vid/Types.hpp"

#include <iostream>
#include <functional>
#include <map>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace bag2vid
{
class Extractor
{
public:
  using ProgressCallback = std::function<void(int)>;

  inline Extractor() {}
  inline ~Extractor() {
    progress_callback_ = nullptr;
  }

  /**
   * @brief Loads the given rosbag file
   *
   * @param bag_file The path to the rosbag file
   * @return True if the bag was loaded successfully, false otherwise
   */
  bool loadBag(const std::string &bag_file);

  void closeBag();

  inline double getBagStartTime() { return bag_start_time_sec_; }

  inline double getBagEndTime() { return bag_end_time_sec_; }

  std::vector<std::string> getImageTopics();

  /**
   * @brief Get the message type string for a given topic
   */
  std::string getTopicType(const std::string& topic);

  bag2vid::MessagesPtr extractMessages(const std::string &topic, const std::string &camera_name);

  /**
   * @brief Captures a screenshot of the current video frame
   *
   * @param frame_id The frame id of the screenshot
   * @param image_file The path to the image file
   * @return True if the screenshot was captured successfully, false otherwise
   */
  bool captureScreenshot(const std::string& camera_name, const int &frame_id, const std::string &image_file);

  /**
   * @brief Writes to video between the given timestamps from the given topic
   *
   * @param topic The topic to extract images from
   * @param start_time The start time of the video (seconds)
   * @param end_time The end time of the video (seconds)
   * @param video_file The path to the video file
   * @return True if the video was written successfully, false otherwise
   */
  bool writeVideo(const std::string &topic, const double &start_time,
                  const double &end_time, const std::string &video_file);

  void setProgressCallback(ProgressCallback callback);

private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;

  // Rosbag file path
  std::string bag_file_;

  // Bag start time and end time (seconds)
  double bag_start_time_sec_ = 0.0;
  double bag_end_time_sec_ = 0.0;

  std::vector<std::string> image_topics_;

  // Maps topic name to its message type string
  std::map<std::string, std::string> topic_type_map_;

  // Maps camera name to its topic name
  std::map<std::string, std::string> camera_topic_map_;

  // Dictionary of image topics
  // Maps camera names to a ref-counted, immutable list of serialized messages.
  // Shared out to N panes without copying the underlying vector.
  std::map<std::string, bag2vid::MessagesPtr> image_data_;

  cv::VideoWriter video_writer_;

  ProgressCallback progress_callback_;

  /**
   * @brief Deserialize a serialized bag message to a cv::Mat image
   */
  cv::Mat deserializeToImage(const bag2vid::MessageInstancePtr& msg, const std::string& type_str);
};

} // namespace bag2vid
