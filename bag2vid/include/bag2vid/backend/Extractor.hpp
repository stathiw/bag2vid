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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

namespace bag2vid
{
class Extractor
{
public:
  inline Extractor() {}
  inline ~Extractor() { bag_.close(); }

  /**
   * @brief Loads the given rosbag file
   *
   * @param bag_file The path to the rosbag file
   * @return True if the bag was loaded successfully, false otherwise
   */
  bool loadBag(const std::string &bag_file);

  std::vector<bag2vid::MessageInstancePtr> extractMessages(const std::string &topic, const std::string &camera_name);

  /**
   * @brief Extracts all images from the given topic
   *
   * @param topic The topic to extract images from
   * @return A vector of shared pointers to the extracted images
   */
  std::vector<bag2vid::ImagePtr> extractImages(const std::string &topic, const std::string &camera_name);

  /**
   * @brief Extracts all compressed images from the given topic
   *
   * @param topic The topic to extract compressed images from
   * @return A vector of shared pointers to the extracted compressed images
   */
  std::vector<bag2vid::CompressedImagePtr> extractCompressedImages(const std::string &topic);

  /**
   * @brief Writes to video between the given timestamps from the given topic
   *
   * @param topic The topic to extract images from
   * @param start_time The start time of the video
   * @param end_time The end time of the video
   * @param video_file The path to the video file
   * @return True if the video was written successfully, false otherwise
   */
  bool writeVideo(const std::string &topic, const ros::Time &start_time,
                  const ros::Time &end_time, const std::string &video_file);

private:
  rosbag::Bag bag_;

  // Rosbag file path
  std::string bag_file_;

  // Bag start time and end time
  ros::Time bag_start_time_;
  ros::Time bag_end_time_;

  // Dictionay of image topics
  // Maps topic names to a vector of shared pointers to the image messages
  std::map<std::string, std::vector<bag2vid::MessageInstancePtr>> image_data_;

  cv::VideoWriter video_writer_;
};

} // namespace bag2vid
