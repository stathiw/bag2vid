/*
 * @file Types.hpp
 * @author Stathi Weir (stathi.weir@gmail.com)
 *
 * @brief This file contains the type definitions used in the project.
 * @date 2024-05-25
 */

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <rosbag/message_instance.h>

#pragma once

namespace bag2vid
{

using ImagePtr = std::shared_ptr<sensor_msgs::Image>;
using CompressedImagePtr = std::shared_ptr<sensor_msgs::CompressedImage>;
using MessageInstancePtr = std::shared_ptr<rosbag::MessageInstance>;

} // namespace bag2vid
