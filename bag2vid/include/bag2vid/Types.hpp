/*
 * @file Types.hpp
 * @author Stathi Weir (stathi.weir@gmail.com)
 *
 * @brief This file contains the type definitions used in the project.
 * @date 2024-05-25
 */

#pragma once

#include <memory>
#include <vector>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>


namespace bag2vid
{

using ImagePtr = std::shared_ptr<sensor_msgs::msg::Image>;
using CompressedImagePtr = std::shared_ptr<sensor_msgs::msg::CompressedImage>;
using MessageInstancePtr = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;

// Immutable, ref-counted list of bag messages — shared across N camera panes
// so the list outlives any single pane reload and never copies per-pane.
using MessagesPtr = std::shared_ptr<const std::vector<MessageInstancePtr>>;

} // namespace bag2vid
