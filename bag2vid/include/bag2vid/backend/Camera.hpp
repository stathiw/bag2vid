/*
 * @file Camera.hpp
 * @author Stathi Weir (stathi.weir@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-09
 */

#pragma once

#include <string>
#include <vector>
#include <memory>

#include <rosbag2_storage/serialized_bag_message.hpp>

namespace bag2vid
{
class Camera
{
  public:
    inline Camera(const std::string &name, const std::string &topic,
                  const std::string &image_type)
        : name_(name), topic_(topic), image_type_(image_type) {}
    ~Camera();

    /**
     * @brief Add messages to the camera
     */
    void addMessages(
        const std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> &messages);

    /**
     * @brief Get the name of the camera
     */
    inline std::string getName() const { return name_; }

    /**
     * @brief Get the topic of the camera
     */
    inline std::string getTopic() const { return topic_; }

    /**
     * @brief Get the image type of the camera
     */
    inline std::string getImageType() const { return image_type_; }

    /**
     * @brief Get the start time of the camera (seconds)
     */
    inline double getStartTime() const { return start_time_; }

    /**
     * @brief Get the end time of the camera (seconds)
     */
    inline double getEndTime() const { return end_time_; }

    /**
     * @brief Get the messages of the camera
     */
    inline std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>
    getMessages() const {
      return messages_;
    }

  private:
    std::string name_;
    std::string topic_;
    std::string image_type_;

    double start_time_;
    double end_time_;

    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages_;
};

} // namespace bag2vid
