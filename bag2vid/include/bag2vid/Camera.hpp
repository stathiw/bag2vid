#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

pragma once

    namespace bag2vid {

  class Camera {
  public:
    inline Camera(const std::string &name, const std::string &topic,
                  const std::string &image_type)
        : name_(name), topic_(topic), image_type_(image_type) {}
    ~Camera();

    /**
     * @brief Add messages to the camera
     */
    void addMessages(
        const std::vector<std::shared_ptr<rosbag::MessageInstance>> &messages);

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
     * @brief Get the start time of the camera
     */
    inline ros::Time getStartTime() const { return start_time_; }

    /**
     * @brief Get the end time of the camera
     */
    inline ros::Time getEndTime() const { return end_time_; }

    /**
     * @brief Get the messages of the camera
     */
    inline std::vector<std::shared_ptr<rosbag::MessageInstance>>
    getMessages() const {
      return messages_;
    }

  private:
    std::string name_;
    std::string topic_;
    std::string image_type_;

    ros::Time start_time_;
    ros::Time end_time_;

    std::vector<std::shared_ptr<rosbag::MessageInstance>> messages_;
  };

} // namespace bag2vid
