#pragma once

#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <QImage>
#include <QObject>

class VideoPlayer : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief Construct a new VideoPlayer object.
     *
     * @param parent
     */
    explicit VideoPlayer(QObject *parent = nullptr);

    /**
     * @brief Destroy the VideoPlayer object.
     *
     */
    ~VideoPlayer();

    /**
     * @brief Get the current frame id.
     *
     * @return int
     */
    int getCurrentFrameId() const { return current_frame_; }

    /**
     * @brief Get the bag-relative timestamp of the previous frame, or -1 if none.
     */
    double prevFrameTime() const;

    /**
     * @brief Get the bag-relative timestamp of the next frame, or -1 if none.
     */
    double nextFrameTime() const;

public slots:
    /**
     * @brief Render the frame appropriate for the given bag-relative time.
     *
     * @param time Seconds since bag start.
     */
    void onClockTick(double time);

    /**
     * @brief Load messages from a rosbag.
     *
     * @param messages Vector of shared pointers to serialized bag messages.
     * @param message_type The ROS message type string (e.g. "sensor_msgs/msg/Image").
     * @param bag_start_time Absolute time (seconds) that corresponds to bag-relative t=0.
     */
    void loadMessages(std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages,
                      const std::string& message_type,
                      double bag_start_time);

signals:
    /**
     * @brief Signal emitted when a new frame is available.
     *
     * @param frame
     */
    void newFrame(const QImage &frame);

private:
    // Current frame index into messages_
    int current_frame_;
    // Absolute time (seconds) that corresponds to bag-relative t=0
    double bag_start_time_;
    // Vector of shared pointers to serialized bag messages
    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages_;
    // The ROS message type string for this set of messages
    std::string message_type_;

    /**
     * @brief Bag-relative timestamp of the message at the given index.
     */
    double messageTime(int index) const;

    /**
     * @brief Process and emit a frame from the message at the given index.
     *
     * @param index The index into messages_.
     */
    void processFrame(int index);

    /**
     * @brief Process ROS Image message.
     *
     * @param msg std::shared_ptr to the Image message.
     */
    void processImageMessage(const sensor_msgs::msg::Image::SharedPtr &msg);

    /**
     * @brief Process ROS CompressedImage message.
     *
     * @param msg std::shared_ptr to the CompressedImage message.
     */
    void processCompressedImageMessage(const sensor_msgs::msg::CompressedImage::SharedPtr &msg);
};
