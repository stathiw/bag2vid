#include "bag2vid/frontend/VideoPlayer.hpp"

#include <QImage>

VideoPlayer::VideoPlayer(QObject *parent) :
    QObject(parent),
    current_frame_(0),
    bag_start_time_(0.0)
{
}

VideoPlayer::~VideoPlayer()
{
}

double VideoPlayer::messageTime(int index) const
{
    if (index < 0 || index >= static_cast<int>(messages_.size()))
    {
        return -1.0;
    }
    return static_cast<double>(messages_[index]->recv_timestamp) / 1e9 - bag_start_time_;
}

double VideoPlayer::prevFrameTime() const
{
    return messageTime(current_frame_ - 1);
}

double VideoPlayer::nextFrameTime() const
{
    return messageTime(current_frame_ + 1);
}

void VideoPlayer::onClockTick(double time)
{
    if (messages_.empty())
    {
        return;
    }

    int new_frame = current_frame_;

    // Walk forward while the next frame is still at or before `time`
    while (new_frame + 1 < static_cast<int>(messages_.size()) &&
           messageTime(new_frame + 1) <= time)
    {
        new_frame++;
    }
    // Walk backward (seek) while the current frame is after `time`
    while (new_frame > 0 && messageTime(new_frame) > time)
    {
        new_frame--;
    }

    if (new_frame != current_frame_)
    {
        current_frame_ = new_frame;
        processFrame(current_frame_);
    }
}

void VideoPlayer::processFrame(int index)
{
    if (index < 0 || index >= static_cast<int>(messages_.size()))
        return;

    rclcpp::SerializedMessage serialized_msg(*messages_[index]->serialized_data);

    if (message_type_ == "sensor_msgs/msg/Image")
    {
        auto ros_image = std::make_shared<sensor_msgs::msg::Image>();
        rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
        serializer.deserialize_message(&serialized_msg, ros_image.get());
        processImageMessage(ros_image);
    }
    else if (message_type_ == "sensor_msgs/msg/CompressedImage")
    {
        auto ros_compressed = std::make_shared<sensor_msgs::msg::CompressedImage>();
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;
        serializer.deserialize_message(&serialized_msg, ros_compressed.get());
        processCompressedImageMessage(ros_compressed);
    }
}

void VideoPlayer::loadMessages(std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages,
                                const std::string& message_type,
                                double bag_start_time)
{
    current_frame_ = 0;
    messages_ = messages;
    message_type_ = message_type;
    bag_start_time_ = bag_start_time;
    if (messages_.empty())
    {
        return;
    }
    processFrame(0);
}

void VideoPlayer::processImageMessage(const sensor_msgs::msg::Image::SharedPtr &msg)
{
    if (msg != nullptr)
    {
        emit newFrame(QImage(msg->data.data(), msg->width, msg->height, QImage::Format_RGB888));
    }
}

void VideoPlayer::processCompressedImageMessage(const sensor_msgs::msg::CompressedImage::SharedPtr &msg)
{
    if (msg != nullptr)
    {
        emit newFrame(QImage::fromData(msg->data.data(), msg->data.size(), "JPEG"));
    }
}
