#include "bag2vid/frontend/VideoPlayer.hpp"

#include <QDebug>
#include <QImage>
#include <QThread>


VideoPlayer::VideoPlayer(QObject *parent) :
    QObject(parent),
    is_playing_(false),
    current_frame_(0),
    start_time_(0.0),
    end_time_(1.0)
{
    connect(&playback_timer_, &QTimer::timeout, this, &VideoPlayer::playback);
}

VideoPlayer::~VideoPlayer()
{
}

void VideoPlayer::seekToTime(double time)
{
    // Set current frame to the first frame after the time
    for (size_t i = 0; i < messages_.size(); i++)
    {
        double msg_time = static_cast<double>(messages_[i]->recv_timestamp) / 1e9;
        if (msg_time >= time + start_time_)
        {
            current_frame_ = i;
            break;
        }
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

void VideoPlayer::playback()
{
    if (is_playing_ && current_frame_ < static_cast<int>(messages_.size()))
    {
        processFrame(current_frame_);
        double frame_timestamp = static_cast<double>(messages_[current_frame_]->recv_timestamp) / 1e9;
        emit currentTimestamp(frame_timestamp - start_time_);
        current_frame_++;
    }
}

void VideoPlayer::play()
{
    if (!is_playing_)
    {
        is_playing_ = true;
        playback_timer_.start(1000 / 30); // 30 FPS
    }
}

void VideoPlayer::pause()
{
    is_playing_ = false;
    playback_timer_.stop();
}

void VideoPlayer::seekBackward()
{
    if (current_frame_ > 0)
    {
        current_frame_--;
        processFrame(current_frame_);
        double frame_timestamp = static_cast<double>(messages_[current_frame_]->recv_timestamp) / 1e9;
        emit currentTimestamp(frame_timestamp - start_time_);
    }
}

void VideoPlayer::seekForward()
{
    if (current_frame_ < static_cast<int>(messages_.size()))
    {
        processFrame(current_frame_);
        double frame_timestamp = static_cast<double>(messages_[current_frame_]->recv_timestamp) / 1e9;
        emit currentTimestamp(frame_timestamp - start_time_);
        current_frame_++;
    }
}

void VideoPlayer::loadMessages(std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages,
                                const std::string& message_type)
{
    current_frame_ = 0;
    messages_ = messages;
    message_type_ = message_type;
    if (messages_.empty())
    {
        return;
    }
    start_time_ = static_cast<double>(messages_[0]->recv_timestamp) / 1e9;
    end_time_ = static_cast<double>(messages_[messages_.size() - 1]->recv_timestamp) / 1e9;
    // Process first frame
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
