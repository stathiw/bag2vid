#include "bag2vid/frontend/VideoPlayer.hpp"

#include <QDebug>
#include <QImage>
#include <QThread>

#include <cv_bridge/cv_bridge.h>


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

void VideoPlayer::seekToTime(float time)
{
    // Set current frame to the first frame after the time
    for (int i = 0; i < messages_.size(); i++)
    {
        if (messages_[i]->getTime().toSec() >= time)
        {
            current_frame_ = i;
            break;
        }
    }
}

void VideoPlayer::playback()
{
    if (is_playing_ && current_frame_ < messages_.size())
    {
        if (messages_[current_frame_]->isType<sensor_msgs::Image>())
        {
            processImageMessage(messages_[current_frame_]->instantiate<sensor_msgs::Image>());
        }
        else if (messages_[current_frame_]->isType<sensor_msgs::CompressedImage>())
        {
            processCompressedImageMessage(messages_[current_frame_]->instantiate<sensor_msgs::CompressedImage>());
        }
        // emit currentTimestamp(messages_[current_frame_]->getTime().toSec() - start_time_);
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

void VideoPlayer::loadMessages(std::vector<std::shared_ptr<rosbag::MessageInstance>> messages)
{
    current_frame_ = 0;
    messages_ = messages;
    start_time_ = messages[0]->getTime().toSec();
    end_time_ = messages[messages.size() - 1]->getTime().toSec();
    // Process first frame
    if (messages_.size() > 0)
    {
        // Process first frame
        if (messages_[0]->isType<sensor_msgs::Image>())
        {
            processImageMessage(messages_[0]->instantiate<sensor_msgs::Image>());
        }
        else if (messages_[0]->isType<sensor_msgs::CompressedImage>())
        {
            processCompressedImageMessage(messages_[0]->instantiate<sensor_msgs::CompressedImage>());
        }
    }
}

void VideoPlayer::processImageMessage(const sensor_msgs::Image::ConstPtr &msg)
{
    if (msg != nullptr)
    {
        emit newFrame(QImage(msg->data.data(), msg->width, msg->height, QImage::Format_RGB888));
    }
}

void VideoPlayer::processCompressedImageMessage(const sensor_msgs::CompressedImage::ConstPtr &msg)
{
    if (msg != nullptr)
    {
        emit newFrame(QImage::fromData(msg->data.data(), msg->data.size(), "JPEG"));
    }
}