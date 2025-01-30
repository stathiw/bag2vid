

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>

#include <QImage>
#include <QObject>
#include <QTimer>

class VideoPlayer : public QObject
{
    Q_OBJECT

public:
    explicit VideoPlayer(QObject *parent = nullptr);
    ~VideoPlayer();

    int getCurrentFrameId() const { return current_frame_; }

public slots:
    void seekToTime(double time);
    void play();
    void pause();
    void loadMessages(std::vector<std::shared_ptr<rosbag::MessageInstance> > messages);

signals:
    void newFrame(const QImage &frame);
    void currentTimestamp(double time);
    void finishedPlaying();

private slots:
    void playback();

private:
    bool is_playing_;
    //  Current frame iterator
    int current_frame_;
    double start_time_;
    double end_time_;
    QTimer playback_timer_;
    std::vector<std::shared_ptr<rosbag::MessageInstance> > messages_;

    void processImageMessage(const sensor_msgs::Image::ConstPtr &msg);
    void processCompressedImageMessage(const sensor_msgs::CompressedImage::ConstPtr &msg);
};