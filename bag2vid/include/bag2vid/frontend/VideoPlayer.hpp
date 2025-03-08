

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

public slots:
    /**
     * @brief Move the video player to the specified time (in seconds).
     * First frame of the rosbag is at time 0.
     * 
     * @param time
     */
    void seekToTime(double time);

    /**
     * @brief Start playing the video.
     */
    void play();

    /**
     * @brief Pause the video.
     */
    void pause();

    /**
     * @brief Load messages from a rosbag.
     * 
     * @param messages Vector of shared pointers to rosbag message instances.
     */
    void loadMessages(std::vector<std::shared_ptr<rosbag::MessageInstance> > messages);

    /**
     * @brief Move the video player to the previous frame.
     */
    void seekBackward();

    /**
     * @brief Move the video player to the next frame.
     */
    void seekForward();

signals:
    /**
     * @brief Signal emitted when a new frame is available.
     * 
     * @param frame
     */
    void newFrame(const QImage &frame);

    /**
     * @brief Set the timestamp of the current frame.
     * 
     * @param time 
     * @return * void 
     */
    void currentTimestamp(double time);

    /**
     * @brief Signal emitted when the video has finished playing.
     * 
     */
    void finishedPlaying();

private slots:
    void playback();

private:
    //  Flag to indicate if the video is playing
    bool is_playing_;
    //  Current frame iterator
    int current_frame_;
    // Time of start-point marker of video extraction
    double start_time_;
    // Time of end-point marker of video extraction
    double end_time_;
    // Timer for playback
    QTimer playback_timer_;
    // Vector of shared pointers to rosbag message instances
    std::vector<std::shared_ptr<rosbag::MessageInstance> > messages_;

    /**
     * @brief Process ROS Image message.
     * 
     * @param msg std::shared_ptr to the Image message.
     */
    void processImageMessage(const sensor_msgs::Image::ConstPtr &msg);

    /**
     * @brief Process ROS CompressedImage message.
     * 
     * @param msg std::shared_ptr to the CompressedImage message.
     */
    void processCompressedImageMessage(const sensor_msgs::CompressedImage::ConstPtr &msg);
};