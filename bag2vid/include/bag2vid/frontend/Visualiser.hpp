/*
 * @file Visualiser.hpp
 * @author Stathi Weir (stathi.weir@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-09
 */
#pragma once

#include <iostream>

#include <QMainWindow>
#include <QVideoWidget>
#include <QFileDialog>
#include <QLabel>
#include <QComboBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMediaPlayer>
#include <QSlider>
#include <QPushButton>
#include <QProgressBar>

#include <ros/ros.h>

#include <bag2vid/backend/Extractor.hpp>
#include <bag2vid/frontend/Timeline.hpp>
#include <bag2vid/frontend/VideoPlayer.hpp>

namespace bag2vid 
{

class Visualiser : public QWidget
{
    // A class for the gui frontend of the bag2vid node
    // Contains windows for selecting the bag file, topic, camera name, and output file
    // Also contains a window for displaying the video preview
    Q_OBJECT // Required for signals and slots

public:
    Visualiser(QWidget *parent = nullptr);
    ~Visualiser();

private slots:
    /**
     * @brief Load a rosbag file into the video extractor.
     * 
     */
    void loadBag();

    /**
     * @brief Toggle the play/pause state of the video player.
     * 
     */
    void togglePlayPause();
    
    /**
     * @brief Extract the video from the rosbag file for the selected topic between the selected start and end times.
     * 
     */
    void extractVideo();
    
    /**
     * @brief Update the dropdown list of topics from the loaded rosbag file.
     * 
     */
    void updateTopicDropdown();

    /**
     * @brief Capture a screenshot of the current frame.
     * 
     */
    void captureScreenshot();

protected:
    /**
     * @brief Resize event handler.
     * 
     * @param event 
     */
    void resizeEvent(QResizeEvent *event) override;
    
    /**
     * @brief Key press event handler.
     * 
     * @param event 
     */
    void keyPressEvent(QKeyEvent *event) override;

private:
    // Extractor object for extracting video from rosbag
    std::unique_ptr<Extractor> extractor_;

    // GUI elements
    QLabel* rosbag_filename_label_;
    QPushButton* load_bag_button_;
    QComboBox* topic_dropdown_;
    QPushButton* play_pause_button_;
    QPushButton* extract_video_button_;
    QPushButton* capture_screenshot_button_;
    QProgressBar* extraction_progress_bar_;
    // Timeline widget for showing current playback time and selecting start and end times
    TimelineWidget* timeline_widget_;
    // Video player for displaying the video preview
    VideoPlayer* video_player_;
    QThread* thread_;
    QLabel* image_label_;

    bool is_playing_;

    /**
     * @brief Set up the GUI elements.
     * 
     */
    void setupUI();

    /**
     * @brief Update the progress bar 
     * 
     */
    void updateProgressBar(int progress);
};
} // namespace bag2vid

