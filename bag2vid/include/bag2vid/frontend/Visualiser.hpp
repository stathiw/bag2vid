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

#include <ros/ros.h>

#include <bag2vid/backend/Extractor.hpp>

namespace bag2vid 
{

class Visualiser : public QMainWindow
{
    // A class for the gui frontend of the bag2vid node
    // Contains windows for selecting the bag file, topic, camera name, and output file
    // Also contains a window for displaying the video preview
    Q_OBJECT // Required for signals and slots

public:
    Visualiser(QWidget *parent = nullptr);
    ~Visualiser();

    // bool loadRosbag();
    // void playPauseVideo();
    // bool extractVideo();

private:
    std::unique_ptr<Extractor> extractor_;

    QPushButton* load_bag_button_;
    QComboBox* topic_dropdown_;
    QPushButton* play_pause_button_;
    QPushButton* extract_video_button_;
    QSlider* start_time_slider_;
    QSlider* end_time_slider_;
    QSlider* timeline_slider_;
    QMediaPlayer* media_player_;
    QVideoWidget* video_widget_;
};
} // namespace bag2vid

