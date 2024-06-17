
#include "bag2vid/frontend/Visualiser.hpp"

#include <iostream>


namespace bag2vid
{

Visualiser::Visualiser(QWidget *parent) :
    QMainWindow(parent),
    load_bag_button_(new QPushButton("Load Bag")),
    topic_dropdown_(new QComboBox()),
    play_pause_button_(new QPushButton("Play/Pause")),
    extract_video_button_(new QPushButton("Extract Video")),
    start_time_slider_(new QSlider(Qt::Orientation::Horizontal)),
    end_time_slider_(new QSlider(Qt::Orientation::Horizontal)),
    timeline_slider_(new QSlider(Qt::Orientation::Horizontal)),
    media_player_(new QMediaPlayer()),
    video_widget_(new QVideoWidget())
{
    // Set up the layout
    QWidget *central_widget = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(central_widget);

    QHBoxLayout *top_layout = new QHBoxLayout;
    top_layout->addWidget(load_bag_button_);
    top_layout->addWidget(topic_dropdown_);
    top_layout->addWidget(play_pause_button_);
    top_layout->addWidget(extract_video_button_);

    QHBoxLayout *slider_layout = new QHBoxLayout;
    slider_layout->addWidget(start_time_slider_);
    slider_layout->addWidget(timeline_slider_);
    slider_layout->addWidget(end_time_slider_);

    layout->addLayout(top_layout);
    layout->addLayout(slider_layout);
    layout->addWidget(video_widget_);

    setCentralWidget(central_widget);

    media_player_->setVideoOutput(video_widget_);
}

Visualiser::~Visualiser() {}

} // namespace bag2vid
