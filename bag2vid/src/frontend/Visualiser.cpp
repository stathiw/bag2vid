
#include "bag2vid/frontend/Visualiser.hpp"

#include <iostream>
#include <thread>

#include <QPainter>
#include <QMouseEvent>
#include <QFileDialog>


namespace bag2vid
{

Visualiser::Visualiser(QWidget *parent) :
    QWidget(parent)
{
    is_playing_ = false;
    setupUI();
}

void Visualiser::setupUI()
{
    // Set up the buttons
    load_bag_button_ = new QPushButton("Load Bag", this);
    topic_dropdown_ = new QComboBox(this);
    play_pause_button_ = new QPushButton("Play", this);
    extract_video_button_ = new QPushButton("Extract Video", this);

    // Set up the timeline widget
    timeline_widget_ = new TimelineWidget(this);

    // Set up the video widget
    video_widget_ = new QVideoWidget(this);
    media_player_ = new QMediaPlayer(this);
    media_player_->setVideoOutput(video_widget_);
    video_widget_->setGeometry(100, 100, 300, 400);
    video_widget_->show();

    // Connect the buttons to their slots
    connect(load_bag_button_, &QPushButton::clicked, this, &Visualiser::loadBag);
    connect(play_pause_button_, &QPushButton::clicked, this, &Visualiser::togglePlayPause);
    connect(extract_video_button_, &QPushButton::clicked, this, &Visualiser::extractVideo);


    // Set up the layout
    QVBoxLayout* main_layout = new QVBoxLayout(this);

    // Menu layout
    QHBoxLayout* top_layout = new QHBoxLayout();
    top_layout->addWidget(load_bag_button_);
    top_layout->addWidget(topic_dropdown_);
    top_layout->addWidget(extract_video_button_);

    // Timeline layout
    QHBoxLayout* timeline_layout = new QHBoxLayout;
    play_pause_button_->setFixedWidth(50);
    timeline_layout->addWidget(play_pause_button_);
    timeline_layout->addWidget(timeline_widget_);

    main_layout->addLayout(top_layout);
    main_layout->addLayout(timeline_layout);
    main_layout->addWidget(video_widget_);

    setLayout(main_layout);
}

Visualiser::~Visualiser() {}

void Visualiser::loadBag()
{
    std::cout << "Load Bag" << std::endl;

    // Load mp4 video
    // QString video_path = "/home/octopus/Downloads/ClipperVHullbot1.mp4";
    QString video_path = QFileDialog::getOpenFileName(this, "Open Video", QDir::homePath(), "Video Files (*.mp4, *.avi, *mpg)");
    media_player_->setMedia(QUrl::fromLocalFile(video_path));
    // Check if the video was loaded successfully
    if (media_player_->mediaStatus() == QMediaPlayer::NoMedia)
    {
        std::cerr << "Error loading video" << std::endl;
        return;
    }
    else if (media_player_->mediaStatus() == QMediaPlayer::LoadedMedia)
    {
        std::cout << "Video loaded successfully" << std::endl;
        video_widget_->show();
        video_widget_->update();
        return;
    }
    else
    {
        std::cout << media_player_->mediaStatus() << std::endl;
    }
}

void Visualiser::togglePlayPause()
{
    if (is_playing_)
    {
        std::cout << "Pause" << std::endl;
        media_player_->pause();
        qDebug() << media_player_->state();
        play_pause_button_->setText("Play");
    }
    else
    {
        std::cout << "Play" << std::endl;
        media_player_->play();
        qDebug() << media_player_->state();
        play_pause_button_->setText("Pause");
    }
    is_playing_ = !is_playing_;
}

void Visualiser::extractVideo()
{
    std::cout << "Extract Video" << std::endl;

    // Get marker positions
    float start_time = timeline_widget_->getStartTime();
    float end_time = timeline_widget_->getEndTime();

    std::cout << "Start time: " << start_time << std::endl;
    std::cout << "End time: " << end_time << std::endl;
}

} // namespace bag2vid
