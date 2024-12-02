
#include "bag2vid/frontend/Visualiser.hpp"

#include <iostream>
#include <thread>

#include <QPainter>
#include <QMouseEvent>
#include <QFileDialog>
#include <QThread>


namespace bag2vid
{

Visualiser::Visualiser(QWidget *parent) :
    QWidget(parent)
{
    is_playing_ = false;
    extractor_ = std::make_unique<Extractor>();
    setupUI();

    // Connect the buttons to their slots
    connect(load_bag_button_, &QPushButton::clicked, this, &Visualiser::loadBag);
    connect(play_pause_button_, &QPushButton::clicked, this, &Visualiser::togglePlayPause);
    connect(extract_video_button_, &QPushButton::clicked, this, &Visualiser::extractVideo);
    connect(topic_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(updateTopicDropdown()));

    connect(video_player_, &VideoPlayer::newFrame, [this](const QImage& frame)
    {
        QSize label_size = image_label_->size();

        QImage resized_frame = frame.scaled(label_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        image_label_->setPixmap(QPixmap::fromImage(resized_frame));
    });

    connect(video_player_, &VideoPlayer::currentTimestamp, [this](float time)
    {
        // std::cout << "Current time: " << time << std::endl;
        timeline_widget_->setCurrentTime(time);
    });

    connect(timeline_widget_, &TimelineWidget::currentTimeChanged, [this](double time)
    {
        // std::cout << "Seek to time: " << time << std::endl;
        video_player_->seekToTime(time);
    });
}

void Visualiser::setupUI()
{
    this->setMinimumSize(640, 480);

    // Set up the buttons
    load_bag_button_ = new QPushButton("Load Bag", this);
    topic_dropdown_ = new QComboBox(this);
    play_pause_button_ = new QPushButton("Play", this);
    extract_video_button_ = new QPushButton("Extract Video", this);

    // Set up the timeline widget
    timeline_widget_ = new TimelineWidget(this);

    // Set up the video widget
    video_player_ = new VideoPlayer(this);
    image_label_ = new QLabel(this);
    image_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    image_label_->setMinimumSize(320, 240);
    image_label_->setMaximumSize(1920, 1080);

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
    // Don't allow the timeline to stretch vertically if the window is resized
    timeline_layout->setAlignment(Qt::AlignTop);

    // Video layout
    QVBoxLayout* video_layout = new QVBoxLayout;
    video_layout->addWidget(image_label_);
    // Allow the video to stretch to fill the available space
    video_layout->setAlignment(Qt::AlignCenter);

    main_layout->addLayout(top_layout);
    main_layout->addLayout(timeline_layout);
    main_layout->addLayout(video_layout);

    setLayout(main_layout);
}

Visualiser::~Visualiser() {}

void Visualiser::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);

    if (image_label_->pixmap() != nullptr && !image_label_->pixmap()->isNull())
    {
        QSize label_size = image_label_->size();
        QImage resized_frame = image_label_->pixmap()->toImage().scaled(label_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        image_label_->setPixmap(QPixmap::fromImage(resized_frame));
    }
}

void Visualiser::loadBag()
{
    std::cout << "Load Bag" << std::endl;

    // Reset extractor
    extractor_ = std::make_unique<Extractor>();

    // Select rosbag file
    QString rosbag_path = QFileDialog::getOpenFileName(this, "Open rosbag", QDir::homePath(), "Ros bag files (*.bag)");
    
    // Load rosbag
    if (extractor_->loadBag(rosbag_path.toStdString()))
    {
        std::cout << "Bag loaded successfully" << std::endl;
        topic_dropdown_->clear();

        // Get topics
        std::vector<std::string> topics = extractor_->getImageTopics();
        for (const auto& topic : topics)
        {
            topic_dropdown_->addItem(QString::fromStdString(topic));
        }
        // Set start and end time of timeline
        timeline_widget_->setBagStartTime(extractor_->getBagStartTime());
        timeline_widget_->setBagEndTime(extractor_->getBagEndTime());
        timeline_widget_->setStartTime(0.0);
        timeline_widget_->setEndTime(extractor_->getBagEndTime() - extractor_->getBagStartTime());
    }
    else
    {
        std::cout << "Failed to load bag" << std::endl;
    }
}

void Visualiser::updateTopicDropdown()
{
    std::cout << "Update Topic Dropdown" << std::endl;
    std::string current_topic = topic_dropdown_->currentText().toStdString();
    // Check dropdown is not empty
    if (current_topic.empty())
    {
        return;
    }
    std::cout << "selected topic: " << current_topic << std::endl;

    // Load messages for the selected topic
    // camera name is first part of topic name (eg. /camera_2/image_raw_relay/compressed -> camera_2)
    std::string camera_name = current_topic.substr(1, current_topic.find("/", 1) - 1);
    
    std::cout << "Extracting messages for camera: " << camera_name << std::endl;
    std::vector<std::shared_ptr<rosbag::MessageInstance>> messages =
      extractor_->extractMessages(topic_dropdown_->currentText().toStdString(), camera_name);
    // Load messages into video player
    video_player_->loadMessages(messages);
    std::cout << "Messages extracted" << std::endl;
}

void Visualiser::togglePlayPause()
{
    if (is_playing_)
    {
        std::cout << "Pause" << std::endl;
        video_player_->pause();
        play_pause_button_->setText("Play");
    }
    else
    {
        std::cout << "Play" << std::endl;
        video_player_->play();
        play_pause_button_->setText("Pause");
    }
    is_playing_ = !is_playing_;
}

void Visualiser::extractVideo()
{
    std::cout << "Extract Video" << std::endl;

    // Get marker positions
    double start_time = timeline_widget_->getStartTime() + extractor_->getBagStartTime();
    double end_time = timeline_widget_->getEndTime() + extractor_->getBagStartTime();

    std::cout << "Start time: " << start_time << std::endl;
    std::cout << "End time: " << end_time << std::endl;

    // Check start time is before end time
    if (start_time >= end_time)
    {
        std::cout << "Invalid start and end times" << std::endl;
        return;
    }

    // Select output file
    QString video_path = QFileDialog::getSaveFileName(this, "Save video", QDir::homePath(), "Video files (*.mp4)");
    // No file specified, cancel extraction
    if (video_path.isEmpty())
    {
        return;
    }
    std::cout << "Video path: " << video_path.toStdString() << std::endl;

    // Extract video
    std::string camera_name = topic_dropdown_->currentText().toStdString().substr(1, topic_dropdown_->currentText().toStdString().find("/", 1) - 1);

    if (extractor_->writeVideo(camera_name, ros::Time(start_time), ros::Time(end_time), video_path.toStdString()))
    {
        std::cout << "Video extracted successfully" << std::endl;
    }
    else
    {
        std::cout << "Failed to extract video" << std::endl;
    }
}

} // namespace bag2vid
