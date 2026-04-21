
#include "bag2vid/frontend/Visualiser.hpp"

#include <iostream>
#include <thread>

#include <QDoubleValidator>
#include <QFile>
#include <QLineEdit>
#include <QPainter>
#include <QMouseEvent>
#include <QFileDialog>
#include <QtConcurrent>


namespace bag2vid
{

Visualiser::Visualiser(QWidget *parent) :
    QWidget(parent)
{
    extractor_ = std::make_unique<Extractor>();
    setupUI();

    // Connect the buttons to their slots
    connect(load_bag_button_, &QPushButton::clicked, this, &Visualiser::loadBag);
    connect(play_pause_button_, &QPushButton::clicked, this, &Visualiser::togglePlayPause);
    connect(extract_video_button_, &QPushButton::clicked, this, &Visualiser::extractVideo);
    connect(topic_dropdown_, &QComboBox::currentIndexChanged, this, &Visualiser::updateTopicDropdown);
    connect(capture_screenshot_button_, &QPushButton::clicked, this, &Visualiser::captureScreenshot);

    connect(video_player_, &VideoPlayer::newFrame, [this](const QImage& frame)
    {
        QSize label_size = image_label_->size();

        QImage resized_frame = frame.scaled(label_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        image_label_->setPixmap(QPixmap::fromImage(resized_frame));
    });

    // Clock drives both the video player and the timeline playhead
    connect(clock_, &PlaybackClock::tick, video_player_, &VideoPlayer::onClockTick);
    connect(clock_, &PlaybackClock::tick, timeline_widget_, &TimelineWidget::setCurrentTime);
    connect(clock_, &PlaybackClock::finished, this, [this]()
    {
        play_pause_button_->setText("Play");
    });

    connect(timeline_widget_, &TimelineWidget::currentTimeChanged, clock_, &PlaybackClock::seek);

    // Playback rate: commit on Enter/focus-out for typed input, or on dropdown selection
    connect(playback_rate_combo_->lineEdit(), &QLineEdit::editingFinished,
            this, &Visualiser::applyPlaybackRateFromCombo);
    connect(playback_rate_combo_, &QComboBox::activated,
            this, &Visualiser::onPlaybackRatePresetSelected);
}

void Visualiser::setupUI()
{
    this->setMinimumSize(640, 480);
    QFile styleFile(":/theme.qss");
    if (styleFile.open(QFile::ReadOnly | QFile::Text))
    {
        this->setStyleSheet(styleFile.readAll());
        styleFile.close();
    }

    // Set up the buttons
    load_bag_button_ = new QPushButton("Load Bag", this);
    topic_dropdown_ = new QComboBox(this);
    play_pause_button_ = new QPushButton("Play", this);
    extract_video_button_ = new QPushButton("Extract Video", this);
    capture_screenshot_button_ = new QPushButton("Capture Screenshot", this);

    // Playback rate combo: editable with presets + free-form numeric entry.
    // The "x" suffix is display-only; typing is restricted to numbers by the validator.
    // NoInsert prevents Qt from auto-inserting typed values as new items (which
    // would otherwise fire activated() with an index past kPresetPlaybackRates).
    playback_rate_combo_ = new QComboBox(this);
    playback_rate_combo_->setEditable(true);
    playback_rate_combo_->setInsertPolicy(QComboBox::NoInsert);
    // Disable inline completion so typed input isn't auto-filled to a matching preset.
    playback_rate_combo_->setCompleter(nullptr);
    for (double rate : kPresetPlaybackRates)
    {
        playback_rate_combo_->addItem(QString::number(rate) + "x");
    }
    playback_rate_combo_->setCurrentText("1");
    playback_rate_combo_->setFixedWidth(80);
    // Validator enforces numeric-only input; range is enforced by the handler so
    // that editingFinished still fires for out-of-range input (allowing revert).
    auto *rate_validator = new QDoubleValidator(this);
    rate_validator->setDecimals(2);
    rate_validator->setNotation(QDoubleValidator::StandardNotation);
    playback_rate_combo_->setValidator(rate_validator);

    // Set up the timeline widget
    timeline_widget_ = new TimelineWidget(this);

    // Set up the video widget
    video_player_ = new VideoPlayer(this);

    // Set up the playback clock
    clock_ = new PlaybackClock(this);
    image_label_ = new QLabel(this);
    image_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    image_label_->setMinimumSize(320, 240);
    image_label_->setMaximumSize(1920, 1080);

    // Set up the layout
    QVBoxLayout* main_layout = new QVBoxLayout(this);

    // Header layout
    QHBoxLayout* header_layout = new QHBoxLayout();
    rosbag_filename_label_ = new QLabel("", this);
    header_layout->addWidget(rosbag_filename_label_);

    // Menu layout
    QHBoxLayout* top_layout = new QHBoxLayout();
    top_layout->addWidget(load_bag_button_);
    top_layout->addWidget(topic_dropdown_);
    top_layout->addWidget(extract_video_button_);
    top_layout->addWidget(capture_screenshot_button_);

    // Video extraction progress bar
    QHBoxLayout* progress_layout = new QHBoxLayout();
    extraction_progress_bar_ = new QProgressBar(this);
    extraction_progress_bar_->setMinimum(0);
    extraction_progress_bar_->setMaximum(100);
    extraction_progress_bar_->setValue(0);
    extraction_progress_bar_->setTextVisible(true);
    progress_layout->addWidget(extraction_progress_bar_);

    // Timeline layout
    QHBoxLayout* timeline_layout = new QHBoxLayout;
    play_pause_button_->setFixedWidth(100);
    timeline_layout->addWidget(play_pause_button_);
    timeline_layout->addWidget(playback_rate_combo_);
    timeline_layout->addWidget(timeline_widget_);
    // Don't allow the timeline to stretch vertically if the window is resized
    timeline_layout->setAlignment(Qt::AlignTop);

    // Video layout
    QVBoxLayout* video_layout = new QVBoxLayout;
    video_layout->addWidget(image_label_);
    // Allow the video to stretch to fill the available space
    video_layout->setAlignment(Qt::AlignCenter);

    main_layout->addLayout(progress_layout);
    main_layout->addLayout(header_layout);
    main_layout->addLayout(top_layout);
    main_layout->addLayout(timeline_layout);
    main_layout->addLayout(video_layout);

    setLayout(main_layout);
}

Visualiser::~Visualiser() {}

void Visualiser::keyPressEvent(QKeyEvent *event)
{
    // Space bar toggles play/pause
    if (event->key() == Qt::Key_Space)
    {
        togglePlayPause();
    }
    // Left moves current frame back 1 frame
    else if (event->key() == Qt::Key_Left)
    {
        if (clock_->isPlaying())
        {
            togglePlayPause();
        }
        double t = video_player_->prevFrameTime();
        if (t >= 0.0)
        {
            clock_->seek(t);
        }
    }
    // Right arrow seeks forward 1 frame
    else if (event->key() == Qt::Key_Right)
    {
        if (clock_->isPlaying())
        {
            togglePlayPause();
        }
        double t = video_player_->nextFrameTime();
        if (t >= 0.0)
        {
            clock_->seek(t);
        }
    }
}

void Visualiser::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);

    if (!image_label_->pixmap().isNull())
    {
        QSize label_size = image_label_->size();
        QImage resized_frame = image_label_->pixmap().toImage().scaled(label_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        image_label_->setPixmap(QPixmap::fromImage(resized_frame));
    }
}

void Visualiser::loadBag()
{
    std::cout << "Load Bag" << std::endl;

    // Pause video player if playing
    if (clock_->isPlaying())
    {
        togglePlayPause();
    }

    // Select rosbag file
    QString rosbag_path = QFileDialog::getOpenFileName(this, "Open rosbag", QDir::homePath(), "Rosbag files (*.bag *.mcap)");

    // No file specified, cancel load
    if (rosbag_path.isEmpty())
    {
        return;
    }
    // Reset extractor
    extractor_ = std::make_unique<Extractor>();

    // Load rosbag
    if (extractor_->loadBag(rosbag_path.toStdString()))
    {
        std::cout << "Bag loaded successfully" << std::endl;
        rosbag_filename_label_->setText("<b>" + rosbag_path + "</b>");
        topic_dropdown_->clear();

        // Get topics
        std::vector<std::string> topics = extractor_->getImageTopics();
        // Sort topics
        std::sort(topics.begin(), topics.end());
        for (const auto& topic : topics)
        {
            topic_dropdown_->addItem(QString::fromStdString(topic));
        }
        // Populating the dropdown triggers updateTopicDropdown, which calls
        // extractMessages as a side effect -- that's what actually populates
        // bag_start_time_sec_ / bag_end_time_sec_ in the extractor. So we must
        // read them out AFTER the dropdown is populated.
        double bag_start = extractor_->getBagStartTime();
        double bag_end = extractor_->getBagEndTime();
        timeline_widget_->setBagStartTime(bag_start);
        timeline_widget_->setBagEndTime(bag_end);
        timeline_widget_->setStartTime(0.0);
        timeline_widget_->setEndTime(bag_end - bag_start);
        clock_->setRange(bag_start, bag_end);
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
    std::vector<bag2vid::MessageInstancePtr> messages =
      extractor_->extractMessages(topic_dropdown_->currentText().toStdString(), camera_name);

    // Look up the message type for this topic
    std::string message_type = extractor_->getTopicType(current_topic);

    // Load messages into video player
    video_player_->loadMessages(messages, message_type, extractor_->getBagStartTime());
    // Re-render the frame for the current clock time so switching topics while
    // paused shows the equivalent moment, not the first frame of the new topic.
    video_player_->onClockTick(clock_->getCurrentTime());
    std::cout << "Messages extracted" << std::endl;
}

void Visualiser::togglePlayPause()
{
    if (clock_->isPlaying())
    {
        std::cout << "Pause" << std::endl;
        clock_->pause();
        play_pause_button_->setText("Play");
    }
    else
    {
        std::cout << "Play" << std::endl;
        clock_->play();
        // play() is a no-op if no bag is loaded; only flip the label if it actually started
        if (clock_->isPlaying())
        {
            play_pause_button_->setText("Pause");
        }
    }
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
    if (!video_path.endsWith(".mp4"))
    {
        video_path += ".mp4";
    }
    std::cout << "Video path: " << video_path.toStdString() << std::endl;

    // Extract video
    std::string camera_name;
    if (!topic_dropdown_->currentText().isEmpty()) {
        camera_name = topic_dropdown_->currentText().toStdString().substr(1, topic_dropdown_->currentText().toStdString().find("/", 1) - 1);
    } else {
        std::cout << "No topic selected" << std::endl;
        return;
    }

    startExtraction(camera_name, start_time, end_time, video_path.toStdString());
}

void Visualiser::startExtraction(const std::string& camera_name,
                                 double start_time,
                                 double end_time,
                                 const std::string& video_path)
{
    extraction_progress_bar_->setValue(0);

    // Progress fires from the worker; marshal back to the GUI thread before touching widgets
    extractor_->setProgressCallback([this](int progress)
    {
        QMetaObject::invokeMethod(this, [this, progress]()
        {
            updateProgressBar(progress);
        }, Qt::QueuedConnection);
    });

    // Block controls that would mutate extractor_ mid-run
    extract_video_button_->setEnabled(false);
    load_bag_button_->setEnabled(false);
    topic_dropdown_->setEnabled(false);

    // Capture by value: the worker outlives this function, so locals would dangle if captured by reference
    QtConcurrent::run([this, camera_name, start_time, end_time, video_path]()
    {
        return extractor_->writeVideo(camera_name, start_time, end_time, video_path);
    }).then(this, [this](bool success)
    {
        if (success)
        {
            std::cout << "Video extracted successfully" << std::endl;
            extraction_progress_bar_->setValue(100);
        }
        else
        {
            std::cout << "Failed to extract video" << std::endl;
        }
        extract_video_button_->setEnabled(true);
        load_bag_button_->setEnabled(true);
        topic_dropdown_->setEnabled(true);
    });
}

void Visualiser::captureScreenshot()
{
    std::cout << "Capture Screenshot" << std::endl;

    // Select output file
    QString screenshot_path = QFileDialog::getSaveFileName(this, "Save screenshot", QDir::homePath(), "Image files (*.png)");
    // No file specified, cancel extraction
    if (screenshot_path.isEmpty())
    {
        return;
    }
    // If no .png extension, add it
    if (!screenshot_path.endsWith(".png"))
    {
        screenshot_path += ".png";
    }

    std::cout << "Screenshot path: " << screenshot_path.toStdString() << std::endl;

    // Get id of current frame
    int frame_id = video_player_->getCurrentFrameId();

    // Get camera name
    std::string camera_name;
    if (!topic_dropdown_->currentText().isEmpty()) {
        camera_name = topic_dropdown_->currentText().toStdString().substr(1, topic_dropdown_->currentText().toStdString().find("/", 1) - 1);
    } else {
        std::cout << "No topic selected" << std::endl;
        return;
    }
    std::cout << "Camera name: " << camera_name << std::endl;

    // Capture screenshot
    if (extractor_->captureScreenshot(camera_name, frame_id, screenshot_path.toStdString()))
    {
        std::cout << "Screenshot captured successfully" << std::endl;
    }
    else
    {
        std::cout << "Failed to capture screenshot" << std::endl;
    }
}

void Visualiser::updateProgressBar(int progress)
{
    extraction_progress_bar_->setValue(progress);
}

void Visualiser::applyPlaybackRateFromCombo()
{
    // Read the current text from playback_rate_combo_, check if numeric value in valid range [kMinPlaybackRate, kMaxPlaybackRate]

    // Get current text in playback_rate_combo_
    QString raw_input = playback_rate_combo_->currentText().trimmed();

    // Check is valid input
    bool is_numeric = false;
    double rate = raw_input.toDouble(&is_numeric);

    // Update if valid input; otherwise revert the field to the current rate
    if (is_numeric && rate >= kMinPlaybackRate && rate <= kMaxPlaybackRate)
    {
        clock_->setPlaybackRate(rate);
    }
    else
    {
        playback_rate_combo_->setEditText(QString::number(clock_->getPlaybackRate()));
    }
}

void Visualiser::onPlaybackRatePresetSelected(int index)
{
    playback_rate_combo_->setEditText(QString::number(kPresetPlaybackRates[index]));
    applyPlaybackRateFromCombo();   
}

} // namespace bag2vid
