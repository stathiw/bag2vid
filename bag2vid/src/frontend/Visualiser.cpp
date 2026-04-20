
#include "bag2vid/frontend/Visualiser.hpp"

#include <iostream>
#include <thread>

#include <QAbstractItemView>
#include <QDoubleValidator>
#include <QFile>
#include <QFileInfo>
#include <QLineEdit>
#include <QPainter>
#include <QMouseEvent>
#include <QFileDialog>
#include <QtConcurrent>
#include <QPixmap>


namespace bag2vid
{

namespace {
QString formatTime(double seconds)
{
    const int total = static_cast<int>(seconds);
    const int mins = total / 60;
    const int secs = total % 60;
    return QString("%1:%2").arg(mins).arg(secs, 2, 10, QChar('0'));
}
}

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
    this->setMinimumSize(820, 560);
    QFile styleFile(":/theme.qss");
    if (styleFile.open(QFile::ReadOnly | QFile::Text))
    {
        this->setStyleSheet(styleFile.readAll());
        styleFile.close();
    }

    // --- Widget creation ---

    // Header: pre-rendered mark PNG (the SVG's clipPath isn't honored by Qt's SVG
    // renderer; Inkscape renders the circular clip correctly at export time).
    logo_label_ = new QLabel(this);
    {
        constexpr int kMarkSize = 36;
        const qreal dpr = this->devicePixelRatioF();
        const QString path = (dpr > 1.5)
            ? QStringLiteral(":/logo/bag2vid-mark-dark-72.png")
            : QStringLiteral(":/logo/bag2vid-mark-dark-36.png");
        QPixmap pixmap(path);
        pixmap.setDevicePixelRatio(dpr);
        logo_label_->setPixmap(pixmap);
        logo_label_->setFixedSize(kMarkSize, kMarkSize);
    }

    wordmark_label_ = new QLabel(this);
    wordmark_label_->setTextFormat(Qt::RichText);
    wordmark_label_->setAlignment(Qt::AlignVCenter);
    wordmark_label_->setText(QStringLiteral(
        "<span style='font-family:Fraunces; font-size:26px; font-weight:500;'>"
        "bag<span style='color:#DDB04A;'>2</span>vid</span>"));

    rosbag_filename_label_ = new QLabel("", this);
    rosbag_filename_label_->setTextFormat(Qt::RichText);
    rosbag_filename_label_->setStyleSheet("font-family: 'IBM Plex Mono'; font-size: 12px;");

    load_bag_button_ = new QPushButton("Load Bag", this);
    load_bag_button_->setProperty("variant", "primary");

    // Topic strip
    topic_dropdown_ = new QComboBox(this);
    topic_dropdown_->setMinimumWidth(300);

    // Video pane
    image_label_ = new QLabel(this);
    image_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    image_label_->setMinimumSize(320, 240);
    image_label_->setAlignment(Qt::AlignCenter);
    image_label_->setStyleSheet("background-color: #1A1E14; border-radius: 10px;");

    // Transport controls
    play_pause_button_ = new QPushButton("Play", this);
    play_pause_button_->setFixedWidth(100);

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

    // Style the combo box popup container (QComboBoxPrivateContainer is a QFrame
    // wrapping the item view). QSS selectors can't reach it, so set its stylesheet
    // directly — otherwise the system chrome bleeds through as white bars above
    // and below the list view.
    auto style_combo_popup = [](QComboBox* combo) {
        if (auto* popup = combo->view()->parentWidget()) {
            popup->setStyleSheet(
                "background: #262A1C; border: 1px solid #3D4220; border-radius: 8px;");
        }
    };
    style_combo_popup(topic_dropdown_);
    style_combo_popup(playback_rate_combo_);

    timeline_widget_ = new TimelineWidget(this);

    const QString kTimeLabelStyle =
        "font-family: 'IBM Plex Mono'; font-size: 11px; color: #A8A18C;";
    bag_start_label_ = new QLabel("0:00", this);
    bag_start_label_->setStyleSheet(kTimeLabelStyle);
    bag_end_label_ = new QLabel("0:00", this);
    bag_end_label_->setStyleSheet(kTimeLabelStyle);

    // Export tray
    extract_video_button_ = new QPushButton("Extract Video", this);
    capture_screenshot_button_ = new QPushButton("Capture Screenshot", this);

    // Status bar (extraction progress)
    extraction_progress_bar_ = new QProgressBar(this);
    extraction_progress_bar_->setMinimum(0);
    extraction_progress_bar_->setMaximum(100);
    extraction_progress_bar_->setValue(0);
    extraction_progress_bar_->setTextVisible(false);
    extraction_progress_bar_->setFixedHeight(6);

    status_label_ = new QLabel("idle", this);
    status_label_->setStyleSheet(
        "font-family: 'IBM Plex Mono'; font-size: 11px; color: #A8A18C; letter-spacing: 1px;");
    status_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    {
        QFont status_font = status_label_->font();
        status_font.setCapitalization(QFont::AllUppercase);
        status_label_->setFont(status_font);
    }

    rosbag_filename_label_->setText(
        QStringLiteral("<span style='color:#A8A18C'>no rosbag loaded</span>"));

    // Non-UI
    video_player_ = new VideoPlayer(this);
    clock_ = new PlaybackClock(this);

    // --- Layout composition (top-down) ---

    auto* main_layout = new QVBoxLayout(this);
    main_layout->setContentsMargins(18, 16, 18, 12);
    main_layout->setSpacing(12);

    // App header: mark | wordmark | file path (stretch) | Load Bag
    auto* app_header = new QHBoxLayout();
    app_header->setSpacing(10);
    app_header->addWidget(logo_label_);
    app_header->addWidget(wordmark_label_);
    app_header->addSpacing(16);
    app_header->addWidget(rosbag_filename_label_, /*stretch=*/1);
    app_header->addWidget(load_bag_button_);
    main_layout->addLayout(app_header);

    // Topic strip: centered
    auto* topic_strip = new QHBoxLayout();
    topic_strip->addStretch(1);
    topic_strip->addWidget(topic_dropdown_);
    topic_strip->addStretch(1);
    main_layout->addLayout(topic_strip);

    // Video pane (hero)
    main_layout->addWidget(image_label_, /*stretch=*/1);

    // Transport row: play | speed | start | timeline | end
    auto* transport = new QHBoxLayout();
    transport->setSpacing(10);
    transport->addWidget(play_pause_button_);
    transport->addWidget(playback_rate_combo_);
    transport->addWidget(bag_start_label_);
    transport->addWidget(timeline_widget_, /*stretch=*/1);
    transport->addWidget(bag_end_label_);
    main_layout->addLayout(transport);

    // Export tray: status on the left, action buttons on the right
    auto* export_tray = new QHBoxLayout();
    export_tray->addWidget(status_label_);
    export_tray->addStretch(1);
    export_tray->addWidget(capture_screenshot_button_);
    export_tray->addWidget(extract_video_button_);
    main_layout->addLayout(export_tray);

    main_layout->addWidget(extraction_progress_bar_);

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
    // Release old shared_ptrs before resetting extractor_: their deleters
    // reference Reader memory and must not outlive it.
    video_player_->loadMessages({}, "", 0.0);

    // Reset extractor
    extractor_ = std::make_unique<Extractor>();

    // Load rosbag
    if (extractor_->loadBag(rosbag_path.toStdString()))
    {
        std::cout << "Bag loaded successfully" << std::endl;
        QFileInfo info(rosbag_path);
        QString dir = info.absolutePath();
        if (!dir.endsWith('/')) dir += '/';
        rosbag_filename_label_->setText(
            QStringLiteral("<span style='color:#A8A18C'>%1</span>%2")
                .arg(dir.toHtmlEscaped(), info.fileName().toHtmlEscaped()));
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

        bag_start_label_->setText(formatTime(0.0));
        bag_end_label_->setText(formatTime(bag_end - bag_start));
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
    // Reset progress bar and status
    updateProgressBar(0);

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
            updateProgressBar(100);
        }
        else
        {
            std::cout << "Failed to extract video" << std::endl;
            status_label_->setText("extraction failed");
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
    if (progress >= 100)
    {
        status_label_->setText("done");
    }
    else if (progress >= 0)
    {
        status_label_->setText(QString("extracting — %1%").arg(progress));
    }
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
