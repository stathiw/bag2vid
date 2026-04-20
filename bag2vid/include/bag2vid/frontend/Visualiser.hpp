/*
 * @file Visualiser.hpp
 * @author Stathi Weir (stathi.weir@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-09
 */
#pragma once

#include <iostream>
#include <vector>

#include <QFileDialog>
#include <QLabel>
#include <QComboBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSlider>
#include <QPushButton>
#include <QProgressBar>

#include <bag2vid/backend/Extractor.hpp>
#include <bag2vid/frontend/PlaybackClock.hpp>
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

    /**
     * @brief Parse the playback rate combo's current text and apply it to the clock.
     *
     * Expects a numeric value in [0.01, 100]. Invalid or out-of-range input is
     * ignored, leaving the clock at its previous rate.
     */
    void applyPlaybackRateFromCombo();

    /**
     * @brief Handle user selecting a preset from the playback rate dropdown.
     *
     * Replaces the "0.5x" display string in the edit field with the bare number
     * (e.g. "0.5"), then applies it.
     */
    void onPlaybackRatePresetSelected(int index);

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
    // Preset playback rates shown in the rate combo dropdown
    inline static const std::vector<double> kPresetPlaybackRates = {
        0.25, 0.5, 1.0, 2.0, 5.0, 10.0
    };
    // Bounds for custom playback rate entry
    static constexpr double kMinPlaybackRate = 0.01;
    static constexpr double kMaxPlaybackRate = 100.0;

    // Extractor object for extracting video from rosbag
    std::unique_ptr<Extractor> extractor_;

    // GUI elements
    QLabel* logo_label_;
    QLabel* wordmark_label_;
    QLabel* rosbag_filename_label_;
    QPushButton* load_bag_button_;
    QComboBox* topic_dropdown_;
    QPushButton* play_pause_button_;
    QComboBox* playback_rate_combo_;
    QLabel* bag_start_label_;
    QLabel* bag_end_label_;
    QPushButton* extract_video_button_;
    QPushButton* capture_screenshot_button_;
    QProgressBar* extraction_progress_bar_;
    QLabel* status_label_;
    // Timeline widget for showing current playback time and selecting start and end times
    TimelineWidget* timeline_widget_;
    // Video player for displaying the video preview
    VideoPlayer* video_player_;
    // Master playback clock driving video player and timeline
    PlaybackClock* clock_;
    QLabel* image_label_;

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

    /**
     * @brief Kick off extraction on a worker thread.
     *
     * Disables controls that would mutate extractor_ state, runs writeVideo
     * off-thread, and re-enables controls from the completion continuation.
     */
    void startExtraction(const std::string& camera_name,
                         double start_time,
                         double end_time,
                         const std::string& video_path);
};
} // namespace bag2vid
