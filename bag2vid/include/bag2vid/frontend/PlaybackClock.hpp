#pragma once

#include <QObject>
#include <QTimer>

namespace bag2vid
{

/**
 * @brief Master playback clock.
 *
 * Owns the playback time, rate and play/pause state. Emits tick() on a
 * regular cadence so subscribers (video players, timeline, etc.) can render
 * the content appropriate for the current time.
 *
 * Times are bag-relative (0.0 = bag start).
 */
class PlaybackClock : public QObject
{
    Q_OBJECT

public:
    explicit PlaybackClock(QObject *parent = nullptr);
    ~PlaybackClock() override = default;

    inline double getCurrentTime() const { return current_time_; }
    inline double getPlaybackRate() const { return playback_rate_; }
    inline bool isPlaying() const { return is_playing_; }

public slots:
    void play();
    void pause();
    void seek(double time);
    void setPlaybackRate(double rate);
    void setRange(double start_time, double end_time);

signals:
    void tick(double time);
    void finished();

private slots:
    void onTick();

private:
    QTimer timer_;

    bool is_playing_;

    // Relative playback time within bag recording (>= 0.0)
    double current_time_;

    // Start and end timestamps of bag (seconds since epoch)
    double start_timestamp_;
    double end_timestamp_;
    
    // Duration of the bag in seconds (end - start timestamps)
    double duration_;

    // Playback rate multiplier
    // for example: 0.5 = half speed, 1.0 = real time, 2.0 = 2x playback speed
    double playback_rate_ = 1.0;

    // PlaybackClock rate - use 60Hz, typical monitor refresh rate
    static constexpr int kClockTickRate = 60;
    static constexpr int kTickIntervalMs = 1000 / kClockTickRate;
};

} // namespace bag2vid
