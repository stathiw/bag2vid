#include "bag2vid/frontend/PlaybackClock.hpp"

#include <algorithm>

namespace bag2vid
{

PlaybackClock::PlaybackClock(QObject *parent) :
    QObject(parent),
    is_playing_(false),
    current_time_(0.0),
    start_timestamp_(0.0),
    end_timestamp_(0.0),
    duration_(0.0),
    playback_rate_(1.0)
{
    timer_.setTimerType(Qt::PreciseTimer);
    connect(&timer_, &QTimer::timeout, this, &PlaybackClock::onTick);
}

void PlaybackClock::play()
{
    if (is_playing_)
    {
        return;
    }
    if (duration_ <= 0.0)
    {
        return;
    }
    if (current_time_ >= duration_)
    {
        current_time_ = 0.0;
        emit tick(current_time_);
    }
    is_playing_ = true;
    timer_.start(kTickIntervalMs);
}

void PlaybackClock::pause()
{
    is_playing_ = false;
    timer_.stop();
}

void PlaybackClock::seek(double time)
{
    current_time_ = std::clamp(time, 0.0, duration_);
    emit tick(current_time_);
}

void PlaybackClock::setPlaybackRate(double rate)
{
    playback_rate_ = rate;
}

void PlaybackClock::setRange(double start_time, double end_time)
{
    start_timestamp_ = start_time;
    end_timestamp_ = end_time;
    duration_ = end_time - start_time;
    current_time_ = 0.0;
    emit tick(current_time_);
}

void PlaybackClock::onTick()
{
    current_time_ += (kTickIntervalMs / 1000.0) * playback_rate_;
    if (current_time_ >= duration_)
    {
        current_time_ = duration_;
        is_playing_ = false;
        timer_.stop();
        emit tick(current_time_);
        emit finished();
        return;
    }
    emit tick(current_time_);
}

} // namespace bag2vid
