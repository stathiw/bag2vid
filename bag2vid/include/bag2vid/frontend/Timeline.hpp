/*
 * @file Timeline.hpp
 * @author Stathi Weir (stathi.weir@gmail.com)
 * @brief A class for the timeline widget
 * @version 0.1
 * @date 2024-06-09
 */

#pragma once

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>

namespace bag2vid
{

class TimelineWidget : public QWidget
{
    Q_OBJECT

public:
    TimelineWidget(QWidget *parent = nullptr);
    ~TimelineWidget();

    inline float getStartTime() const { return start_time_; }
    inline float getEndTime() const { return end_time_; }
    inline float getCurrentTime() const { return current_time_; }

    void setStartTime(float time) { start_time_ = time; update(); }
    void setEndTime(float time) { end_time_ = time; update(); }
    void setCurrentTime(float time) { current_time_ = time; update(); }

signals:
    // void startTimeChanged(float time);
    // void endTimeChanged(float time);
    void currentTimeChanged(float time);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    float start_time_;
    float end_time_;
    float current_time_;

    bool dragging_start_;
    bool dragging_end_;
    bool dragging_timeline_;

    void updateMarkerPosition(float& markerPos, int mouseX);
};

} // namespace bag2vid