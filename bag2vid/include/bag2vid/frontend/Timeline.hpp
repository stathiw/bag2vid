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

    inline double getBagStartTime() const { return start_bag_time_; }
    inline double getBagEndTime() const { return end_bag_time_; }

    inline double getStartTime() const { return start_time_; }
    inline double getEndTime() const { return end_time_; }
    inline double getCurrentTime() const { return current_time_; }

    void setBagStartTime(double time) { start_bag_time_ = time; duration_ = end_bag_time_-start_bag_time_; update(); }
    void setBagEndTime(double time) { end_bag_time_ = time; duration_ = end_bag_time_-start_bag_time_; update(); }

    void setStartTime(double time) { start_time_ = time; update(); }
    void setEndTime(double time) { end_time_ = time; update(); }
    void setCurrentTime(double time) { current_time_ = time; update(); }

signals:
    // void startTimeChanged(float time);
    // void endTimeChanged(float time);
    void currentTimeChanged(double time);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    double start_bag_time_;
    double end_bag_time_;
    double duration_;

    double start_time_;
    double end_time_;
    double current_time_;

    bool dragging_start_;
    bool dragging_end_;
    bool dragging_timeline_;

    void updateMarkerPosition(double& markerPos, int mouseX);
};

} // namespace bag2vid