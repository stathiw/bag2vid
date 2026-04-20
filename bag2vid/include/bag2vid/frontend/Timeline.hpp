/*
 * @file Timeline.hpp
 * @author Stathi Weir (stathi.weir@gmail.com)
 * @brief A class for the timeline widget
 * @version 0.1
 * @date 2024-06-09
 */

#pragma once

#include <QColor>
#include <QWidget>
#include <QPainter>
#include <QMouseEvent>

namespace bag2vid
{

class TimelineWidget : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(QColor containerColor READ containerColor WRITE setContainerColor)
    Q_PROPERTY(QColor barColor       READ barColor       WRITE setBarColor)
    Q_PROPERTY(QColor startColor     READ startColor     WRITE setStartColor)
    Q_PROPERTY(QColor endColor       READ endColor       WRITE setEndColor)
    Q_PROPERTY(QColor playheadColor  READ playheadColor  WRITE setPlayheadColor)
    Q_PROPERTY(QColor textColor      READ textColor      WRITE setTextColor)

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

    QColor containerColor() const { return container_color_; }
    QColor barColor()       const { return bar_color_; }
    QColor startColor()     const { return start_color_; }
    QColor endColor()       const { return end_color_; }
    QColor playheadColor()  const { return playhead_color_; }
    QColor textColor()      const { return text_color_; }

    void setContainerColor(const QColor& c) { container_color_ = c; update(); }
    void setBarColor(const QColor& c)       { bar_color_ = c; update(); }
    void setStartColor(const QColor& c)     { start_color_ = c; update(); }
    void setEndColor(const QColor& c)       { end_color_ = c; update(); }
    void setPlayheadColor(const QColor& c)  { playhead_color_ = c; update(); }
    void setTextColor(const QColor& c)      { text_color_ = c; update(); }

signals:
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

    QColor container_color_;
    QColor bar_color_;
    QColor start_color_;
    QColor end_color_;
    QColor playhead_color_;
    QColor text_color_;

    void updateMarkerPosition(double& markerPos, int mouseX);
};

} // namespace bag2vid
