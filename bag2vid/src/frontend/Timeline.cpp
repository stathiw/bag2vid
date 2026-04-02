#include "bag2vid/frontend/Timeline.hpp"

#include <QPainter>
#include <QMouseEvent>

#include <iostream>

namespace bag2vid
{

TimelineWidget::TimelineWidget(QWidget *parent) :
    QWidget(parent),
    start_time_(0.0),
    end_time_(1.0),
    current_time_(0.0),
    dragging_start_(false),
    dragging_end_(false),
    dragging_timeline_(false)
{
    setMinimumHeight(100);
    setMinimumWidth(500);
    std::cout << "TimelineWidget created" << std::endl;
}


TimelineWidget::~TimelineWidget()
{
    std::cout << "TimelineWidget destroyed" << std::endl;
}

void TimelineWidget::paintEvent(QPaintEvent *event)
{
    // Draw the timeline
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(text_color_);

    int bar_height = 50;        // Height of the timeline bar
    int marker_height = 20;     // Height of the timeline markers

    // Draw the timeline bar
    QPen bar_pen(bar_color_, 2);
    painter.setPen(bar_pen);
    painter.drawLine(10, bar_height, width()-10, bar_height);

    // Draw the start marker (right-pointing triangle)
    painter.setPen(Qt::NoPen);
    painter.setBrush(start_color_);
    int startMarkerX = (start_time_  / duration_) * (width() - 20) + 10;
    int mid = bar_height - marker_height / 2;
    QPolygon start_triangle;
    start_triangle << QPoint(startMarkerX, bar_height - marker_height)
                   << QPoint(startMarkerX + 10, mid)
                   << QPoint(startMarkerX, bar_height);
    painter.drawPolygon(start_triangle);
    painter.setPen(text_color_);
    painter.drawText(startMarkerX - 20, bar_height + 20, QString::number(start_time_, 'f', 2));

    // Draw end marker (left-pointing triangle)
    painter.setPen(Qt::NoPen);
    painter.setBrush(end_color_);
    int endMarkerX = (end_time_ / duration_) * (width() - 20) + 10;
    QPolygon end_triangle;
    end_triangle << QPoint(endMarkerX, bar_height - marker_height)
                 << QPoint(endMarkerX - 10, mid)
                 << QPoint(endMarkerX, bar_height);
    painter.drawPolygon(end_triangle);
    painter.setPen(text_color_);
    painter.drawText(endMarkerX - 20, bar_height + 20, QString::number(end_time_, 'f', 2));

    // Draw current time marker
    painter.setPen(Qt::NoPen);
    painter.setBrush(playhead_color_);
    int currentTimeMarkerX = (current_time_ / duration_) * (width() - 20) + 10;
    painter.drawRoundedRect(currentTimeMarkerX - 2, bar_height - marker_height - 10, 4, marker_height + 20, 2, 2);
    painter.setPen(text_color_);
    painter.drawText(currentTimeMarkerX - 20, bar_height - marker_height - 20, QString::number(current_time_, 'f', 2));
}

void TimelineWidget::mousePressEvent(QMouseEvent *event)
{
    int mouseX = event->x();

    // Check if mouse press is on start marker
    int startMarkerX = (start_time_ / duration_) * (width() - 20) + 10;
    if (mouseX >= startMarkerX - 5 && mouseX <= startMarkerX + 5) {
        dragging_start_ = true;
    }

    // Check if mouse press is on end marker
    int endMarkerX = (end_time_ / duration_) * (width() - 20) + 10;
    if (mouseX >= endMarkerX - 5 && mouseX <= endMarkerX + 5) {
        dragging_end_ = true;
    }

    // Check if mouse press is on current time marker
    int currentTimeMarkerX = (current_time_ / duration_) * (width() - 20) + 10;
    if (mouseX >= currentTimeMarkerX - 2 && mouseX <= currentTimeMarkerX + 2) {
        dragging_timeline_ = true;
    }
}

void TimelineWidget::mouseMoveEvent(QMouseEvent* event) {
    if (dragging_start_) {
        updateMarkerPosition(start_time_, event->x());
        update();
    } else if (dragging_end_) {
        updateMarkerPosition(end_time_, event->x());
        update();
    } else if (dragging_timeline_) {
        updateMarkerPosition(current_time_, event->x());
        update();
        emit currentTimeChanged(current_time_);
    }
}

void TimelineWidget::mouseReleaseEvent(QMouseEvent *event)
{
    dragging_start_ = false;
    dragging_end_ = false;
    dragging_timeline_ = false;
}

void TimelineWidget::updateMarkerPosition(double& markerPos, int mouseX)
{
    markerPos = static_cast<double>(mouseX - 10) / (width() - 20);
    if (markerPos < 0.0) markerPos = 0.0;
    if (markerPos > 1.0) markerPos = 1.0;
    markerPos = markerPos * duration_;
}

} // namespace bag2vid
