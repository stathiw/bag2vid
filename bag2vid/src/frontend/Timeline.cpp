#include "bag2vid/frontend/Timeline.hpp"

#include <QFont>
#include <QFontMetrics>
#include <QPainter>
#include <QMouseEvent>

#include <iostream>

namespace bag2vid
{

namespace {
constexpr int kPlayheadTextZone = 18;  // text area above container
constexpr int kMarkerLabelZone = 18;   // text area below container (start/end marker times)
constexpr int kHPad = 16;              // horizontal padding inside container
constexpr int kTriHalf = 7;            // marker triangle half-height
constexpr int kTriW = 10;              // marker triangle horizontal length
constexpr int kPlayheadInset = 6;      // vertical inset of playhead from container edges
constexpr int kContainerRadius = 10;
constexpr int kHitTolerance = 8;

int timeToX(double time, double duration, int widget_width)
{
    if (duration <= 0.0) return kHPad;
    return static_cast<int>((time / duration) * (widget_width - 2 * kHPad) + kHPad);
}
}

TimelineWidget::TimelineWidget(QWidget *parent) :
    QWidget(parent),
    start_time_(0.0),
    end_time_(1.0),
    current_time_(0.0),
    dragging_start_(false),
    dragging_end_(false),
    dragging_timeline_(false)
{
    setMinimumHeight(80);
    setMinimumWidth(300);
    std::cout << "TimelineWidget created" << std::endl;
}


TimelineWidget::~TimelineWidget()
{
    std::cout << "TimelineWidget destroyed" << std::endl;
}

void TimelineWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    const int container_top = kPlayheadTextZone;
    const int container_bottom = height() - kMarkerLabelZone;
    const int track_y = container_top + (container_bottom - container_top) / 2;

    // Container background — rounded rect, Kelp night by default
    painter.setPen(Qt::NoPen);
    painter.setBrush(container_color_);
    painter.drawRoundedRect(
        QRectF(0, container_top, width(), container_bottom - container_top),
        kContainerRadius, kContainerRadius);

    // Track line — thin horizontal across the container
    painter.setPen(QPen(bar_color_, 2));
    painter.drawLine(kHPad, track_y, width() - kHPad, track_y);

    // Start marker — right-pointing triangle
    const int startMarkerX = timeToX(start_time_, duration_, width());
    painter.setPen(Qt::NoPen);
    painter.setBrush(start_color_);
    QPolygon start_triangle;
    start_triangle << QPoint(startMarkerX, track_y - kTriHalf)
                   << QPoint(startMarkerX + kTriW, track_y)
                   << QPoint(startMarkerX, track_y + kTriHalf);
    painter.drawPolygon(start_triangle);

    // End marker — left-pointing triangle
    const int endMarkerX = timeToX(end_time_, duration_, width());
    painter.setBrush(end_color_);
    QPolygon end_triangle;
    end_triangle << QPoint(endMarkerX, track_y - kTriHalf)
                 << QPoint(endMarkerX - kTriW, track_y)
                 << QPoint(endMarkerX, track_y + kTriHalf);
    painter.drawPolygon(end_triangle);

    // Playhead — vertical line
    const int currentTimeMarkerX = timeToX(current_time_, duration_, width());
    painter.setBrush(playhead_color_);
    painter.drawRoundedRect(
        QRectF(currentTimeMarkerX - 1, container_top + kPlayheadInset,
               2, (container_bottom - container_top) - 2 * kPlayheadInset),
        1, 1);

    // Marker & playhead timestamps — centered on their marker, clamped to widget bounds.
    QFont mono(QStringLiteral("IBM Plex Mono"), 9);
    painter.setFont(mono);
    const QFontMetrics fm(mono);

    const auto centered_x = [&](const QString& text, int anchor_x) {
        const int w = fm.horizontalAdvance(text);
        return qBound(0, anchor_x - w / 2, width() - w);
    };

    const int label_y = container_bottom + kMarkerLabelZone - 4;

    const QString playhead_text = QString::number(current_time_, 'f', 2);
    painter.setPen(text_color_);
    painter.drawText(centered_x(playhead_text, currentTimeMarkerX),
                     kPlayheadTextZone - 4, playhead_text);

    const QString start_text = QString::number(start_time_, 'f', 2);
    painter.setPen(start_color_);
    painter.drawText(centered_x(start_text, startMarkerX), label_y, start_text);

    const QString end_text = QString::number(end_time_, 'f', 2);
    painter.setPen(end_color_);
    painter.drawText(centered_x(end_text, endMarkerX), label_y, end_text);
}

void TimelineWidget::mousePressEvent(QMouseEvent *event)
{
    int mouseX = static_cast<int>(event->position().x());

    int startMarkerX = timeToX(start_time_, duration_, width());
    if (mouseX >= startMarkerX - kHitTolerance && mouseX <= startMarkerX + kHitTolerance + kTriW) {
        dragging_start_ = true;
    }

    int endMarkerX = timeToX(end_time_, duration_, width());
    if (mouseX >= endMarkerX - kHitTolerance - kTriW && mouseX <= endMarkerX + kHitTolerance) {
        dragging_end_ = true;
    }

    int currentTimeMarkerX = timeToX(current_time_, duration_, width());
    if (mouseX >= currentTimeMarkerX - kHitTolerance && mouseX <= currentTimeMarkerX + kHitTolerance) {
        dragging_timeline_ = true;
    }
}

void TimelineWidget::mouseMoveEvent(QMouseEvent* event) {
    if (dragging_start_) {
        updateMarkerPosition(start_time_, static_cast<int>(event->position().x()));
        update();
    } else if (dragging_end_) {
        updateMarkerPosition(end_time_, static_cast<int>(event->position().x()));
        update();
    } else if (dragging_timeline_) {
        updateMarkerPosition(current_time_, static_cast<int>(event->position().x()));
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
    markerPos = static_cast<double>(mouseX - kHPad) / (width() - 2 * kHPad);
    if (markerPos < 0.0) markerPos = 0.0;
    if (markerPos > 1.0) markerPos = 1.0;
    markerPos = markerPos * duration_;
}

} // namespace bag2vid
