/*
#    Copyright (c) 2026 Adorno-Lab
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License.
#    If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana (email: juanjose.quirozomana@manchester.ac.uk)
#
# ################################################################
*/

/**
 * @file navigation_view.cpp
 * @brief Implementation of NavigationView.
 */

#include "navigation_view.hpp"
#include <algorithm>
#include <cmath>
#include <utility>
#include <QPainter>

namespace haptic_tools
{

using Status = haptic_tools_msgs::msg::HapticNavigationStatus;   ///< The status of the navigation.

namespace
{

const QColor kTranslation(40, 170, 70);   ///< Green.
const QColor kRotation(230, 140, 20);     ///< Orange.
const QColor kStopped(50, 110, 220);      ///< Blue: the clutch is pressed, and the base is stopped.
const QColor kInactive(150, 150, 150);    ///< Grey.

/// @brief Returns the mode of the status, as text, and its colour.
std::pair<QString, QColor> _mode(const Status& s)
{
    if (!s.device_data_valid)
        return {"No valid data of the device", kInactive};
    if (s.waiting_for_clutch_release)
        return {"Release the clutch, and press it again", kInactive};
    if (!s.active)
        return {"Clutch released: the base is stopped", kInactive};
    if (s.submode == Status::SUBMODE_TRANSLATION)
        return {"TRANSLATION", kTranslation};
    if (s.submode == Status::SUBMODE_ROTATION)
        return {"ROTATION", kRotation};
    return {"Stopped (inside the dead zone)", kStopped};
}

}

NavigationView::NavigationView(QWidget* parent):
    QWidget(parent)
{
    setMinimumSize(300, 360);
}

void NavigationView::set_status(const Status& status)
{
    status_ = status;
    update();
}

void NavigationView::clear()
{
    status_.reset();
    update();
}

void NavigationView::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), palette().base());
    if (!status_.has_value())
    {
        painter.drawText(rect(), Qt::AlignCenter, "No status of the navigation.\nIs haptic_navigation_example_node running?");
        return;
    }
    const Status& s = *status_;
    const auto [mode, colour] = _mode(s);

    // The mode.
    const QRectF title(rect().left(), rect().top() + 6, rect().width(), 24);
    QFont bold = painter.font();
    bold.setBold(true);
    bold.setPointSizeF(bold.pointSizeF()*1.3);
    painter.setFont(bold);
    painter.setPen(colour);
    painter.drawText(title, Qt::AlignCenter, mode);
    painter.setFont(font());

    // Top view of the base: x forward (up), y to the left.
    const QRectF view = QRectF(rect()).adjusted(10, 36, -10, -70);
    const double half = std::min(view.width(), view.height())/2.0;
    const QPointF centre = view.center();
    painter.setPen(QPen(palette().mid().color(), 1));
    painter.drawLine(QPointF(centre.x(), centre.y() - half), QPointF(centre.x(), centre.y() + half));
    painter.drawLine(QPointF(centre.x() - half, centre.y()), QPointF(centre.x() + half, centre.y()));
    painter.setPen(palette().text().color());
    painter.drawText(QPointF(centre.x() + 4, centre.y() - half + 12), "forward");
    painter.drawText(QPointF(centre.x() - half + 2, centre.y() - 4), "left");

    // The dead zone fills 40% of the view, unless the stylus is further away.
    const double r = std::max(s.dead_zone_radius, 1e-6);
    const double range = std::max(2.5*r, 1.2*std::hypot(s.stylus_offset.x, s.stylus_offset.y));
    const double pixels_per_m = half/range;
    painter.setPen(QPen(palette().text().color(), 1.5, Qt::DashLine));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(centre, r*pixels_per_m, r*pixels_per_m);
    const QPointF stylus(centre.x() - s.stylus_offset.y*pixels_per_m, centre.y() - s.stylus_offset.x*pixels_per_m);
    painter.setPen(QPen(colour.darker(130), 1.5));
    painter.setBrush(colour);
    painter.drawEllipse(stylus, 8, 8);
    painter.setPen(palette().text().color());
    painter.drawText(view, Qt::AlignBottom | Qt::AlignLeft, QString("dead zone r = %1 m").arg(r, 0, 'f', 3));

    // theta, from -pi to pi, with the rotation zones beyond the threshold.
    const QRectF bar(rect().left() + 14, rect().bottom() - 56, rect().width() - 28, 16);
    auto to_bar = [&](const double& angle) {
        return bar.left() + (std::clamp(angle, -M_PI, M_PI) + M_PI)/(2.0*M_PI)*bar.width();
    };
    painter.setPen(QPen(palette().mid().color(), 1));
    painter.setBrush(palette().window());
    painter.drawRect(bar);
    painter.setBrush(QColor(kRotation.red(), kRotation.green(), kRotation.blue(), 90));
    painter.drawRect(QRectF(QPointF(bar.left(), bar.top()), QPointF(to_bar(-s.rotation_threshold), bar.bottom())));
    painter.drawRect(QRectF(QPointF(to_bar(s.rotation_threshold), bar.top()), QPointF(bar.right(), bar.bottom())));
    const double needle = to_bar(s.gimbal_roll);
    painter.setPen(QPen(s.submode == Status::SUBMODE_ROTATION ? kRotation : palette().text().color(), 3));
    painter.drawLine(QPointF(needle, bar.top() - 4), QPointF(needle, bar.bottom() + 4));
    painter.setPen(palette().text().color());
    painter.drawText(QRectF(bar.left(), bar.bottom() + 6, bar.width(), 18), Qt::AlignCenter,
                     QString("theta = %1 rad   (limit \u00B1%2)")
                         .arg(s.gimbal_roll, 0, 'f', 2).arg(s.rotation_threshold, 0, 'f', 2));
}

}
