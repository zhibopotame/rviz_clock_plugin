#ifndef CLOCK_WIDGET_H
#define CLOCK_WIDGET_H

#include <ros/ros.h>
#include <rviz/panel.h>
// #include <QWidget>
#include <QTime>
#include <QTimer>
#include <QLCDNumber>
#include <rviz_clock_plugin/ETA.h>

namespace rviz_clock_plugin
{
class DigitalClock : public QLCDNumber
{
Q_OBJECT

public:
    DigitalClock(QWidget *parent = nullptr);

public Q_SLOTS:
    void time_show();
    void set_eta(const rviz_clock_plugin::ETA& msg);

private:
    rviz_clock_plugin::ETA eta_msg_;
    QPalette palette_;
};

} //namespace rviz_clock_plugin
#endif // CLOCK_WIDGET_H