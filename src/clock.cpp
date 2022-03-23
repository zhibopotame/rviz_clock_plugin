#include "clock.h"

namespace rviz_clock_plugin
{
DigitalClock::DigitalClock(QWidget *parent): QLCDNumber(parent)
{
    setSegmentStyle(Filled);
    setWindowTitle(tr("Digital Clock"));
    resize(150, 60);

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &DigitalClock::time_show);
    timer->start(300);
    time_show();
}

void DigitalClock::set_eta(const rviz_clock_plugin::ETA& msg)
{
    eta_msg_ = msg;
}

void DigitalClock::time_show()
{
    QTime time(0,0,0);
    time = time.addSecs(int(eta_msg_.time));
    QString text = time.toString("mm:ss");
    switch(eta_msg_.status)
    {
        case(rviz_clock_plugin::ETA::STATUS_WAITING):
            palette_.setColor(palette_.WindowText, Qt::gray);
            setPalette(palette_);
            break;
        case(rviz_clock_plugin::ETA::STATUS_FINISHED):
            palette_.setColor(palette_.WindowText, Qt::green);
            setPalette(palette_);
            break;
        case(rviz_clock_plugin::ETA::STATUS_PENDING):
            palette_.setColor(palette_.WindowText, Qt::red);
            setPalette(palette_);
            break;
        case(rviz_clock_plugin::ETA::STATUS_EXECUTING):
            palette_.setColor(palette_.WindowText, Qt::blue);
            setPalette(palette_);
            break;
        
        default:
            palette_.setColor(palette_.WindowText, Qt::gray);
            setPalette(palette_);
            break;
    }

    display(text);
}

} //namespace rviz_clock_plugin