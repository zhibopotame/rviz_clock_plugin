#ifndef CLOCK_PANEL_H
#define CLOCK_PANEL_H

#include <ros/ros.h>

#include <rviz/panel.h>
#include <rviz_clock_plugin/ETA.h>

#include "clock.h"


class QLineEdit;

namespace rviz_clock_plugin
{

class DriveWidget;

class ClockPanel: public rviz::Panel
{


Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  ClockPanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:
  // The control area, DriveWidget, sends its output to a Qt signal
  // for ease of re-use, so here we declare a Qt slot to receive it.

  // In this example setTopic() does not get connected to any signal
  // (it is called directly), but it is easy to define it as a public
  // slot instead of a private function in case it would be useful to
  // some other user.
  void setTopic( const QString& topic );

  // Here we declare some internal slots.
protected Q_SLOTS:
  // topic.  Internally this is connected to a timer which calls it 10
  // times per second.

  // updateTopic() reads the topic name from the QLineEdit and calls
  // setTopic() with the result.
  void updateTopic();

  void eta_callback(const rviz_clock_plugin::ETA& msg);

  // Then we finish up with protected member variables.
protected:
  DigitalClock* digital_clock_;

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* output_topic_editor_;

  // The current name of the output topic.
  QString output_topic_;

  ros::Subscriber eta_sub_;

  // The ROS node handle.
  ros::NodeHandle nh_;

};

} // end namespace rviz_obstacle_plugin

#endif // DETECTION_PANEL_H