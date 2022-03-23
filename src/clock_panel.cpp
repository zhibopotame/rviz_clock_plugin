#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include "clock.h"
#include "clock_panel.h"

namespace rviz_clock_plugin
{
ClockPanel::ClockPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Topic:" ));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_ );

  // Then create the control widget.
  digital_clock_ = new DigitalClock;

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  layout->addWidget( digital_clock_ );
  setLayout( layout );
  QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));

  // Start the timer.
  output_timer->start( 100 );

  // Make the control widget start disabled, since we don't start with an output topic.
  digital_clock_->setEnabled( false );
}

// Read the topic name from the QLineEdit and call setTopic() with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void ClockPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}

// Set the topic name we are publishing to.
void ClockPanel::setTopic( const QString& new_topic )
{
  // Only take action if the name has changed.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( output_topic_ != "" )
    {
      eta_sub_ = nh_.subscribe( output_topic_.toStdString(), 1 ,&ClockPanel::eta_callback, this);
    }
    // rviz::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }

  // Gray out the control widget when the output topic is empty.
  digital_clock_->setEnabled( output_topic_ != "" );
}

void ClockPanel::eta_callback(const rviz_clock_plugin::ETA &msg)
{
  // std::cout<<msg<<std::endl;
  digital_clock_->set_eta(msg);
}


// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void ClockPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void ClockPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}

} // end namespace rviz_obstacle_plugin

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_clock_plugin::ClockPanel,rviz::Panel )
// END_TUTORIAL