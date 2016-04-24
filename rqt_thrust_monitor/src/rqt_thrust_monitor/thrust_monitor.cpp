/*
 * Copyright (c) 2013, Kevin DeMarco
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <std_msgs/String.h>

#include <rqt_thrust_monitor/thrust_monitor.h>

#include <QMessageBox>
#include <QPainter>

using std::cout;
using std::endl;

namespace rqt_thrust_monitor {

     thrust_monitor::thrust_monitor()
          : rqt_gui_cpp::Plugin()
          , widget_(0)
     {
          setObjectName("thrust_monitor");
     }

     void thrust_monitor::initPlugin(qt_gui_cpp::PluginContext& context)
     {
          widget_ = new QWidget();
          ui_.setupUi(widget_);

          if (context.serialNumber() > 1)
          {
               widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
          }
          context.addWidget(widget_);

          this->subscriber_ = getNodeHandle().subscribe<videoray_control::Throttle>("/videoray_control/throttle_cmd", 100, &thrust_monitor::callback_throttle, this);

          // Start GUI update timer
          timer_ = new QTimer(this);
          connect(timer_, SIGNAL(timeout()), this, SLOT(updateGUI()));
          timer_->start(10);
     }     

     void thrust_monitor::updateGUI()
     {
          ui_.port_slider->setValue(throttle_.PortInput);
          ui_.port_spinbox->setValue(throttle_.PortInput);

          ui_.vert_slider->setValue(throttle_.VertInput);
          ui_.vert_spinbox->setValue(throttle_.VertInput);

          ui_.star_slider->setValue(throttle_.StarInput);
          ui_.star_spinbox->setValue(throttle_.StarInput);
     }

     bool thrust_monitor::eventFilter(QObject* watched, QEvent* event)
     {
          return QObject::eventFilter(watched, event);
     }

     void thrust_monitor::shutdownPlugin()
     {
          timer_->stop();
          subscriber_.shutdown();
     }
     
     void thrust_monitor::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
     {
          //instance_settings.setValue("desired_heading", ui_.desired_heading_double_spin_box->value());
     }

     void thrust_monitor::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
     {
          //double desired_heading = instance_settings.value("desired_heading", ui_.desired_heading_double_spin_box->value()).toDouble();
          //ui_.desired_heading_double_spin_box->setValue(desired_heading);          
     }
     
     void thrust_monitor::callback_throttle(const videoray_control::ThrottleConstPtr& msg)
     {
          throttle_ = *msg;
     }
}

PLUGINLIB_EXPORT_CLASS(rqt_thrust_monitor::thrust_monitor, rqt_gui_cpp::Plugin)
