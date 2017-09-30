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

#ifndef rqt_compass__compass_H
#define rqt_compass__compass_H

// ROS headers
#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

// Qt headers
#include <QImage>
#include <QList>
#include <QMutex>
#include <QString>
#include <QSize>
#include <QWidget>

#include <QtGui>
#include <QResource>
#include <QSettings>

// Qwt headers
#include <qwt_compass.h>
#include <qwt_compass_rose.h>
#include <qwt_dial_needle.h>

// Qt widget header
#include <ui_compass.h>

// STL headers
#include <vector>

namespace rqt_compass {

     class compass
          : public rqt_gui_cpp::Plugin
     {

          Q_OBJECT

     private:
          
     public:

          compass();

          virtual void initPlugin(qt_gui_cpp::PluginContext& context);

          virtual bool eventFilter(QObject* watched, QEvent* event);
          
          virtual void shutdownPlugin();

          virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

          virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

          protected slots:
          
     protected:

          protected slots:

          virtual void onEnableDesiredHeading(bool checked);          
          virtual void onSetHeading(bool checked);
          virtual void updateGUI();

     protected:

          virtual void callback_pose(const videoray_control::NavData& msg);
          
          Ui::compassWidget ui_;

          QWidget* widget_;

          //ros::Publisher publisher_;
          ros::Subscriber subscriber_;

          QwtSimpleCompassRose *rose_;
          QwtCompassMagnetNeedle *needle_;
          
          QTimer *timer_;
          geometry_msgs::Vector3 orientation_;
     };

}

#endif // rqt_compass__compass_H
