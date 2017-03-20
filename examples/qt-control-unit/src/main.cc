//! Implementation of the main function for the Velleman/OWI Robotic Arm's virtual control unit.
/*!
 *  \file
 *  \author Maarten De Munck, <maarten@vijfendertig.be>
 *  \date 2017
 *  \copyright Licensed under the MIT License. See LICENSE for the full license.
 */


#include "../include/qt-control-unit-window.h"

#include <QtGui>
#include <QApplication>
#include <QIcon>


int main(int argc, char ** argv)
{
  QApplication application(argc, argv);
  QIcon robotic_arm_icon;
  robotic_arm_icon.addFile(":/icons/robotic-arm-16x16.png", QSize(16, 16));
  robotic_arm_icon.addFile(":/icons/robotic-arm-24x24.png", QSize(24, 24));
  robotic_arm_icon.addFile(":/icons/robotic-arm-32x32.png", QSize(32, 32));
  robotic_arm_icon.addFile(":/icons/robotic-arm-48x48.png", QSize(48, 48));
  robotic_arm_icon.addFile(":/icons/robotic-arm-64x64.png", QSize(64, 64));
  robotic_arm_icon.addFile(":/icons/robotic-arm-96x96.png", QSize(96, 96));
  robotic_arm_icon.addFile(":/icons/robotic-arm-128x128.png", QSize(128, 128));
  robotic_arm_icon.addFile(":/icons/robotic-arm-256x256.png", QSize(256, 256));
  robotic_arm_icon.addFile(":/icons/robotic-arm-512x512.png", QSize(512, 512));
  robotic_arm_icon.addFile(":/icons/robotic-arm-1024x1024.png", QSize(1024, 1024));
  application.setWindowIcon(robotic_arm_icon);
  vijfendertig::QtControlUnitWindow window(argc, argv);
  window.setWindowIcon(robotic_arm_icon);
  window.show();
  application.connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));

  int result = application.exec();

  return result;
}
