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


int main(int argc, char ** argv)
{
  QApplication application(argc, argv);
  vijfendertig::QtControlUnitWindow window(argc, argv);
  window.show();
  application.connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));

  int result = application.exec();

  return result;
}
