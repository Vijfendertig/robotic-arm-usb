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
