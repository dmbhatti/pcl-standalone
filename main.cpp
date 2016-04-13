#include <QApplication>
#include <QMainWindow>

#include "pclviewer.h"

int main (int argc, char *argv[])
{
  QApplication a(argc, argv);

  // Create the PCL viewer
  PCLViewer w;

  w.show ();

  return a.exec ();
}
