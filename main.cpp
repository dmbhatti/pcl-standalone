#include <QApplication>
#include <QMainWindow>

#include "pclviewer.h"
#include "capture.h"

#include "algos/planeextractor.h"

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);

  // Create a new grabber for OpenNI devices
  pcl::Grabber* grabber = new pcl::OpenNIGrabber();

  // Create a Capture object running the grabber in a seperate thread
  Capture* camera = new Capture(grabber);

  PlaneExtractor* planeExtractor = new PlaneExtractor;
  planeExtractor->connectToCloud(camera);

  // Create the PCL viewer attached to the camera
  PCLViewer w;

  w.setCaptureDevice(camera);

  //w.connectToCloud(camera);
  w.connectToCloud(planeExtractor);

  w.show ();



  return a.exec ();
}
