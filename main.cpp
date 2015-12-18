#include <QApplication>
#include <QMainWindow>

#include "pclviewer.h"
#include "capture.h"

#include "algos/downsampler.h"
#include "algos/planeextractor.h"

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);

  // Create a new grabber for OpenNI devices
  pcl::Grabber* grabber = new pcl::OpenNIGrabber();

  // Create a Capture object running the grabber in a seperate thread
  Capture* camera = new Capture(grabber);

  // Create the PCL viewer
  PCLViewer w;
  w.setCaptureDevice(camera);

  /*--------------------------------*/
  /* Setup and link algorithms here */
  /*--------------------------------*/

  Downsampler* downsampler = new Downsampler;
  downsampler->connectToCloud(camera);

  PlaneExtractor* planeExtractor = new PlaneExtractor;
  planeExtractor->connectToCloud(downsampler);


  /*------------------------------------------------------*/
  /* Link the output of the algorithms to the viewer here */
  /*------------------------------------------------------*/

  //w.connectToCloud(camera); // View source point cloud
  //w.connectToCloud(downsampler); // View downsampled source cloud
  w.connectToCloud(planeExtractor); // View extracted plane

  w.show ();



  return a.exec ();
}
