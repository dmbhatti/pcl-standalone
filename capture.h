#ifndef CAPTURE_H
#define CAPTURE_H

#include "cloudemitterreceiver.h"
#include <boost/shared_ptr.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include "globals.h"


/** Wrapper for the grabbing from an OpenNI device.
*   Wrapper is used to run the grabbing in a separate thread (QThread). */

class Capture : public CloudEmitter
{
public:
    /** Creates a new wrapper object from the specified grabber. */
    Capture (pcl::Grabber * grabber);
    Capture (QString file);
    virtual ~Capture ();

    /** Starts the thread. */
    void run ();

    /** Stop the thread */
    void stop();

    /** Get status */
    bool isRunning();

    /** Callback that is used to get cloud data from the grabber. */
    void cloudCallback (const PointCloudT::ConstPtr & input);

private:
    /** The grabber data is received from. */
    pcl::Grabber * grabber_;

    /** If Capture should simulate a camera using a file */
    bool useFile;

    /** Holds a pointcloud */
    PointCloudT::Ptr cloudFromFile;

    int instanceID;
};

#endif // CAPTURE_H
