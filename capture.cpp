#include <QMetaType>
#include <pcl/common/time.h>
#include "capture.h"

Capture::Capture (pcl::Grabber * grabber) : grabber_ (grabber)
{
    instanceID = InstanceID::getUniqueID();

    // Allows sending CloudMessage::ConstPtr through signal/slots.
    qRegisterMetaType<CloudMessage::ConstPtr> ("CloudMessage::ConstPtr");
}

Capture::~Capture (){
    // stop the grabber
    grabber_->stop ();
}

void Capture::cloudCallback (const PointCloudT::ConstPtr &input)
{
    static double last = pcl::getTime ();
    double now = pcl::getTime ();
    std::cout << "capture dT: " << now-last << std::endl;

    const CloudMessage::Ptr message(new CloudMessage);
    message->emitterName = "Camera";
    message->emitterID = instanceID;
    message->emitterInfo.push_back("FPS: " + QString::number(1/(now - last)));
    message->emitterInfo.push_back("Points: " + QString::number(input->points.size()));
    last = now;

    const Cloud::Ptr cloud(new Cloud);
    cloud->cloudName = "Source";
    cloud->pointCloud = input;
    cloud->cloudColor.setColor(255,0,0);
    message->clouds.push_back(cloud);



    emit newCloudMessage(message);
}

void Capture::run ()
{
    if (!grabber_->isRunning()) {
        std::cout << "[CAPTURE] Starting new grabber thread..." << std::endl;

        // make callback function from member function
        boost::function<void (const PointCloudT::ConstPtr&)> f =
                boost::bind (&Capture::cloudCallback, this, _1);

        // connect callback function for desired signal. In this case its a point cloud with color values
        boost::signals2::connection c = grabber_->registerCallback (f);

        // start receiving point clouds
        grabber_->start ();
    }
    else std::cerr << "[CAPTURE] Grabber already running!" << std::endl;
}

void Capture::stop() {
    if (grabber_->isRunning()) {
        std::cout << "[CAPTURE] Stopping grabber thread..." << std::endl;
        grabber_->stop();
    }
}

bool Capture::isRunning() {
   return grabber_->isRunning();
}
