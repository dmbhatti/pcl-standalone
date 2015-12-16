#include "planeextractor.h"
#include <QThread>
#include <QtConcurrentRun>
#include <QFuture>

#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>

PlaneExtractor::PlaneExtractor()
{
    instanceID = InstanceID::getUniqueID();

    // Init thread variable with a stupid task (used to check if thread finished).
    thread1 = QtConcurrent::run(this, &PlaneExtractor::dummy);
    thread2 = QtConcurrent::run(this, &PlaneExtractor::dummy);
}

void PlaneExtractor::receiveCloudMessage(const CloudMessage::ConstPtr &message) {

    if(thread1.isFinished())
        thread1 = QtConcurrent::run(this, &PlaneExtractor::algorithm, message);
    else if(thread2.isFinished())
        thread2 = QtConcurrent::run(this, &PlaneExtractor::algorithm, message);
    else std::cout << "[PlaneExtractor] Message dropped." << std::endl;
}

void PlaneExtractor::algorithm(const CloudMessage::ConstPtr &message) {

    //double tic = pcl::getTime ();

    PointCloudT::Ptr cloud_downsampled(new PointCloudT());

    //downsample point cloud
    pcl::VoxelGrid<PointT> vox_grid;
    vox_grid.setInputCloud(message->clouds.at(0)->pointCloud);
    vox_grid.setLeafSize(0.01, 0.01, 0.01); //meter leaf size
    vox_grid.filter(*cloud_downsampled);


    const CloudMessage::Ptr newMessage(new CloudMessage);
    newMessage->emitterName = "PlaneExtractor";
    newMessage->emitterID = instanceID;

    const Cloud::Ptr cloud(new Cloud);
    cloud->cloudName = "Downsampled cloud";
    cloud->pointCloud = cloud_downsampled;
    cloud->cloudColor.setColor(0,255,0);
    newMessage->clouds.push_back(cloud);

    // Done working, send message
    emit newCloudMessage(newMessage);

    //std::cout << "algo dT: " << pcl::getTime()-tic << std::endl;
}

void PlaneExtractor::dummy() {
    return;
}
