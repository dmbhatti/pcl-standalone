#include "downsampler.h"

#include <QThread>
#include <QtConcurrentRun>
#include <QFuture>

#include <pcl/common/time.h>
#include <pcl/filters/voxel_grid.h>

#define MAX_THREADS 4
#define LEAF_SIZE 0.01

Downsampler::Downsampler()
{
    instanceID = InstanceID::getUniqueID();

    // Init threads
    for(int i=0; i<MAX_THREADS; i++)
        threads.push_back(QtConcurrent::run(this, &Downsampler::dummy));
}

void Downsampler::receiveCloudMessage(const CloudMessage::ConstPtr &message) {

    for(int i=0; i<MAX_THREADS; i++) {
        if(threads[i].isFinished()) {
            threads[i] = QtConcurrent::run(this, &Downsampler::algorithm, message);
            return;
        }
    }
    std::cout << "[Downsampler] Message dropped." << std::endl;
}

void Downsampler::algorithm(const CloudMessage::ConstPtr &message) {

    //double tic = pcl::getTime();

    PointCloudT::Ptr downsampledCloud(new PointCloudT());

    // Downsample cloud
    pcl::VoxelGrid<PointT> voxGrid;
    voxGrid.setInputCloud(message->clouds.at(0)->pointCloud);
    voxGrid.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE); //meter leaf size
    voxGrid.filter(*downsampledCloud);


    const CloudMessage::Ptr newMessage(new CloudMessage);
    newMessage->emitterName = "Downsampler";
    newMessage->emitterID = instanceID;

    const Cloud::Ptr cloud(new Cloud);
    cloud->cloudName = "Downsampled cloud";
    cloud->pointCloud = downsampledCloud;
    cloud->cloudColor.setColor(0,255,0);
    newMessage->clouds.push_back(cloud);

    // Done working, send message
    emit newCloudMessage(newMessage);

    //std::cout << "[Downsampler] dt: " << pcl::getTime()-tic << std::endl;
}

void Downsampler::dummy() {
    return;
}
