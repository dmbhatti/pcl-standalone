#ifndef PLANEEXTRACTOR_H
#define PLANEEXTRACTOR_H
#include <QFuture>
#include <pcl/ModelCoefficients.h>
#include "../cloudemitterreceiver.h"
#include <boost/thread/mutex.hpp>

class PlaneExtractor : public CloudEmitterReceiver
{
    Q_OBJECT

public:
    PlaneExtractor();
    boost::mutex mutexChull;

public slots:
    void receiveCloudMessage(const CloudMessage::ConstPtr &message);

private:
    void algorithm(const CloudMessage::ConstPtr &message);
    void dummy();
    int instanceID;

    bool fastPlanSegmentation(const PointCloudT::ConstPtr inputCloud, Eigen::Vector3f vertical, double cam_height, double table_height,
                                              pcl::PointIndices::Ptr inliers, pcl::PointIndices::Ptr objects_indices,
                              pcl::ModelCoefficients::Ptr coefficients);

    QVector<QFuture<void> > threads;
    double processingTime;
    double lastSpawn;
};

#endif // PLANEEXTRACTOR_H
