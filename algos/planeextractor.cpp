#include "planeextractor.h"
#include <math.h>
#include <QThread>
#include <QtConcurrentRun>
#include <QFuture>
#include <pcl/common/time.h>

// For ADE Code
#include <opencv2/opencv.hpp>
#include "ADEVision/PCLFunctions.hpp"
#include "ADEVision/ExtractedPlane.hpp"

// For our own fast plane segmentation
#include <algorithm>
#include <pcl/common/angles.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Dense>

#define MAX_THREADS 8
#define USE_FAST_PLANE_SEGMENTATION

PlaneExtractor::PlaneExtractor() {
    instanceID = InstanceID::getUniqueID();
    processingTime = 0;
    lastSpawn =  pcl::getTime();

    // Init threads
    for(int i=0; i<MAX_THREADS; i++)
        threads.push_back(QtConcurrent::run(this, &PlaneExtractor::dummy));
}

void PlaneExtractor::receiveCloudMessage(const CloudMessage::ConstPtr &message) {
    for(int i=0; i<MAX_THREADS; i++) {
        if(threads[i].isFinished() && pcl::getTime() > (lastSpawn+(processingTime/MAX_THREADS))) {
            threads[i] = QtConcurrent::run(this, &PlaneExtractor::algorithm, message);
            lastSpawn = pcl::getTime();
            return;
        }
    }
    //std::cout << "[PlaneExtractor] Message dropped." << std::endl;
}

void PlaneExtractor::algorithm(const CloudMessage::ConstPtr &message) {
    double tic = pcl::getTime ();

    /* We need this here because the code imported from ADE does not support PointXYZRGBA */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_xyz->points.resize(message->clouds.at(0)->pointCloud->points.size());
    for (size_t i = 0; i < message->clouds.at(0)->pointCloud->points.size(); i++) {
        cloud_xyz->points[i].x = message->clouds.at(0)->pointCloud->points[i].x;
        cloud_xyz->points[i].y = message->clouds.at(0)->pointCloud->points[i].y;
        cloud_xyz->points[i].z = message->clouds.at(0)->pointCloud->points[i].z;
    }

    // Camera position
    static const double ROTX = pcl::deg2rad(-30.0); //-30deg
    static const double CAM_HEIGHT = 1.6;

    // Table
    static const double TABLE_HEIGHT = 0.7;

    // Default values to interface with ADE Vision code...
    static const unsigned long long FRAMENUM = 0;
    static const cv::Mat TRANSFORM = (cv::Mat_<double>(4,4) <<
                                      1, 0,           0,          0,
      0,  sin(ROTX),  cos(ROTX),  0,
      0, -cos(ROTX),  sin(ROTX),  CAM_HEIGHT,
      0, 0,           0,          1);

    ExtractedPlane::Ptr currentPlane(new ExtractedPlane());
    currentPlane->setCloud(cloud_xyz);
    currentPlane->setTransform(TRANSFORM);
    currentPlane->setFrameNumber(FRAMENUM);

#ifndef USE_FAST_PLANE_SEGMENTATION
    if (!SegmentPlane<pcl::PointXYZ>(cloud_xyz,
                                     currentPlane->getPlaneIndices(),
                                     currentPlane->getObjectsIndices(),
                                     currentPlane->getPlaneCoefficients(), TRANSFORM)) {
        std::cout << "[PlaneExtractor] SegmentPlane did not find plane." << std::endl;
        return;
    }
#else
    Eigen::Matrix3f rot;
    rot << 1, 0, 0,
      0,  cos(ROTX),  sin(ROTX),
      0, -sin(ROTX),  cos(ROTX);

    // plane normal in the base coordinate system
    Eigen::Vector3f base_up(0, -1, 0);
    Eigen::Vector3f desired_plane_norm = rot * base_up;
    desired_plane_norm.normalize();

    if(!fastPlanSegmentation(message->clouds.at(0)->pointCloud, desired_plane_norm, CAM_HEIGHT, TABLE_HEIGHT,
                             currentPlane->getPlaneIndices(), currentPlane->getObjectsIndices(),
                             currentPlane->getPlaneCoefficients())) {
        std::cout << "[PlaneExtractor] fastPlanSegmentation did not find plane." << std::endl;
        return;
    }
#endif

    boost::mutex::scoped_lock lock(mutexChull);
    if (!filterPointsOnPlane<pcl::PointXYZ>(cloud_xyz,
                                            currentPlane->getPlaneIndices(),
                                            currentPlane->getObjectsIndices(),
                                            currentPlane->getPlaneCoefficients(),
                                            currentPlane->getFilteredObjectsIndices(), &mutexChull)) {
        std::cout << "[PlaneExtractor] filterPointsOnPlane failed to filter points on plane." << std::endl;
        return;
    }
    lock.unlock();

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(message->clouds.at(0)->pointCloud);

    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (message->clouds.at(0)->pointCloud);
    ec.setIndices(currentPlane->getFilteredObjectsIndices());
    ec.extract (clusters);

    const CloudMessage::Ptr newMessage(new CloudMessage);
    newMessage->emitterName = "PlaneExtractor";
    newMessage->emitterID = instanceID;

    const Cloud::Ptr cloud(new Cloud);
    cloud->cloudName = "Cloud";
    cloud->pointCloud = message->clouds.at(0)->pointCloud;
    cloud->cloudColor.setColor(50,50,50);

    const Object::Ptr plane(new Object);
    plane->objectName = "Plane";
    plane->indices = currentPlane->getPlaneIndices();
    plane->objectColor.setColor(0,0,255);
    cloud->objects.push_back(plane);

    for (int i=0; i<clusters.size(); i++) {
        const Object::Ptr objectOnTable(new Object);
        std::stringstream ss;
        ss << "Object " << i;
        std::string s = ss.str();
        objectOnTable->objectName = QString(s.c_str());
        pcl::PointIndices::Ptr obj (new pcl::PointIndices());
        obj->indices = clusters[i].indices;
        objectOnTable->indices = obj;
        objectOnTable->objectColor.setColor(std::max(255-50*i,0),std::min(50*i,255),0);
        cloud->objects.push_back(objectOnTable);
    }

    newMessage->clouds.push_back(cloud);

    // Done working, send message
    emit newCloudMessage(newMessage);

    processingTime = pcl::getTime()-tic;
    std::cout << "[PlaneExtractor] dT: " << processingTime << std::endl;
}


bool PlaneExtractor::fastPlanSegmentation(const PointCloudT::ConstPtr inputCloud, Eigen::Vector3f vertical, double cam_height, double table_height,
                                          pcl::PointIndices::Ptr inliers, pcl::PointIndices::Ptr objects_indices, pcl::ModelCoefficients::Ptr coefficients)
{
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(vertical);
    seg.setEpsAngle(pcl::deg2rad(20.0));
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.0125);
    seg.setMaxIterations(2000);
    seg.setInputCloud (inputCloud);
    seg.segment (*inliers, *coefficients);

    if(inliers->indices.size()>500) {
        /*double planeHeight = 0;
        int samples = 0;
        for(int i=0; i<inliers->indices.size(); i+=100) {
            planeHeight += Eigen::Vector3f(inputCloud->points[inliers->indices[0]].getArray3fMap()).dot(vertical);
            samples++;
        }
        planeHeight /= samples;

        if(fabs(planeHeight+cam_height-table_height) < 0.1) {*/

        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        pcl::IndicesPtr tmpPlaneIndices = pcl::IndicesPtr(new std::vector<int>(inliers->indices));
        tree->setInputCloud(inputCloud, tmpPlaneIndices);
        std::vector<pcl::PointIndices> clusters;
        pcl::extractEuclideanClusters<PointT>(*inputCloud, *tmpPlaneIndices, tree, 0.03, clusters);
        int largestClusterSize = 0;
        int largestIndex = -1;
        for (int i = 0; i < clusters.size(); ++i) {
            if (clusters[i].indices.size() > largestClusterSize) {
                largestClusterSize = clusters[i].indices.size();
                largestIndex = i;
            }
        }
        inliers->indices = clusters[largestIndex].indices;

        //check normal (we want the normal facing towards the camera and there's no guarantee of it's direction)
        Eigen::Vector3f plane_norm(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        plane_norm.normalize();
        float cos_ang = plane_norm.dot(vertical);
        float ang_btwn = std::acos(cos_ang);
        //printf("[SegmentPlane] plane norm: %f %f %f angle: %f\n", plane_norm[0], plane_norm[1], plane_norm[2], ang_btwn * (180.0 / M_PI));

        //if norm is facing wrong way, flip it
        if (ang_btwn > (M_PI - M_PI / 6.0)) {
            //printf("[SegmentPlane] wrong normal direction...flipping.\n\n");
            coefficients->values[0] = -coefficients->values[0];
            coefficients->values[1] = -coefficients->values[1];
            coefficients->values[2] = -coefficients->values[2];
            coefficients->values[3] = -coefficients->values[3];
        } else if (ang_btwn > M_PI / 6.0) {
            PCL_ERROR("[SegmentPlane][ERROR] Wrong normal!\n");
        }

        std::vector<bool> used_indices(inputCloud->size(), false);
        for (int i = 0; i < inliers->indices.size(); ++i) {
            int index = inliers->indices[i];
            used_indices.at(index) = true;
        }

        objects_indices->indices.reserve(inputCloud->size() - inliers->indices.size());
        for (int i = 0; i < used_indices.size(); ++i) {
            if (!used_indices[i]) {
                objects_indices->indices.push_back(i);
            }
        }

        return true;
        /*}
        else return false;*/

    }
    else std::cerr << "[PlaneExtractor] Not enough points in plane! " << std::endl;

    return false;
}



void PlaneExtractor::dummy() {
    return;
}
