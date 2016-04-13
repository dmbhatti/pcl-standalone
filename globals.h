#ifndef GLOBALS_H
#define GLOBALS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <QVector>

#include "instanceid.h"

/*  Point type used in this tool.
 *  Changing the point type can be done here
 *  and will be used throughout this tool (not totally true: code copied from ADE expect PointXYZ...).
 *  PointXYZRGBA (heavier, more information)
 *  PointXYZ (lighter)
 */
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PointColorRGB: public pcl::visualization::PointCloudColorHandlerCustom<PointT> {
public:
    PointColorRGB() : pcl::visualization::PointCloudColorHandlerCustom<PointT>(255, 255, 255) {}
    void setColor(int r, int g, int b) { r_ = r; r__ = r; g_ = g; g__ = g; b_ = b; b__ = b;}
    int r__, g__, b__;
};


class Object {
public:
    pcl::PointIndicesConstPtr indices;
    QString objectName;
    PointColorRGB objectColor;
    QVector<QString> objectInfo;

    typedef boost::shared_ptr<Object > Ptr;
    typedef boost::shared_ptr<const Object > ConstPtr;
};

class Cloud {
public:
    PointCloudT::ConstPtr pointCloud;
    QString cloudName;
    PointColorRGB cloudColor;
    QVector<QString> cloudInfo;
    QVector<Object::Ptr> objects;

    typedef boost::shared_ptr<Cloud > Ptr;
    typedef boost::shared_ptr<const Cloud > ConstPtr;
};

class CloudMessage {
public:
    QVector<Cloud::Ptr> clouds;
    int emitterID;
    QString emitterName;
    QVector<QString> emitterInfo;

    typedef boost::shared_ptr<CloudMessage > Ptr;
    typedef boost::shared_ptr<const CloudMessage > ConstPtr;
};

#endif // GLOBALS_H
