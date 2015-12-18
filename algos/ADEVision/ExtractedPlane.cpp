#include "ExtractedPlane.hpp"
#include "PCLFunctions.hpp"

ExtractedPlane::ExtractedPlane() :
transform(),
cloud(new pcl::PointCloud<pcl::PointXYZ>),
indices_plane(new pcl::PointIndices()),
indices_objects(new pcl::PointIndices()),
indices_filtered_objects(new pcl::PointIndices()),
coefficients(new pcl::ModelCoefficients()),
filtered_objects_normals(new pcl::PointCloud<pcl::Normal>()),
frameNumber(-1),
fileteredObjectsNormalsComputed(false) {
}

ExtractedPlane::~ExtractedPlane() {
}

void ExtractedPlane::reset() {
  transform = cv::Mat();
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  indices_plane.reset(new pcl::PointIndices());
  indices_objects.reset(new pcl::PointIndices());
  indices_filtered_objects.reset(new pcl::PointIndices());
  coefficients.reset(new pcl::ModelCoefficients());
  filtered_objects_normals.reset(new pcl::PointCloud<pcl::Normal>());
  frameNumber = -1;
  fileteredObjectsNormalsComputed = false;
}

void ExtractedPlane::setFrameNumber(unsigned long int _frameNumber) {
  frameNumber = _frameNumber;
}

void ExtractedPlane::setTransform(const cv::Mat _transform) {
  transform = _transform.clone();
}

void ExtractedPlane::setCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr _cloud) {
  cloud = _cloud->makeShared();
}

bool ExtractedPlane::computeFileteredObjectsNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  if (fileteredObjectsNormalsComputed) {
    return (true);
  } else {
    if (!ComputePointNormals<pcl::PointXYZ>(cloud, indices_filtered_objects,
            filtered_objects_normals)) {
      fileteredObjectsNormalsComputed = false;
      return (false);
    }
  }
  fileteredObjectsNormalsComputed = true;
  return (true);
}


////////////////////////////////////////////////////
//                 const getters            ////////
////////////////////////////////////////////////////

unsigned long long ExtractedPlane::getFrameNumber() const {
  return (frameNumber);
}

const cv::Mat ExtractedPlane::getTransform() const {
  return transform;
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr ExtractedPlane::getCloud() const  {
  return cloud;
}

pcl::PointIndices::ConstPtr ExtractedPlane::getPlaneIndices() const  {
  return (indices_plane);
}

pcl::PointIndices::ConstPtr ExtractedPlane::getObjectsIndices() const  {
  return (indices_objects);
}

pcl::PointIndices::ConstPtr ExtractedPlane::getFilteredObjectsIndices() const  {
  return (indices_filtered_objects);
}

pcl::ModelCoefficients::ConstPtr ExtractedPlane::getPlaneCoefficients() const  {
  return (coefficients);
}

pcl::PointCloud<pcl::Normal>::ConstPtr ExtractedPlane::getFileteredObjectsNormals() const  {
  return (filtered_objects_normals);
}

////////////////////////////////////////////////////
//               non-const getters          ////////
////////////////////////////////////////////////////

cv::Mat ExtractedPlane::getTransform() {
  return transform;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractedPlane::getCloud() {
  return cloud;
}

pcl::PointIndices::Ptr ExtractedPlane::getPlaneIndices() {
  return (indices_plane);
}

pcl::PointIndices::Ptr ExtractedPlane::getObjectsIndices() {
  return (indices_objects);
}

pcl::PointIndices::Ptr ExtractedPlane::getFilteredObjectsIndices() {
  return (indices_filtered_objects);
}

pcl::ModelCoefficients::Ptr ExtractedPlane::getPlaneCoefficients() {
  return (coefficients);
}

pcl::PointCloud<pcl::Normal>::Ptr ExtractedPlane::getFileteredObjectsNormals() {
  return (filtered_objects_normals);
}
