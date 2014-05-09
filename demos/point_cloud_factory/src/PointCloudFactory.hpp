#ifndef POINTCLOUDFACTORY_HPP
#define POINTCLOUDFACTORY_HPP

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;

class PointCloudFactory {
public:
  PointCloudFactory();
  void createCube(PointCloud<PointXYZRGB>::Ptr &cloud);
  void createSphere(PointCloud<PointXYZRGB>::Ptr &cloud);
};

#endif
