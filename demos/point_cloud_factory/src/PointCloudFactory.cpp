#include "PointCloudFactory.hpp"

#include <pcl/io/pcd_io.h>

using namespace pcl;

PointCloudFactory::PointCloudFactory()
{
  std::cout << "Created a new Point Cloud Factory" << std::endl;
}

void PointCloudFactory::createCube(PointCloud<PointXYZRGB>::Ptr &cloud)
{
  cloud->points.resize(6 * 256);

  cloud->width=6 * 256;
  cloud->height=1;
  cloud->is_dense = true;

  for (int i = 0; i < 256; i++) {
    cloud->points[6*i].rgba = 0x00ff00ff;
    cloud->points[6*i].x = 0.5;
    cloud->points[6*i].y = (float) (i % 16) / 16.0f - 0.5f;
    cloud->points[6*i].z = (float) (i / 16) / 16.0f - 0.5f;
    cloud->points[6*i + 1].rgba = 0x00ff0000;
    cloud->points[6*i + 1].y = 0.5;
    cloud->points[6*i + 1].x = (float) (i % 16) / 16.0f - 0.5f;
    cloud->points[6*i + 1].z = (float) (i / 16) / 16.0f - 0.5f;
    cloud->points[6*i + 2].rgba = 0x0000ff00;
    cloud->points[6*i + 2].z = 0.5;
    cloud->points[6*i + 2].y = (float) (i % 16) / 16.0f - 0.5f;
    cloud->points[6*i + 2].x = (float) (i / 16) / 16.0f - 0.5f;
    cloud->points[6*i + 3].rgba = 0x000000ff;
    cloud->points[6*i + 3].x = - 0.5;
    cloud->points[6*i + 3].y = (float) (i % 16) / 16.0f - 0.5f;
    cloud->points[6*i + 3].z = (float) (i / 16) / 16.0f - 0.5f;
    cloud->points[6*i + 4].rgba = 0x0000ffff;
    cloud->points[6*i + 4].y = - 0.5;
    cloud->points[6*i + 4].x = (float) (i % 16) / 16.0f - 0.5f;
    cloud->points[6*i + 4].z = (float) (i / 16) / 16.0f - 0.5f;
    cloud->points[6*i + 5].rgba = 0x00ffff00;
    cloud->points[6*i + 5].z = - 0.5;
    cloud->points[6*i + 5].y = (float) (i % 16) / 16.0f - 0.5f;
    cloud->points[6*i + 5].x = (float) (i / 16) / 16.0f - 0.5f;
  }

  for (int i = 0; i < 256; i++) {}
  std::cout << "Creating a cube piont cloud" << std::endl;
}

int main(int argc, char **argv)
{
  PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>());
  PointCloudFactory factory;
  factory.createCube(cloud);
  io::savePCDFileASCII("cube.pcd", *cloud);

  pcl::visualization::PCLVisualizer viewer("Point Cloud Factory Test");
  viewer.addSphere (PointXYZ(), 1);

  while(!viewer.wasStopped()) {
    viewer.spinOnce(1, true);
  }
}
