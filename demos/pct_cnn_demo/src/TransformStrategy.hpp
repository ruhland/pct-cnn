#ifndef TRANSFORMSTRATEGY_HPP
#define TRANSFORMSTRATEGY_HPP

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>

class TransformStrategy
{
public:
    virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target) = 0;
};

#endif
