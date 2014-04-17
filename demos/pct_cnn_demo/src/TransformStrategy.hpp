#ifndef TRANSFORMSTRATEGY_HPP
#define TRANSFORMSTRATEGY_HPP

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>

using namespace pcl;
using namespace std;

class TransformStrategy
{
public:
    virtual ~TransformStrategy() {}
    virtual PointCloud<PointXYZRGB>::Ptr transform(const PointCloud<PointXYZRGB>::Ptr source, const PointCloud<PointXYZRGB>::Ptr target) = 0;
};

#endif
