#ifndef TRANSFORMSTRATEGY_HPP
#define TRANSFORMSTRATEGY_HPP

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>

using namespace pcl;
using namespace std;


template<typename PointT>
class TransformStrategy
{
public:
    virtual ~TransformStrategy<PointT>() {}
    virtual typename PointCloud<PointT>::Ptr transform(
            const typename PointCloud<PointT>::Ptr source,
            const typename PointCloud<PointT>::Ptr target) = 0;
    virtual void loadDefaultPresets() = 0;
};

#endif
