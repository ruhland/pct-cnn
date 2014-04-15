#ifndef TRANSFORMSTRATEGY_HEADER
#define TRANSFORMSTRATEGY_HEADER

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>

class TransformStrategy{
public:
	virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input)=0;
};
#endif
