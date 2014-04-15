#ifndef PFHTRANSFORMSTRATEGY_HPP
#define PFHTRANSFORMSTRATEGY_HPP

#include "TransformStrategy.hpp"

class PFHTransformStrategy : public TransformStrategy
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy(source);
	int move=0;
		for(int i=0;i< (*copy).size();i++){
			(*copy)[i].x=(*copy)[i].x+100000;
			move=(*copy)[i].x;
		}
        return copy;
    }
};

#endif
