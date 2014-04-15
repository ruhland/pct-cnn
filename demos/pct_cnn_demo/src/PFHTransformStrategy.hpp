#ifndef PFHTRANSFORMSTRATEGY_HEADER
#define PFHTRANSFORMSTRATEGY_HEADER

#include "TransformStrategy.hpp"

class PFHTransformStrategy:public TransformStrategy{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy(input);
		int move=0;
		for(int i=0;i< (*copy).size();i++){
			(*copy)[i].x=(*copy)[i].x+100000;
			move=(*copy)[i].x;
		}
		return copy;
	}
};
#endif
