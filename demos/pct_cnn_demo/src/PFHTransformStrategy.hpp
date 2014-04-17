#ifndef PFHTRANSFORMSTRATEGY_HPP
#define PFHTRANSFORMSTRATEGY_HPP

#include "TransformStrategy.hpp"

class PFHTransformStrategy : public TransformStrategy {
public:
    PFHTransformStrategy();
    PointCloud<PointXYZRGB>::Ptr transform(const PointCloud<PointXYZRGB>::Ptr source, const PointCloud<PointXYZRGB>::Ptr target);
};

#endif
