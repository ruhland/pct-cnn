#ifndef DEMO_HEADER
#define DEMO_HEADER

#include <string>

#include <pcl/point_types.h>

#include "DemoVisualizer.hpp"
#include "PFHTransformStrategy.hpp"

using namespace std;
using namespace pcl;

class Demo
{
public:

    Demo();

    void enableKinect();

    void setSourceCloud(const std::string &name,
            PointCloud<PointXYZRGB>::Ptr &source_cloud);

    void setTargetCloud(const std::string &output,
            PointCloud<PointXYZRGB>::Ptr &target_cloud);

    void transformClouds(PointCloud<PointXYZRGB>::Ptr source,
            PointCloud<PointXYZRGB>::Ptr target);

    void setOutputFile(const std::string &output);

    void run();

private:

    DemoVisualizer visualizer;
    TransformStrategy<PointXYZRGB> *transformer;

};
#endif
