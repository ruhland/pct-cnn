#ifndef DEMO_HEADER
#define DEMO_HEADER

#include <string>
#include "DemoVisualizer.hpp"
#include "PFHTransformStrategy.hpp"
#include "KinectGrabber.hpp"
#include "Configuration.hpp"

class Demo {

public:
    Demo();
    void enableKinect();
    void setTargetFile(const std::string& name);
    void setSourceFile(const std::string& output);
    void setConfigFile(const std::string& file);
    void run();
private:
	DemoVisualizer	visualizer;
	TransformStrategy<PointXYZRGB> *transformer;
	KinectGrabber kinect;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloud;
	boost::shared_ptr<Configuration> configuration;
};
#endif
