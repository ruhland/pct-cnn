#ifndef DEMO_HEADER
#define DEMO_HEADER

#include <string>
#include "DemoVisualizer.hpp"
#include "PFHTransformStrategy.hpp"
#include "KinectGrabber.hpp"

class Demo {

public:
    Demo();
    void enableKinect();
    void setTargetFile(const std::string& name);
    void setSourceFile(const std::string& output);
    void run();
private:
	DemoVisualizer	visualizer;
	TransformStrategy *transformer;
	KinectGrabber kinect;

};
#endif
