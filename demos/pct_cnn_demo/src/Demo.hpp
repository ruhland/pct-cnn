#ifndef DEMO_HEADER
#define DEMO_HEADER

#include <string>
#include "DemoVisualizer.hpp"
#include "PFHTransformStrategy.hpp"
class Demo {

public:
    Demo();
    void enableKinect();
    void setInputFile(const std::string& name);
    void setOutputFile(const std::string& output);
    void run();
private:
	DemoVisualizer	visualizer;
	TransformStrategy *transformer;

};
#endif
