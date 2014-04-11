#ifndef DEMO
#define DEMO

#include <string>

class Demo {

public:
    Demo();
    void enableKinect();
    void setInputFile(const std::string& name);
    void setOutputFile(const std::string& output);
    void run();
private:


};
#endif
