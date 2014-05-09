// disable warnings for deprecated code from pcl headers (fopen, etc.)
#ifdef _MSC_VER
	#ifndef _CRT_SECURE_NO_WARNINGS
	#define _CRT_SECURE_NO_WARNINGS
	#endif
// also disable specific warnings from MSVC Compiler on Windows
#pragma warning(disable: 4514 4711 4996)
#pragma warning(push, 1)
#endif

#include <boost/program_options.hpp>
#include <iostream>
#include <exception>
#include <pcl/io/pcd_io.h>
#include <thread>
#include "Demo.hpp"
#include "CNNTransformStrategy.hpp"
// re-enable warnings for Windows
#ifdef _MSC_VER
#pragma warning(pop)
#endif

Demo::Demo(): visualizer(),kinect(),targetCloud(new pcl::PointCloud<pcl::PointXYZRGB>),sourceCloud(new pcl::PointCloud<pcl::PointXYZRGB>),configuration(new Configuration()){
   std::cout<<"Demo created"<<std::endl;
  // transformer= new PFHTransformStrategy<PointXYZRGB>();
   transformer= new CNNTransformStrategy<PointXYZRGB>();
}

void Demo::setTargetFile(const std::string& name){
	pcl::io::loadPCDFile (name, *targetCloud);
	DemoVisualizer::moveToCenter(targetCloud);
	DemoVisualizer::scaleToXAxis(targetCloud,		1.0f);
	visualizer.setTargetPC(targetCloud);
}
void Demo::setConfigFile(const std::string& file){
	configuration->loadFromFile(file);
	std::cout<<"Configuration read: nearestNeighbors: "<<configuration->getNearestNeighborsToSearch()<<std::endl;
}

void Demo::setSourceFile(const std::string& output){
	pcl::io::loadPCDFile (output, *sourceCloud);
	DemoVisualizer::moveToCenter(sourceCloud);
	DemoVisualizer::scaleToXAxis(sourceCloud,		1.0f);
	visualizer.setSourcePC(sourceCloud);
}
void Demo::enableKinect(){
	if(!kinect.connect())
		std::cout<<"Not possible to connect to Kinect";
}
void Demo::run(){
	int lastface=0;
	visualizer.show();
	int lastrequestedTransofmation=0;
	std::thread extractFace(&KinectGrabber::extractFaceLoop,&kinect);
	while(!visualizer.wasStopped()){
		visualizer.spinOnce ();
		if(kinect.isConnected() && lastface!=kinect.getFraceNr()){
			lastface=kinect.getFraceNr();
			sourceCloud=kinect.getLatestFace();
			visualizer.setSourcePC(sourceCloud);
		}
		if(lastrequestedTransofmation< visualizer.getRequestedTransformations()){
			lastrequestedTransofmation=visualizer.getRequestedTransformations();
			visualizer.setTransformedPC(transformer->transform(sourceCloud,targetCloud));
		}
	}
	kinect.disconnect();
	extractFace.join();
}

int main (int argc, char **argv)
    {
    try{
    // Parsing input parameters:
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("t",boost::program_options::value<std::string>()->required(),"target file (required)")
        ("s",boost::program_options::value<std::string>(),"source file")
        ("f",boost::program_options::value<std::string>(),"file for parameter configuration")
        ("k","enable Kinect instead of source file")
    ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);

     if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    boost::program_options::notify(vm);    
    
    Demo demo;
    if(vm.count("k"))
        demo.enableKinect();
    demo.setTargetFile(vm["t"].as<std::string>());
    if(vm.count("s"))
        demo.setSourceFile(vm["s"].as<std::string>());
    if(vm.count("f"))
    	demo.setConfigFile(vm["f"].as<std::string>());
    demo.run();
    std::cout<<"Program exit\n";
    }
    /*catch(boost::program_options::error& e){
            std::cout<<"Exception in boost :"<<e.what();
    }*/
    catch(const std::exception& e){
        std::cout<<"Exception in programm:"<<e.what()<<std::endl;    
    }
    return 0;
}
