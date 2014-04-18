#include <boost/program_options.hpp>
#include <iostream>
#include <exception>
#include <pcl/io/pcd_io.h>
#include <thread>
#include "Demo.hpp"

Demo::Demo(): visualizer(),kinect(){
   std::cout<<"Demo created";
   transformer= new PFHTransformStrategy();
}

void Demo::setTargetFile(const std::string& name){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in 	(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile (name, *cloud_in);
	visualizer.setTargetPC(cloud_in);
	//visualizer.setTransformedPC(transformer->transform(cloud_in, cloud_in));
}
void Demo::setSourceFile(const std::string& output){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in 	(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile (output, *cloud_in);
	visualizer.setSourcePC(cloud_in);
}
void Demo::enableKinect(){
	if(!kinect.connect())
		std::cout<<"Not possible to connect to Kinect";
}
void Demo::run(){
	int lastface=0;
	visualizer.show();
	std::thread extractFace(&KinectGrabber::extractFaceLoop,&kinect);
	while(!visualizer.wasStopped()){
		visualizer.spinOnce ();
		if(kinect.isConnected() && lastface!=kinect.getFraceNr()){
			lastface=kinect.getFraceNr();
			visualizer.setSourcePC(kinect.getLatestFace());
		}
	}
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
    demo.run();
    }
    /*catch(boost::program_options::error& e){
            std::cout<<"Exception in boost :"<<e.what();
    }*/
    catch(const std::exception& e){
        std::cout<<"Exception in programm:"<<e.what()<<std::endl;    
    }
     
}
