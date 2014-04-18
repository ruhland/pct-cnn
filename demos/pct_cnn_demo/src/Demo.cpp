#include <boost/program_options.hpp>
#include <iostream>
#include <exception>
#include <pcl/io/pcd_io.h>

#include "Demo.hpp"

Demo::Demo(): visualizer()
{
   std::cout << "Demo created";
   transformer = new PFHTransformStrategy<PointXYZRGB>();
}

void Demo::setSourceCloud(const std::string &filename,
        PointCloud<PointXYZRGB>::Ptr &source_cloud)
{
    io::loadPCDFile (filename, *source_cloud);
    visualizer.setSourceCloud<PointXYZRGB>(source_cloud);
}

void Demo::setTargetCloud(const std::string &filename,
        PointCloud<PointXYZRGB>::Ptr &target_cloud)
{
    io::loadPCDFile (filename, *target_cloud);
    visualizer.setTargetCloud<PointXYZRGB>(target_cloud);
}

void Demo::transformClouds(PointCloud<PointXYZRGB>::Ptr source,
        PointCloud<PointXYZRGB>::Ptr target)
{
    visualizer.setTransformedCloud<PointXYZRGB>(
            transformer->transform(source, target));
}

void Demo::setOutputFile(const std::string& output) {}

void Demo::enableKinect() {}

void Demo::run()
{
    visualizer.show();
    while(!visualizer.wasStopped()){
        visualizer.spinOnce ();
    }
}

int main (int argc, char **argv)
{
    try {
        // Parsing input parameters:
        boost::program_options::options_description desc("Allowed options");
        desc.add_options()
        ("help", "produce help message")
        ("s",boost::program_options::value<std::string>()->required(),"source cloud file (required)")
        ("t",boost::program_options::value<std::string>()->required(),"target cloud file (required)")
        ("o",boost::program_options::value<std::string>(),"output file")
        ("k","enable Kinect")
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

    PointCloud<PointXYZRGB>::Ptr source_cloud (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr target_cloud (new PointCloud<PointXYZRGB>);

    demo.setSourceCloud(vm["s"].as<std::string>(), source_cloud);
    demo.setTargetCloud(vm["t"].as<std::string>(), target_cloud);
    demo.transformClouds(source_cloud, target_cloud);

    if(vm.count("o"))
        demo.setOutputFile(vm["o"].as<std::string>());
    demo.run();
    }

    /*catch(boost::program_options::error& e){
            std::cout<<"Exception in boost :"<<e.what();
    }*/
    catch(const std::exception& e){
        std::cout<<"Exception in programm:"<<e.what()<<std::endl;    
    }
}
