#include "DemoVisualizer.hpp"
#include <iostream>
#include <vector>
#include <pcl/visualization/common/common.h>

std::string const DemoVisualizer::pcInput="InputPointCloud";
std::string const DemoVisualizer::pcTransformed="TransformedPointCloud";

    DemoVisualizer::DemoVisualizer(): viewer("ICP demo"){
    	viewer.createViewPort (0.0, 0.0, 0.33, 1.0, vp1);
	    viewer.createViewPort (0.33, 0.0, 0.66, 1.0, vp2);
        viewer.createViewPort (0.66, 0.0, 1.0, 1.0, vp3);
}
    void DemoVisualizer::setInputPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr arg){
        viewer.addPointCloud(arg,pcInput,vp1);
    }

    void DemoVisualizer::setTransformedPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr arg){
        viewer.addPointCloud(arg,pcTransformed,vp2);
    }

    void DemoVisualizer::show(){
    	//viewer.initCameraParameters();
    	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, pcInput);
        viewer.setCameraPosition(0, 0, 980000, 0, 1, 0, 0);
        viewer.setCameraClipDistances(0.01f,1.0e10);
        viewer.setSize(1280, 1024); // Visualiser window size
    }

    void DemoVisualizer::close(){
    }

    bool DemoVisualizer::wasStopped(){
    	return viewer.wasStopped();
    }

    void DemoVisualizer::spinOnce(){
    	viewer.spinOnce(1,true);
    }

