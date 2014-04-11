#include "DemoVisualizer.hpp"

    DemoVisualizer::DemoVisualizer(): viewer("ICP demo"){
}
    void DemoVisualizer::setInputPC(pcl::PointCloud<PointT>::Ptr arg){
        viewer.addPointCloud(arg,pcInput,v1);
    }

    void DemoVisualizer::setTransformedPC(pcl::PointCloud<PointT>::Ptr arg){
        viewer.addPointCloud(arg,pcOutput,v1);  
    }

    void DemoVisualizer::show(){
    	viewer.createViewPort (0.0, 0.0, 0.33, 1.0, vp1);
	    viewer.createViewPort (0.33, 0.0, 0.66, 1.0, vp2);
        viewer.createViewPort (0.66, 0.0, 1.0, 1.0, vp3);
    }

    void DemoVisualizer::close(){
    }

