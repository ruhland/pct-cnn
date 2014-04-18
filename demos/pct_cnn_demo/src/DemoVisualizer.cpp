#include "DemoVisualizer.hpp"
#include <iostream>
#include <vector>
#include <pcl/visualization/common/common.h>

std::string const DemoVisualizer::source_cloud_id = "source_cloud";
std::string const DemoVisualizer::target_cloud_id = "target_cloud";
std::string const DemoVisualizer::transformed_cloud_id = "transformed_cloud";

DemoVisualizer::DemoVisualizer() : viewer("Demo Visualizer"), viewports(6)
{
    for (int i = 0; i < 6; i++) {
        viewports[i] = i + 1;
    }

    viewer.createViewPort (0.0, 0.0, 0.33, 0.5, viewports[0]);
    viewer.createViewPort (0.33, 0.0, 0.66, 0.5, viewports[1]);
    viewer.createViewPort (0.66, 0.0, 1.0, 0.5, viewports[2]);
    viewer.createViewPort (0.0, 0.5, 0.33, 1.0, viewports[3]);
    viewer.createViewPort (0.33, 0.5, 0.66, 1.0, viewports[4]);
    viewer.createViewPort (0.66, 0.5, 1.0, 1.0, viewports[5]);
}

void DemoVisualizer::show(){
    //viewer.initCameraParameters();
    viewer.setPointCloudRenderingProperties(
            visualization::PCL_VISUALIZER_POINT_SIZE, 3, source_cloud_id);
    viewer.setCameraPosition(0, 0, 980000, 0, 1, 0, 0);
    viewer.setCameraClipDistances(0.01f,1.0e10);
    viewer.setSize(1280, 1024); // Visualiser window size
}

void DemoVisualizer::close()
{
    cout << "[DemoVisualizer::close] Closing the visualizer..." << endl;
}

bool DemoVisualizer::wasStopped()
{
    return viewer.wasStopped();
}

void DemoVisualizer::spinOnce()
{
    viewer.spinOnce(1, true);
}

