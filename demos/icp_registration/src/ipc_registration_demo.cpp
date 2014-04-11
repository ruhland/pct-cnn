#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <boost/program_options.hpp>

using namespace pcl;

int main(int argc, char **argv) {
    PLYReader ply_reader;
    PCDWriter pcd_writer;

    PointCloud<PointXYZRGB>::Ptr source_pointer (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr target_pointer (new PointCloud<PointXYZRGB>);

    PointCloud<PointXYZRGB> output_cloud;

    std::string source_filename = argc[1];
    std::string target_filename = argc[2];

    ply_reader.read(source_filename, *source_pointer);
    ply_reader.read(target_filename, *target_pointer);

    IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;

    icp.setInputSource(source_pointer);
    icp.setInputTarget(target_pointer);

    icp.align(output_cloud);

    pcd_writer.write("icp_test_face.pcd", output_cloud);

    return 0;
}
