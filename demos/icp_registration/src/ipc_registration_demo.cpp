// disable warnings for deprecated code from pcl headers (fopen, etc.)
#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
// also disable specific warnings from MSVC Compiler on Windows
#pragma warning(disable: 4305 4514 4711 4996)
#pragma warning(push, 1)
#if _MSC_VER > 1600
#define _MSC_VER_BAK _MSC_VER
#undef _MSC_VER
#define _MSC_VER 1600
#endif
#endif

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <boost/program_options.hpp>

// re-enable everything for Windows
#ifdef _MSC_VER
#undef _MSC_VER
#define _MSC_VER _MSC_VER_BAK
#pragma warning(pop)
#endif

using namespace pcl;

int main(int argc, char **argv) {
    PLYReader ply_reader;
    PCDWriter pcd_writer;

    PointCloud<PointXYZRGB>::Ptr source_pointer (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr target_pointer (new PointCloud<PointXYZRGB>);

    PointCloud<PointXYZRGB> output_cloud;

    std::string source_filename = argv[1];
    std::string target_filename = argv[2];

    ply_reader.read(source_filename, *source_pointer);
    ply_reader.read(target_filename, *target_pointer);

    IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;

#if PCL_MINOR_VERSION > 6
	icp.setInputSource(source_pointer);
#else
	icp.setInputCloud(source_pointer);
#endif
    icp.setInputTarget(target_pointer);

    icp.align(output_cloud);

    pcd_writer.write("icp_test_face.pcd", output_cloud);

    return 0;
}
