// disable warnings for deprecated code from pcl headers (fopen, etc.)
#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
// also disable specific warnings from MSVC Compiler on Windows
#pragma warning(disable: 4503 4305 4514 4711 4996)
#pragma warning(push, 1)
#endif

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

// re-enable everything for Windows
#ifdef _MSC_VER
#pragma warning(pop)
#endif

int main(int argc, char **argv) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

    cloud_in->width = 5;
    cloud_in->height = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);

    for (size_t i = 0; i < cloud_in->points.size(); i++) {
        cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cout << "Saved " << cloud_in->points.size() << " data points to \
        input:" << std::endl;

    for (size_t i = 0; i < cloud_in->points.size(); i++) {
        std::cout << "  " << cloud_in->points[i].x << " " \
            << cloud_in->points[i].y << " " << cloud_in->points[i].z \
            << std::endl;
    }

    *cloud_out = *cloud_in;

    std::cout << "size: " << cloud_out->points.size() << std::endl;

    for (size_t i = 0; i < cloud_out->points.size(); i++) {
        cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    }

    std::cout << "Transformed " << cloud_in->points.size() << " data points:"
        << std::endl;

    for (size_t i = 0; i < cloud_out->points.size(); i++) {
        std::cout << "  " << cloud_out->points[i].x << " " << \
            cloud_out->points[i].y << " " << cloud_out->points[i].z << \
            std::endl;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

#if PCL_MINOR_VERSION > 6
    icp.setInputSource(cloud_in);
#else
	icp.setInputCloud(cloud_in);
#endif

    icp.setInputTarget(cloud_out);

    pcl::PointCloud<pcl::PointXYZ> Final;

    icp.align(Final);

    std::cout << "has converged: " << icp.hasConverged() << " score: "
        << icp.getFitnessScore() << std::endl;

    std::cout << icp.getFinalTransformation() << std::endl;

    return 0;
}
