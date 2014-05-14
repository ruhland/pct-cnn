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

#include "DemoVisualizer.hpp"
#include <iostream>
#include <vector>
#include <pcl/visualization/common/common.h>
#include <pcl/common/impl/common.hpp>

// re-enable everything for Windows
#ifdef _MSC_VER
#undef _MSC_VER
#define _MSC_VER _MSC_VER_BAK
#undef _MSC_VER_BAK
#pragma warning(pop)
#endif


void DemoVisualizer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event) {
	if (event.getKeySym() == "space" && event.keyDown())
		requestedTransformations++;
	if (event.getKeyCode() == 'l')
		sourceLocked = true;
}

std::string const DemoVisualizer::pcSource = "SourcePointCloud";
std::string const DemoVisualizer::pcTarget = "TargetPointCloud";
std::string const DemoVisualizer::pcTransformed = "TransformedPointCloud";

DemoVisualizer::DemoVisualizer() :
		vp1(1), vp2(2), vp3(3), sourceLocked(false), requestedTransformations(0),
		viewer("ICP demo"), cloud_transformed(
				new pcl::PointCloud<pcl::PointXYZRGB>), icp() {
	viewer.createViewPort(0.0, 0.0, 0.33, 1.0, vp1);
	viewer.createViewPort(0.33, 0.0, 0.66, 1.0, vp2);
	viewer.createViewPort(0.66, 0.0, 1.0, 1.0, vp3);
        viewer.addCoordinateSystem();
	boost::function<
				void(const pcl::visualization::KeyboardEvent& event)> i = boost::bind(
				&DemoVisualizer::keyboardEventOccurred, this, _1);

	viewer.registerKeyboardCallback(i);
}
void DemoVisualizer::setSourcePC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr arg) {
	if (sourceLocked) {
		std::cout << "Viewer Source locked" << std::endl;
		return;
	}

	cloud_source = arg;
	//rotateZAxis(cloud_source,180);
	//moveToCenter(cloud_source);
	//scaleToXAxis(cloud_source, 2.0f);
	//moveToCenter(cloud_source);
	viewer.removePointCloud(pcSource);
	viewer.addPointCloud(cloud_source, pcSource, vp1);
	viewer.setPointCloudRenderingProperties(
				pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, pcSource);
}

void DemoVisualizer::scaleToXAxis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
		float maxScale) {

	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	pcl::PointXYZRGB min;
	pcl::PointXYZRGB max;
	pcl::getMinMax3D<pcl::PointXYZRGB>(*pc, min, max);
	std::cout << " scaleToXAxis before " << min << " max " << max << std::endl;
	float scale = maxScale / (max.x-min.x);
	transformation_matrix(0, 0) = scale;
	transformation_matrix(1, 1) = scale;
	transformation_matrix(2, 2) = scale;
	pcl::transformPointCloud(*pc, *pc, transformation_matrix);
	pcl::getMinMax3D<pcl::PointXYZRGB>(*pc, min, max);
	std::cout << " scaleToXAxis after " << min << " max " << max << std::endl;
}

void DemoVisualizer::rotateZAxis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
		float degrees) {
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	float theta = degrees * static_cast<float>(M_PI) / 180.0f; // The angle of rotation in radians
	transformation_matrix(0, 0) = cos(theta);
	transformation_matrix(0, 1) = -sin(theta);
	transformation_matrix(1, 0) = sin(theta);
	transformation_matrix(1, 1) = cos(theta);
	pcl::transformPointCloud(*pc, *pc, transformation_matrix);
}
void DemoVisualizer::moveToCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) {

	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	pcl::PointXYZRGB min;
	pcl::PointXYZRGB max;
	pcl::getMinMax3D<pcl::PointXYZRGB>(*pc, min, max);
	std::cout << " moveToCenter before " << min << " max " << max << std::endl;
	transformation_matrix(0, 3) = -(min.x + max.x) / 2;
	transformation_matrix(1, 3) = -(min.y + max.y) / 2;
	transformation_matrix(2, 3) = -(min.z + max.z) / 2;
	pcl::transformPointCloud(*pc, *pc, transformation_matrix);
	pcl::getMinMax3D<pcl::PointXYZRGB>(*pc, min, max);
	std::cout << " moveToCenter after" << min << " max " << max << std::endl;
}

void DemoVisualizer::setTargetPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr arg) {
	cloud_target = arg;
	//moveToCenter(cloud_target);
	//scaleToXAxis(cloud_target, 1.0f);
	viewer.removePointCloud(pcTarget);
	viewer.addPointCloud(cloud_target, pcTarget, vp2);
}

void DemoVisualizer::setTransformedPC(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr arg) {
	viewer.removePointCloud(pcTransformed);
	viewer.addPointCloud(arg, pcTransformed, vp3);
	viewer.setPointCloudRenderingProperties(
				pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, pcTransformed);
}

void DemoVisualizer::show() {
	//viewer.initCameraParameters();
	viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, pcTarget);
	viewer.setPointCloudRenderingProperties(
				pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, pcSource);
	viewer.setCameraPosition(0, 0, 9, 0, 1, 0, 0);
#if PCL_MINOR_VERSION > 6
	viewer.setCameraClipDistances(0.01f, 1.0e10);
	viewer.setSize(1280, 1024); // Visualiser window size
#endif
}

void DemoVisualizer::close() {
}

bool DemoVisualizer::wasStopped() {
	return viewer.wasStopped();
}

void DemoVisualizer::spinOnce() {
	viewer.spinOnce(1, true);
}

