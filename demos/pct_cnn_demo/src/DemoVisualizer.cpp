// disable warnings for deprecated code from pcl headers (fopen, etc.)
#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
// also disable specific warnings from MSVC Compiler on Windows
#pragma warning(disable: 4503 4305 4514 4711 4996)
#pragma warning(push, 1)
#endif

#include "DemoVisualizer.hpp"
#include <iostream>
#include <vector>
#include <pcl/visualization/common/common.h>
#include <pcl/common/impl/common.hpp>

// re-enable everything for Windows
#ifdef _MSC_VER
#pragma warning(pop)
#endif

void DemoVisualizer::keyboardEventOccurred(
		const pcl::visualization::KeyboardEvent& event) {
	if (event.getKeySym() == "space" && event.keyDown())
		requestedTransformations++;
	if (event.getKeyCode() == 'l' && event.keyDown()){
		sourceLocked = !sourceLocked;
		if(sourceLocked)
			std::cout << "Viewer Source is now locked" << std::endl;
		else
			std::cout << "Viewer Source is now unlocked" << std::endl;
	}
}

std::string const DemoVisualizer::pcSource = "SourcePointCloud";
std::string const DemoVisualizer::pcTarget = "TargetPointCloud";
std::string const DemoVisualizer::pcTransformed = "TransformedPointCloud";

DemoVisualizer::DemoVisualizer() :
		vp1(1), vp2(2), vp3(3), sourceLocked(false), requestedTransformations(
				0), viewer("ICP demo"), cloud_transformed(
				new pcl::PointCloud<pcl::PointXYZRGB>), icp() {
	viewer.createViewPort(0.0, 0.0, 0.33, 1.0, vp1);
	viewer.createViewPort(0.33, 0.0, 0.66, 1.0, vp2);
	viewer.createViewPort(0.66, 0.0, 1.0, 1.0, vp3);
	viewer.addCoordinateSystem();
	boost::function<void(const pcl::visualization::KeyboardEvent& event)> i =
			boost::bind(&DemoVisualizer::keyboardEventOccurred, this, _1);

	viewer.registerKeyboardCallback(i);
}

void DemoVisualizer::transformFaceToNormal(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc) {
	rotateXYZ(pc, 0, 0, 180);
	moveToCenter(pc);
	scaleToXAxis(pc, 2.0f);
	moveToCenter(pc);
}

void DemoVisualizer::setSourcePC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr arg) {
	if (sourceLocked) {
		std::cout << "Viewer Source locked" << std::endl;
		return;
	}

	cloud_source = arg;
	viewer.removePointCloud(pcSource);
	viewer.addPointCloud(cloud_source, pcSource, vp1);
	viewer.setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, pcSource);
}

void DemoVisualizer::screenshot(pcl::PointCloud<pcl::PointXYZRGB>::Ptr arg) {
	int viewport=42;
	pcl::visualization::PCLVisualizer viewerscreenshot("screenshot");
	viewerscreenshot.createViewPort(0.0, 0.0, 1.0, 1.0, viewport);
	std::string cloudname= "tscreenshot";
	viewerscreenshot.addPointCloud(arg,cloudname,viewport);
	viewerscreenshot.setPointCloudRenderingProperties(
				pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloudname);
	viewerscreenshot.setCameraPosition(0, 0, 3, 0, 1, 0, 0);
	#if PCL_MINOR_VERSION > 6
		viewerscreenshot.setCameraClipDistances(0.01f, 1.0e10);
		viewerscreenshot.setSize(1024, 1024); // Visualiser window size
	#endif
	viewerscreenshot.spinOnce(1,true);
	viewerscreenshot.saveScreenshot("screen.png");

}

void DemoVisualizer::scaleToXAxis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
		float maxScale) {

	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	pcl::PointXYZRGB min;
	pcl::PointXYZRGB max;
	pcl::getMinMax3D<pcl::PointXYZRGB>(*pc, min, max);
	std::cout << " scaleToXAxis before " << min << " max " << max << std::endl;
	float scale = maxScale / (max.x - min.x);
	transformation_matrix(0, 0) = scale;
	transformation_matrix(1, 1) = scale;
	transformation_matrix(2, 2) = scale;
	pcl::transformPointCloud(*pc, *pc, transformation_matrix);
	pcl::getMinMax3D<pcl::PointXYZRGB>(*pc, min, max);
	std::cout << " scaleToXAxis after " << min << " max " << max << std::endl;
}

void DemoVisualizer::rotateXYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
		float xdegrees, float ydegrees, float zdegrees) {
	Eigen::Matrix4f transformation_matrix_z = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transformation_matrix_x = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transformation_matrix_y = Eigen::Matrix4f::Identity();
// A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	float theta = zdegrees * static_cast<float>(M_PI) / 180.0f; // The angle of rotation in radians
	transformation_matrix_z(0, 0) = cos(theta);
	transformation_matrix_z(0, 1) = -sin(theta);
	transformation_matrix_z(1, 0) = sin(theta);
	transformation_matrix_z(1, 1) = cos(theta);

	theta = xdegrees * static_cast<float>(M_PI) / 180.0f; // The angle of rotation in radians
	transformation_matrix_x(1, 1) = cos(theta);
	transformation_matrix_x(1, 2) = -sin(theta);
	transformation_matrix_x(2, 1) = sin(theta);
	transformation_matrix_x(2, 2) = cos(theta);

	theta = ydegrees * static_cast<float>(M_PI) / 180.0f; // The angle of rotation in radians
	transformation_matrix_y(0, 0) = cos(theta);
	transformation_matrix_y(2, 0) = -sin(theta);
	transformation_matrix_y(0, 2) = sin(theta);
	transformation_matrix_y(2, 2) = cos(theta);

	Eigen::Matrix4f transformation = transformation_matrix_z
			* transformation_matrix_x * transformation_matrix_y;

	pcl::transformPointCloud(*pc, *pc, transformation);
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

