// disable warnings for deprecated code from pcl headers (fopen, etc.)
#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
// also disable specific warnings from MSVC Compiler on Windows
#pragma warning(disable: 4503 4305 4514 4711 4996)
#pragma warning(push, 1)
#endif

#include "KinectGrabber.hpp"
#include <iostream>
#include <XnCppWrapper.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/openni_camera/openni_image_rgb24.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <thread>

// re-enable everything for Windows
#ifdef _MSC_VER
#pragma warning(pop)
#endif

namespace pcl {
typedef union {
	struct {
		unsigned char Blue;
		unsigned char Green;
		unsigned char Red;
		unsigned char Alpha;
	};
	float float_value;
	uint32_t long_value;
} RGBValue;
}

KinectGrabber::KinectGrabber() :
		faceNr(0), latestFrameNr(0), frameNr(0), openniGrabber(nullptr),
		latestFace(new pcl::PointCloud<pcl::PointXYZRGB>), faceCloudId(
				"ExportKinect"), lastImage(), lastDepthImage(), facedetector(
				"/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml") {
}

bool KinectGrabber::connect() {
	std::lock_guard<std::mutex> lock(connectMutex);
	openniGrabber = new pcl::OpenNIGrabber();
	if (openniGrabber == 0)
		return false;
	//boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
	//		boost::bind(&KinectGrabber::grabberCallback, this, _1);
	//openniGrabber->registerCallback(f);

	boost::function<
			void(const boost::shared_ptr<openni_wrapper::Image>&,
					const boost::shared_ptr<openni_wrapper::DepthImage>&,
					float constan)> i = boost::bind(
			&KinectGrabber::grabberCallbackImage, this, _1, _2, _3);
	openniGrabber->registerCallback(i);
	openniGrabber->start();
	return true;
}

void KinectGrabber::disconnect(){
	if(!isConnected())
		return;
	std::lock_guard<std::mutex> lock(connectMutex);
	std::lock_guard<std::mutex> lock2(latestFaceMutex);
	openniGrabber->stop();
	openniGrabber=0;
	return;
}

bool KinectGrabber::isConnected() {
	std::lock_guard<std::mutex> lock(connectMutex);
	return openniGrabber != 0;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr KinectGrabber::getLatestFace() {
	std::lock_guard<std::mutex> lock(latestFaceMutex);
	return latestFace;
}

void KinectGrabber::extractFaceLoop() {
	while (isConnected()) {
		extractFace();
	}
}

/*
 * \brief filter outlier should be optimized later StatisticalOutlierRemoval takes ~1 second
 *
 */
template<typename PointT>
void KinectGrabber::filterOutliers(
		typename pcl::PointCloud<PointT>::Ptr cloud) {
	pcl::RadiusOutlierRemoval<PointT> outrem;
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(2);
	outrem.setMinNeighborsInRadius(400);
	outrem.filter(*cloud);

	/*pcl::StatisticalOutlierRemoval<PointT> sor;
	 sor.setInputCloud(cloud);
	 sor.setMeanK(800);
	 sor.setStddevMulThresh(0.5f);
	 sor.filter(*cloud);*/
}

int KinectGrabber::getFraceNr() {
	std::lock_guard<std::mutex> lock(latestFaceMutex);
	return faceNr;
}

void KinectGrabber::extractFace() {
	clock_t begin_time = clock();
	std::unique_lock<std::mutex> lock(latestFaceMutex);
	if (frameNr == latestFrameNr) {
		//std::cout << "No new data" << std::endl;
		 std::this_thread::yield();
		return;
	}

	/*
	 * Copy pointers for save work
	 */
	boost::shared_ptr<openni_wrapper::Image> lastImage(this->lastImage);
	boost::shared_ptr<openni_wrapper::DepthImage> lastDepthImage(
			this->lastDepthImage);
	lock.unlock();

	if (lastImage->getEncoding() != openni_wrapper::Image::RGB) {
		boost::shared_array<unsigned char> rgb_data(
				new unsigned char[lastImage->getWidth() * lastImage->getHeight()
						* 3]);
		lastImage->fillRGB(lastImage->getWidth(), lastImage->getHeight(),
				rgb_data.get());


		cv::Mat matimg(lastImage->getHeight(), lastImage->getWidth(), CV_8UC3,
				rgb_data.get(), 0);
		cv::cvtColor(matimg, matimg, CV_BGR2RGB);


		std::vector<cv::Rect> faces;
		facedetector.detectMultiScale(matimg, faces, 2, 6);

		cv::Rect selectedFace;
		for (cv::Rect f : faces) {
			cv::rectangle(matimg, f, cv::Scalar(255, 0, 0));
			selectedFace = f;
		}
		cv::imshow("CVFace1", matimg);
		cv::waitKey(10);
		if (selectedFace.width == 0) {
			//std::cerr << "No face found!";
		} else if (lastImage->getWidth() != lastDepthImage->getWidth()
				|| lastImage->getHeight() != lastDepthImage->getHeight()) {
			std::cerr
					<< "strange Kinect depth image different size than RGB image .. implement skipping";
		} else {
			boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud(
					new pcl::PointCloud<pcl::PointXYZRGB>);
			cloud->header.frame_id = "ExportedKinect";
			cloud->height = selectedFace.height - 1;
			cloud->width = selectedFace.width - 1;
			cloud->is_dense = false;
			cloud->points.resize(cloud->height * cloud->width);

			//Specific for Kinect but device_ in grabber is private. Important for scaling
			float constant_x = 0.0017f;
			float constant_y = 0.0017f;
			float centerX = ((float) cloud->width - 10.f) / 2.f;
			float centerY = ((float) cloud->height - 10.f) / 2.f;

			int pointnr = 0;
			pcl::RGBValue color;
			color.Alpha = 0;

			const XnDepthPixel* depth_map =
					lastDepthImage->getDepthMetaData().Data();

			for (int y = selectedFace.y + 1;
					y < selectedFace.y + selectedFace.height - 1;
					++y, pointnr++) {
				for (int x = selectedFace.x + 1;
						x < selectedFace.x + selectedFace.width - 1;
						++x, pointnr++) {
					pcl::PointXYZRGB& pt = cloud->points[pointnr];
					int arraypos = x + y * lastDepthImage->getWidth();
					pt.z = depth_map[arraypos] * 0.01f;
					pt.x = (static_cast<float>(x) - centerX) * pt.z
							* constant_x;
					pt.y = (static_cast<float>(y) - centerY) * pt.z
							* constant_y;
					color.Red = rgb_data[arraypos * 3 + 2];
					color.Green = rgb_data[arraypos * 3 + 1];
					color.Blue = rgb_data[arraypos * 3];
					pt.rgba = color.long_value;
				}
			}

			filterOutliers<pcl::PointXYZRGB>(cloud);
			lock.lock();
			latestFace = cloud;
			faceNr++;
			lock.unlock();
		}
	}

}

void KinectGrabber::grabberCallbackImage(
		const boost::shared_ptr<openni_wrapper::Image>& image,
		const boost::shared_ptr<openni_wrapper::DepthImage>& depthImage,
		float constant) {
	//std::cout << "New image data nr:" << frameNr << std::endl;
	std::lock_guard<std::mutex> lock(latestFaceMutex);
	frameNr++;
	lastImage = image;
	lastDepthImage = depthImage;
}
/*
 void KinectGrabber::grabberCallback(
 const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud) {
 std::cout << "New data nr:" << facenr << std::endl;
 std::lock_guard<std::mutex> lock(latestFaceMutex);
 facenr++;
 pcl::copyPointCloud(*cloud, *latestFace);
 }
 */
KinectGrabber::~KinectGrabber() {
	if (openniGrabber != 0)
		delete openniGrabber;
	openniGrabber = 0;
}
