// disable warnings for deprecated code from pcl headers
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

/** Somehow when compiling with std=c++11 linux is not defined **/
#ifdef __linux
#define linux 1
#endif

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/image_viewer.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <boost/shared_ptr.hpp>

class KinectGrabber{

public:
	KinectGrabber();

	/**
	 * \brief connect to Kinect
	 */
	bool connect();

	/**
	 * \brief Get Number of the last extracted Face. Higher number means
	 * 			a new face was extracted
	 */
	int getFraceNr();

	/**
	 * \brief a face is extracted from latest captured kinect data
	 * 			if a face is found it is stored as latest Face
	 */
	void extractFace();

	/**
	 * \brief infinity loop executing extractFace
	 */
	void extractFaceLoop();

	/**
	 * \brief get the last face extracted with the extractFace method
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getLatestFace();

	/**
	 * \brief check if kinect is currently connected
	 */
	bool isConnected();
	void disconnect();
	~KinectGrabber();

private:
	void grabberCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud);
	void grabberCallbackImage(const boost::shared_ptr<openni_wrapper::Image>&,const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant);
	int faceNr;
	int latestFrameNr;
	int frameNr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr latestFace;
	pcl::Grabber* openniGrabber;
	std::mutex latestFaceMutex;
	std::mutex connectMutex;
	boost::shared_ptr<openni_wrapper::Image> lastImage;
	boost::shared_ptr<openni_wrapper::DepthImage> lastDepthImage;
	std::string faceCloudId;
	cv::CascadeClassifier facedetector;

	static const bool displayFaceDetection=false;

	template <typename PointT>
	void filterOutliers(typename pcl::PointCloud<PointT>::Ptr);
};

