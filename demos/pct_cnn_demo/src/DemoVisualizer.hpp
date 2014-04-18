#ifndef DemoVisualizer_HEADER
#define DemoVisualizer_HEADER
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/registration/icp.h>

class DemoVisualizer  {
	public:
	DemoVisualizer();
		void setSourcePC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
		void setTargetPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
		void setTransformedPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
		void show();
		void close();
		/*--- wrapped function calls --*/
		bool wasStopped();
		void spinOnce();
		static void scaleToXAxis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr,float max);
		static void moveToCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
		static void rotateZAxis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
				float degrees) ;
		inline int getRequestedTransformations(){return requestedTransformations;}
	private:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed;
		pcl::visualization::PCLVisualizer viewer;
		void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event);
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

		/** /brief Viewport 1 **/
		int vp1 = 1;
		int vp2 = 2;
		int vp3 = 3;
		bool sourceLocked = false;

		int requestedTransformations=0;

		/** /brief ID for Point Cload */
		static std::string const pcTarget;
		static std::string const pcSource;
		static std::string const pcTransformed;
	};

#endif
