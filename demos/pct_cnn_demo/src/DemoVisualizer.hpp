#ifndef DemoVisualizer_HEADER
#define DemoVisualizer_HEADER
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>

class DemoVisualizer  {
	public:
	DemoVisualizer();
		void setInputPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
		void setTransformedPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
		void show();
		void close();
		/*--- wrapped function calls --*/
		bool wasStopped();
		void spinOnce();
	private:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed;
		pcl::visualization::PCLVisualizer viewer;

		/** /brief Viewport 1 **/
		int vp1 = 1;
		int vp2 = 2;
		int vp3 = 3;

		/** /brief ID for Point Cload */
		static std::string const pcInput;
		static std::string const pcTransformed;
	};

#endif
