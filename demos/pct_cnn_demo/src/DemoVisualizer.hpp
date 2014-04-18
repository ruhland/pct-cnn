#ifndef DEMOVISUALIZER_HPP
#define DEMOVISUALIZER_HPP

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace std;

class DemoVisualizer
{
public:
    DemoVisualizer();

    vector<int> getViewports();

    template<typename PointT>
    void setCloud(typename PointCloud<PointT>::Ptr cloud,
            std::string id, int viewport)
    {
        viewer.addPointCloud(cloud, id, viewport);
    }

    template<typename PointT>
    void setSourceCloud(typename PointCloud<PointT>::Ptr cloud)
    {
        setCloud<PointT>(cloud, source_cloud_id, viewports[0]);
    }

    template<typename PointT>
    void setTargetCloud(typename PointCloud<PointT>::Ptr cloud)
    {
        setCloud<PointT>(cloud, target_cloud_id, viewports[1]);
    }

    template<typename PointT>
    void setTransformedCloud(
            typename PointCloud<PointT>::Ptr cloud)
    {
        setCloud<PointT>(cloud, transformed_cloud_id, viewports[2]);
    }

    void show();
    void close();

    /*--- wrapped function calls --*/
    bool wasStopped();
    void spinOnce();

private:
    visualization::PCLVisualizer viewer;

    vector<int> viewports;

    /** /brief ID for Point Cload */
    static std::string const source_cloud_id;
    static std::string const target_cloud_id;
    static std::string const transformed_cloud_id;
};

#endif
