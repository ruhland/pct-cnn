#ifndef DemoVisualizer
#define DemoVisualizer

class DemoVisualizer{


public:
    DemoVisualizer();
    void setInputPC(pcl::PointCloud<PointT>::Ptr);
    void setTransformedPC(pcl::PointCloud<PointT>::Ptr);
    void show();
    void close();
private:
    pcl::PointCloud<PointT>::Ptr cloud_in;
    pcl::PointCloud<PointT>::Ptr cloud_transformed;
    pcl::visualization::PCLVisualizer viewer;
    
    /** /brief Viewport 1 **/
    int vp1(1);
    int vp2(2);
    int vp3(3);

    /** /brief ID for Point Cload */
    static const std::string pcInput= "InputPointCloud";
    static const std::string pcTransformed= "TransformedPointCloud";
    };

#endif
