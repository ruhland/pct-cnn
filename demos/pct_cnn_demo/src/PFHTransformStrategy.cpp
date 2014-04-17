#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "PFHTransformStrategy.hpp"

// Downsample a point cloud by using a voxel grid.
// See http://pointclouds.org/documentation/tutorials/voxel_grid.php
void filter (PointCloud<PointXYZRGB>::Ptr cloud, 
        PointCloud<PointXYZRGB>::Ptr cloud_filtered, float leaf_size=0.01f)
{
    PointCloud<PointXYZRGB>::Ptr tmp_ptr1(new PointCloud<PointXYZRGB>);

    VoxelGrid<PointXYZRGB> vox_grid;
    vox_grid.setInputCloud(cloud);
    vox_grid.setSaveLeafLayout(true);
    vox_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox_grid.filter(*tmp_ptr1);

    PointCloud<PointXYZRGB>::Ptr tmp_ptr2(new PointCloud<PointXYZRGB>);

    PassThrough<PointXYZRGB> pass1;
    pass1.setInputCloud(tmp_ptr1);
    pass1.setFilterFieldName("z");
    pass1.setFilterLimits(0.0, 3.0);
    pass1.filter(*tmp_ptr2);

    PassThrough<PointXYZRGB> pass2;
    pass2.setInputCloud(tmp_ptr2);
    pass2.setFilterFieldName("x");
    pass2.setFilterLimits(-2.0, 1.0);
    pass2.filter(*cloud_filtered);
}

void create_normals (PointCloud<PointXYZRGB>::Ptr cloud, 
        PointCloud<Normal>::Ptr normals, float normal_radius=0.03)
{
    NormalEstimation<PointXYZRGB, Normal> nest;

    nest.setInputCloud(cloud);
    nest.setSearchMethod (search::KdTree<PointXYZRGB>::Ptr (new search::KdTree<PointXYZRGB>));
    nest.setRadiusSearch(normal_radius);
    nest.compute(*normals);
}

void detect_keypoints (PointCloud<PointXYZRGB>::Ptr cloud, 
        PointCloud<PointWithScale>::Ptr keypoints, float min_scale=0.01,
        int nr_octaves=3, int nr_octaves_per_scale=3, float min_contrast=10.0)
{
    SIFTKeypoint<PointXYZRGB, PointWithScale> sift_detect;

    sift_detect.setInputCloud(cloud);
    sift_detect.setSearchMethod(search::KdTree<PointXYZRGB>::Ptr(new search::KdTree<PointXYZRGB>));
    sift_detect.setScales(min_scale, nr_octaves, nr_octaves_per_scale),
    sift_detect.setMinimumContrast(min_contrast);
    sift_detect.compute(*keypoints);
}

void compute_PFH_features_at_keypoints(PointCloud<PointXYZRGB>::Ptr &points,
        PointCloud<Normal>::Ptr &normals,
        PointCloud<PFHSignature125>::Ptr &descriptors,
        vector<int> &indices,
        float feature_radius=0.08)
{
    PFHEstimation<PointXYZRGB, Normal, PFHSignature125> pfh_est;

    pfh_est.setSearchMethod(search::KdTree<PointXYZRGB>::Ptr(new search::KdTree<PointXYZRGB>));
    pfh_est.setRadiusSearch(feature_radius);
    pfh_est.setInputCloud(points);
    pfh_est.setInputNormals(normals);

    pfh_est.compute(*descriptors);
}

void find_feature_correspondance(
        PointCloud<PFHSignature125>::Ptr &source_descriptors,
        PointCloud<PFHSignature125>::Ptr &target_descriptors,
        vector<int> &correspondance, vector<float> correspondance_scores)
{
    correspondance.resize(source_descriptors->size());
    correspondance_scores.resize(source_descriptors->size());

    pcl::KdTreeFLANN<pcl::PFHSignature125> descriptor_kdtree;
    descriptor_kdtree.setInputCloud(target_descriptors);

    const int k = 1;
    vector<int> k_indices(k);
    vector<float> k_squared_distances(k);

    for (size_t i = 0; i  < source_descriptors->size(); i++)
    {
        descriptor_kdtree.nearestKSearch(*source_descriptors, i, k, k_indices,
                k_squared_distances);
        correspondance[i] = k_indices[0];
        correspondance_scores[i] = k_squared_distances[0];
    }
}


PFHTransformStrategy::PFHTransformStrategy() : TransformStrategy() {}

PointCloud<PointXYZRGB>::Ptr PFHTransformStrategy::transform(
        const PointCloud<PointXYZRGB>::Ptr source, 
        const PointCloud<PointXYZRGB>::Ptr target)
{
    PointCloud<PointXYZRGB>::Ptr source_points(source);
    PointCloud<PointXYZRGB>::Ptr source_filtered(new PointCloud<PointXYZRGB>);
    PointCloud<Normal>::Ptr source_normals(new PointCloud<Normal>);
    PointCloud<PointWithScale>::Ptr source_keypoints(new PointCloud<PointWithScale>);
    PointCloud<PFHSignature125>::Ptr source_descriptors(new PointCloud<PFHSignature125>);

    PointCloud<PointXYZRGB>::Ptr target_points(target);
    PointCloud<PointXYZRGB>::Ptr target_filtered(new PointCloud<PointXYZRGB>);
    PointCloud<Normal>::Ptr target_normals(new PointCloud<Normal>);
    PointCloud<PointWithScale>::Ptr target_keypoints(new PointCloud<PointWithScale>);
    PointCloud<PFHSignature125>::Ptr target_descriptors(new PointCloud<PFHSignature125>);

    filter(source_points, source_filtered);
    filter(target_points, target_filtered);

    create_normals(source_filtered, source_normals);
    create_normals(target_filtered, target_normals);

    detect_keypoints(source_filtered, source_keypoints);
    detect_keypoints(target_filtered, target_keypoints);

    vector<int> source_indices(source_keypoints->points.size());
    vector<int> target_indices(target_keypoints->points.size());

    compute_PFH_features_at_keypoints(source_filtered, source_normals,
            source_descriptors, target_indices);
    compute_PFH_features_at_keypoints(target_filtered, target_normals,
            target_descriptors, target_indices);

    vector<int> correspondances;
    vector<float> correspondance_scores;
    find_feature_correspondance(source_descriptors, target_descriptors,
            correspondances, correspondance_scores);

    std::cout << "First cloud: Found " << source_keypoints->size()
        << " keypoints out of " << source_filtered->size() << " total points."
        << std::endl;
    std::cout << "Second cloud: Found " << target_keypoints->size()
        << " keypoints out of " << target_filtered->size() << " total points."
        << std::endl;

    return PointCloud<PointXYZRGB>::Ptr (source_points);
}
