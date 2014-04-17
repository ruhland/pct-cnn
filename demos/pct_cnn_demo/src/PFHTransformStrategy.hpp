#ifndef PFHTRANSFORMSTRATEGY_H
#define PFHTRANSFORMSTRATEGY_H

#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
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

#include <opencv/cv.h>

#include "TransformStrategy.hpp"

template<typename PointT>
class PFHTransformStrategy : public TransformStrategy<PointT>
{
public:
    PFHTransformStrategy()
    {
        cout << "Point Feature Histogram" << endl;
    };

    typename PointCloud<PointT>::Ptr transform(
            const typename PointCloud<PointT>::Ptr source,
            const typename PointCloud<PointT>::Ptr target)
    {
        source_cloud = *source;
        target_cloud = *target;

        cout << "source = " << source << ", &source_cloud = " << &source_cloud << endl;

        typename PointCloud<PointT>::Ptr transformed(new PointCloud<PointT>);

        typename PointCloud<PointXYZRGB>::Ptr source_points(source);
        typename PointCloud<PointXYZRGB>::Ptr source_filtered(new PointCloud<PointT>);
        PointCloud<Normal>::Ptr source_normals(new PointCloud<Normal>);
        PointCloud<PointWithScale>::Ptr source_keypoints(new PointCloud<PointWithScale>);
        PointCloud<PFHSignature125>::Ptr source_descriptors(new PointCloud<PFHSignature125>);

        typename PointCloud<PointT>::Ptr target_points(target);
        typename PointCloud<PointT>::Ptr target_filtered(new PointCloud<PointT>);
        PointCloud<Normal>::Ptr target_normals(new PointCloud<Normal>);
        PointCloud<PointWithScale>::Ptr target_keypoints(new PointCloud<PointWithScale>);
        PointCloud<PFHSignature125>::Ptr target_descriptors(new PointCloud<PFHSignature125>);

        filter(source_points, source_filtered);
        filter(target_points, target_filtered);

        create_normals<Normal>(source_filtered, source_normals);
        create_normals<Normal>(target_filtered, target_normals);

        detect_keypoints(source_filtered, source_keypoints);
        detect_keypoints(target_filtered, target_keypoints);

        vector<int> source_indices(source_keypoints->points.size());
        vector<int> target_indices(target_keypoints->points.size());

        compute_PFH_features_at_keypoints(source_filtered, source_normals,
                source_descriptors, target_indices);
        compute_PFH_features_at_keypoints(target_filtered, target_normals,
                target_descriptors, target_indices);

        vector<int> correspondences;
        vector<float> correspondence_scores;
        find_feature_correspondence(source_descriptors, target_descriptors,
                correspondences, correspondence_scores);

        cout << "First cloud: Found " << source_keypoints->size() << " keypoints "
            << "out of " << source_filtered->size() << " total points." << endl;
        cout << "Second cloud: Found " << target_keypoints->size() << " keypoints"
            << " out of " << target_filtered->size() << " total points." << endl;

        // Start with the actual transformation. Yeay :)
        TransformationFromCorrespondences tfc;
        tfc.reset();

        vector<int> sorted_scores;
        cv::sortIdx(correspondence_scores, sorted_scores, 2);

        vector<float> tmp(correspondence_scores);
        sort(tmp.begin(), tmp.end());

        float median_score = tmp[tmp.size() / 2];
        vector<int> fidx;
        vector<int> fidxt;

        Eigen::Vector3f source_position(0, 0, 0);
        Eigen::Vector3f target_position(0, 0, 0);

        for (size_t i = 0; i < correspondence_scores.size(); i++) {
            int index = sorted_scores[i];
            if (median_score >= correspondence_scores[index]) {
                source_position[0] = source_keypoints->points[index].x;
                source_position[1] = source_keypoints->points[index].y;
                source_position[2] = source_keypoints->points[index].z;

                target_position[0] = target_keypoints->points[index].x;
                target_position[1] = target_keypoints->points[index].y;
                target_position[2] = target_keypoints->points[index].z;

                if (abs(source_position[1] - target_position[1]) > 0.2) {
                    continue;
                }

                tfc.add(source_position, target_position,
                        correspondence_scores[index]);
                fidx.push_back(source_indices[index]);
                fidxt.push_back(target_indices[correspondences[index]]);
            }
        }

        Eigen::Affine3f tr;
        tr = tfc.getTransformation();

        cout << "TFC transformation: " << endl;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                cout << tr(i, i) << "\t";
            }
            cout << endl;
        }

        transformPointCloud(*source_filtered, *transformed,
                tfc.getTransformation());

        return transformed;
    };

    void loadDefaultSettings()
    {
        cout << "Loading default settings for the Point Feature Historgram"
            << " Transformation:" << endl;
    };

private:

    PointCloud<PointT> source_cloud;
    PointCloud<PointT> target_cloud;

    // Downsample a point cloud by using a voxel grid.
    // See http://pointclouds.org/documentation/tutorials/voxel_grid.php
    void filter (typename PointCloud<PointT>::Ptr cloud,
            typename PointCloud<PointT>::Ptr cloud_filtered,
            float leaf_size=0.1f)
    {
        typename PointCloud<PointT>::Ptr tmp_ptr1(new PointCloud<PointT>);

        VoxelGrid<PointT> vox_grid;
        vox_grid.setInputCloud(cloud);
        vox_grid.setSaveLeafLayout(true);
        vox_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        vox_grid.filter(*tmp_ptr1);

        typename PointCloud<PointT>::Ptr tmp_ptr2(new PointCloud<PointT>);

        PassThrough<PointT> pass1;
        pass1.setInputCloud(tmp_ptr1);
        pass1.setFilterFieldName("z");
        pass1.setFilterLimits(0.0, 3.0);
        pass1.filter(*tmp_ptr2);

        PassThrough<PointT> pass2;
        pass2.setInputCloud(tmp_ptr2);
        pass2.setFilterFieldName("x");
        pass2.setFilterLimits(-2.0, 1.0);
        pass2.filter(*cloud_filtered);
    };

    // Create/Estimate the surface normals.
    // See http://pointclouds.org/documentation/tutorials/normal_estimation.php
    template<typename NormalT>
    void create_normals (typename PointCloud<PointT>::Ptr cloud,
            typename PointCloud<NormalT>::Ptr normals,
            float normal_radius=0.03)
    {
        NormalEstimation<PointT, NormalT> nest;

        nest.setInputCloud(cloud);
        nest.setSearchMethod (typename search::KdTree<PointT>::Ptr
                 (new search::KdTree<PointT>));
        nest.setRadiusSearch(normal_radius);
        nest.compute(*normals);
    };

    void detect_keypoints (typename PointCloud<PointT>::Ptr cloud,
            PointCloud<PointWithScale>::Ptr keypoints, float min_scale=0.01,
            int nr_octaves=3, int nr_octaves_per_scale=3,
            float min_contrast=10.0)
    {
        SIFTKeypoint<PointT, PointWithScale> sift_detect;

        sift_detect.setInputCloud(cloud);
        sift_detect.setSearchMethod(typename search::KdTree<PointT>::Ptr(
                    new search::KdTree<PointT>));
        sift_detect.setScales(min_scale, nr_octaves, nr_octaves_per_scale),
        sift_detect.setMinimumContrast(min_contrast);
        sift_detect.compute(*keypoints);
    };

    void compute_PFH_features_at_keypoints(PointCloud<PointXYZRGB>::Ptr &points,
            PointCloud<Normal>::Ptr &normals,
            PointCloud<PFHSignature125>::Ptr &descriptors,
            vector<int> &indices,
            float feature_radius=0.08)
    {
        PFHEstimation<PointXYZRGB, Normal, PFHSignature125> pfh_est;

        pfh_est.setSearchMethod(search::KdTree<PointXYZRGB>::Ptr(
                    new search::KdTree<PointXYZRGB>));
        pfh_est.setRadiusSearch(feature_radius);
        pfh_est.setInputCloud(points);
        pfh_est.setInputNormals(normals);

        pfh_est.compute(*descriptors);
    };

    void find_feature_correspondence(
            PointCloud<PFHSignature125>::Ptr &source_descriptors,
            PointCloud<PFHSignature125>::Ptr &target_descriptors,
            vector<int> &correspondence, vector<float> correspondence_scores)
    {
        correspondence.resize(source_descriptors->size());
        correspondence_scores.resize(source_descriptors->size());

        pcl::KdTreeFLANN<pcl::PFHSignature125> descriptor_kdtree;
        descriptor_kdtree.setInputCloud(target_descriptors);

        const int k = 1;
        vector<int> k_indices(k);
        vector<float> k_squared_distances(k);

        for (size_t i = 0; i  < source_descriptors->size(); i++)
        {
            descriptor_kdtree.nearestKSearch(*source_descriptors, i, k,
                    k_indices, k_squared_distances);
            correspondence[i] = k_indices[0];
            correspondence_scores[i] = k_squared_distances[0];
        }
    };
};

#endif
