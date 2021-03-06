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
    PFHTransformStrategy() {};

    typename PointCloud<PointT>::Ptr transform(
            const typename PointCloud<PointT>::Ptr source,
            const typename PointCloud<PointT>::Ptr target,boost::shared_ptr<Configuration> configuration)
    {
        source_cloud = *source;
        target_cloud = *target;

        cout << "[PFHTransformStrategy::transform] Setting source cloud: "
            << source->size() << " points" << endl;
        cout << "[PFHTransformStrategy::transform] Setting target cloud: "
            << target->size() << " points" << endl;

        typename PointCloud<PointT>::Ptr transformed(new PointCloud<PointT>);

        typename PointCloud<PointXYZRGB>::Ptr source_points(source);
        typename PointCloud<PointXYZRGB>::Ptr source_filtered(
                new PointCloud<PointT>);
        PointCloud<Normal>::Ptr source_normals(new PointCloud<Normal>);
        PointCloud<PointWithScale>::Ptr source_keypoints(
                new PointCloud<PointWithScale>);
        PointCloud<PFHSignature125>::Ptr source_descriptors(
                new PointCloud<PFHSignature125>);

        typename PointCloud<PointT>::Ptr target_points(target);
        typename PointCloud<PointT>::Ptr target_filtered(
                new PointCloud<PointT>);
        PointCloud<Normal>::Ptr target_normals(
                new PointCloud<Normal>);
        PointCloud<PointWithScale>::Ptr target_keypoints(
                new PointCloud<PointWithScale>);
        PointCloud<PFHSignature125>::Ptr target_descriptors(
                new PointCloud<PFHSignature125>);

        cout << "[PFHTransformStrategy::transform] Downsampling source and "
            << "target clouds..." << endl;

        //filter(source_points, source_filtered, 0.01f);
        //filter(target_points, target_filtered, 0.01f);
        source_filtered=source_points;
        target_filtered=target_points;
        cout << "[PFHTransformStrategy::transform] Creating normals for "
            << "source and target cloud..." << endl;

        create_normals<Normal>(source_filtered, source_normals);
        create_normals<Normal>(target_filtered, target_normals);

        cout << "[PFHTransformStrategy::transform] Finding keypoints in "
            << "source and target cloud..." << endl;

        detect_keypoints(source_filtered, source_keypoints);
        detect_keypoints(target_filtered, target_keypoints);

        for(PointWithScale p: source_keypoints->points){
        	cout<<"keypoint "<<p;
        }

        vector<int> source_indices(source_keypoints->points.size());
        vector<int> target_indices(target_keypoints->points.size());

        cout << "[PFHTransformStrategy::transform] Computing PFH features "
            << "for source and target cloud..." << endl;

        compute_PFH_features_at_keypoints(source_filtered, source_normals,source_keypoints,
                source_descriptors, target_indices);
        compute_PFH_features_at_keypoints(target_filtered, target_normals,target_keypoints,
                target_descriptors, target_indices);

        vector<int> correspondences;
        vector<float> correspondence_scores;

        find_feature_correspondence(source_descriptors, target_descriptors,
                correspondences, correspondence_scores);

        cout << "correspondences: " << correspondences.size() << endl;
        cout << "c. scores: " << correspondence_scores.size() << endl;

        cout << "First cloud: Found " << source_keypoints->size() << " keypoints "
            << "out of " << source_filtered->size() << " total points." << endl;
        cout << "Second cloud: Found " << target_keypoints->size() << " keypoints"
            << " out of " << target_filtered->size() << " total points." << endl;

        // Start with the actual transformation. Yeay :)
        TransformationFromCorrespondences tfc;
        tfc.reset();

        vector<int> sorted_scores;
        cv::sortIdx(correspondence_scores, sorted_scores, CV_SORT_EVERY_ROW);

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
               // cout<< "abs position difference:"<<abs(source_position[1] - target_position[1])<<"C-Score"<<correspondence_scores[index]<<endl;
               // cout<<" Source Position " <<source_position<<endl;
               // cout<<" target position "<<target_position<<endl;
                tfc.add(source_position, target_position,
                        correspondence_scores[index]);
                fidx.push_back(source_indices[index]);
                fidxt.push_back(target_indices[correspondences[index]]);
            }
        }
        cout << "TFC samples: "<<tfc.getNoOfSamples()<<" AccumulatedWeight "<<tfc.getAccumulatedWeight()<<endl;
        Eigen::Affine3f tr;
        tr = tfc.getTransformation();
        cout << "TFC transformation: " << endl;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cout << tr.rotation()(i, i) << "\t";
            }
            cout << endl;
        }

        transformPointCloud(*source_filtered, *transformed,
                tfc.getTransformation());
        cout << "transformation finished";
        return transformed;
    };

    void loadDefaultPresets()
    {
        cout << "[PFH Transformation] loading default presets..." << endl;
    };

private:

    PointCloud<PointT> source_cloud;
    PointCloud<PointT> target_cloud;

    // Downsample a point cloud by using a voxel grid.
    // See http://pointclouds.org/documentation/tutorials/voxel_grid.php
    void filter (typename PointCloud<PointT>::Ptr cloud,
            typename PointCloud<PointT>::Ptr cloud_filtered,
            float leaf_size=0.01f)
    {
        cout << "[PFHTransformationStrategy::filter] Input cloud:"
            << endl << "    " << cloud->points.size() << " points" << endl;

        typename PointCloud<PointT>::Ptr tmp_ptr1(new PointCloud<PointT>);

        VoxelGrid<PointT> vox_grid;
        vox_grid.setInputCloud(cloud);
        vox_grid.setSaveLeafLayout(true);
        vox_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

        cout << "[PFHTransformationStrategy::filter] Creating a voxel grid:"
            << endl << "    leaf size: [" << leaf_size << ", "
            << leaf_size << ", " << leaf_size << "]" << endl;

        // Skip the rest...
        // vox_grid.filter(*tmp_ptr1);
        vox_grid.filter(*cloud_filtered);

        cout << "[PFHTransformationStrategy::filter] Result of voxel grid"
            << " filtering:" << endl << "    " << cloud_filtered->points.size()
            << " points remaining" << endl;

        return;

        typename PointCloud<PointT>::Ptr tmp_ptr2(new PointCloud<PointT>);

        float pass1_limit_min = 0.0;
        float pass1_limit_max = 3.0;

        PassThrough<PointT> pass1;
        pass1.setInputCloud(tmp_ptr1);
        pass1.setFilterFieldName("z");
        pass1.setFilterLimits(pass1_limit_min, pass1_limit_max);

        cout << "[PFH Transformation : Downsampling] starting first filter"
            << " pass through:" << endl;
        cout << "  filter field name: " << pass1.getFilterFieldName() << endl;
        cout << "  filter limits: min = " << pass1_limit_min << ", max = "
            << pass1_limit_max << endl;

        float avg;
        for (PointT point : *tmp_ptr1) {
            avg += point.z;
        }

        cout << "  average field value: " << avg / tmp_ptr1->size() << endl;

        pass1.filter(*tmp_ptr2);

        cout << "[PFH Transformation : Downsampling] result of first pass:"
            << endl << "  " << tmp_ptr2->points.size() << " points remaining."
            << endl;

        float pass2_limit_min = -2.0;
        float pass2_limit_max = 1.0;

        PassThrough<PointT> pass2;
        pass2.setInputCloud(tmp_ptr2);
        pass2.setFilterFieldName("x");
        pass2.setFilterLimits(pass2_limit_min, pass2_limit_max);

        cout << "[PFH Transformation : Downsampling] starting second filter"
            << " pass through:" << endl;
        cout << "  filter field name: " << pass2.getFilterFieldName() << endl;
        cout << "  filter limits: min = " << pass2_limit_min << ", max = "
            << pass2_limit_max << endl;

        pass2.filter(*cloud_filtered);

        cout << "[PFH Transformation : Downsampling] result of second pass:"
            << endl << "  " << cloud_filtered->points.size()
            << " points remaining" << endl;
        cout << "[PFH Transformation : Downsampling] size of output cloud:"
            << endl << "  " << cloud_filtered->points.size() << " points"
            << endl << endl;
    };

    // Create/Estimate the surface normals.
    // See http://pointclouds.org/documentation/tutorials/normal_estimation.php
    template<typename NormalT>
    void create_normals (typename PointCloud<PointT>::Ptr cloud,
            typename PointCloud<NormalT>::Ptr normals,
            float normal_radius=0.03)
    {
        NormalEstimation<PointT, NormalT> nest;

        cout << "[PFHTransformationStrategy::create_normals] Input cloud "
            << cloud->points.size() << " points" << endl;

        nest.setInputCloud(cloud);
        nest.setSearchMethod (typename search::KdTree<PointT>::Ptr
                 (new search::KdTree<PointT>));
        nest.setRadiusSearch(normal_radius);
        nest.compute(*normals);
    };

    void detect_keypoints (typename PointCloud<PointT>::Ptr cloud,
            PointCloud<PointWithScale>::Ptr keypoints, float min_scale=0.01,
            int nr_octaves=3, int nr_octaves_per_scale=3,
            float min_contrast=5.0)
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
            pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints,
            PointCloud<PFHSignature125>::Ptr &descriptors,
            vector<int> &indices,
            float feature_radius=0.08)
    {
        PFHEstimation<PointXYZRGB, Normal, PFHSignature125> pfh_est;

        pfh_est.setSearchMethod(search::KdTree<PointXYZRGB>::Ptr(
                    new search::KdTree<PointXYZRGB>));
        pfh_est.setRadiusSearch(feature_radius);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);

        pfh_est.setSearchSurface (points);

        pfh_est.setInputCloud(keypoints_xyzrgb);
        pfh_est.setInputNormals(normals);

        cout << "[PFH Transformation : PFH Estimation]" << endl;
        cout << "  setting input cloud: " << points->size() << " points"
            << endl;
        cout << "  setting input normals: " << normals->size() << " normals"
            << endl;

        for (int i = 0; i < normals->points.size(); i++)
        {
          if (!pcl::isFinite<pcl::Normal>(normals->points[i]))
          {
            cerr<<"normal "<< i<<" is not finite\n";
           // normals->points[i]=new pcl::Normal();
          }
        }

        pfh_est.compute(*descriptors);
    };

    void find_feature_correspondence(
            PointCloud<PFHSignature125>::Ptr &source_descriptors,
            PointCloud<PFHSignature125>::Ptr &target_descriptors,
            vector<int> &correspondence, vector<float> &correspondence_scores)
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

            cout << "step " << i << ": correspondence=" << correspondence[i] << endl;
            cout << "   correspondence_score=" << correspondence_scores[i] << endl;
        }
    };
};

#endif
