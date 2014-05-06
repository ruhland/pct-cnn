#ifndef CNNTransformStrategy_HPP
#define CNNTransformStrategy_HPP
#include "TransformStrategy.hpp"
#include <queue>
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
#include <algorithm>
#include <opencv/cv.h>

template<typename PointT>
class CNNTransformStrategy: public TransformStrategy<PointT> {

	//TODO: Move somewhere accessible from all transformation and NO CTRL_C
	// Downsample a point cloud by using a voxel grid.
	// See http://pointclouds.org/documentation/tutorials/voxel_grid.php
private:
	static const int NEARESTNEIGHBORSTOSEARCH = 5;

	class PointWithScore {
	public:
		PointWithScore() {
			index = 0;
			score = 0;
		}
		PointWithScore(int indexpoint, float scorepoint) {
			index = indexpoint;
			score = scorepoint;
		}
		bool operator<(const PointWithScore &rp) const {
			return score < rp.score;
		}
		int index;
		float score;
	};

	// Create/Estimate the surface normals.
	// See http://pointclouds.org/documentation/tutorials/normal_estimation.php
	template<typename NormalT>
	void create_normals(typename PointCloud<PointT>::Ptr cloud,
			typename PointCloud<NormalT>::Ptr normals,
			float normal_radius = 0.1) {
		NormalEstimation<PointT, NormalT> nest;

		cout << "[PFHTransformationStrategy::create_normals] Input cloud "
				<< cloud->points.size() << " points" << endl;

		nest.setInputCloud(cloud);
		nest.setSearchMethod(
				typename search::KdTree<PointT>::Ptr(
						new search::KdTree<PointT>));
		nest.setRadiusSearch(normal_radius);
		nest.compute(*normals);
	}
	;

	void filter(typename PointCloud<PointT>::Ptr cloud,
			typename PointCloud<PointT>::Ptr cloud_filtered, float leaf_size =
					0.01f) {
		cout << "[PFHTransformationStrategy::filter] Input cloud:" << endl
				<< "    " << cloud->points.size() << " points" << endl;

		typename PointCloud<PointT>::Ptr tmp_ptr1(new PointCloud<PointT>);

		VoxelGrid<PointT> vox_grid;
		vox_grid.setInputCloud(cloud);
		vox_grid.setSaveLeafLayout(true);
		vox_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

		cout << "[PFHTransformationStrategy::filter] Creating a voxel grid:"
				<< endl << "    leaf size: [" << leaf_size << ", " << leaf_size
				<< ", " << leaf_size << "]" << endl;

		// Skip the rest...
		// vox_grid.filter(*tmp_ptr1);
		vox_grid.filter(*cloud_filtered);

		cout << "[PFHTransformationStrategy::filter] Result of voxel grid"
				<< " filtering:" << endl << "    "
				<< cloud_filtered->points.size() << " points remaining" << endl;

		return;
	}
	;

	void compute_PFH_features(PointCloud<PointXYZRGB>::Ptr &points,
			PointCloud<Normal>::Ptr &normals,
			PointCloud<PFHSignature125>::Ptr &descriptors,
			float feature_radius = 0.08) {
		PFHEstimation<PointXYZRGB, Normal, PFHSignature125> pfh_est;

		pfh_est.setSearchMethod(
				search::KdTree<PointXYZRGB>::Ptr(
						new search::KdTree<PointXYZRGB>));
		pfh_est.setRadiusSearch(feature_radius);

		pfh_est.setInputCloud(points);
		pfh_est.setInputNormals(normals);

		cout << "[PFH Transformation : PFH Estimation]" << endl;
		cout << "  setting input cloud: " << points->size() << " points"
				<< endl;
		cout << "  setting input normals: " << normals->size() << " normals"
				<< endl;

		for (int i = 0; i < normals->points.size(); i++) {
			if (!pcl::isFinite<pcl::Normal>(normals->points[i])) {
				cerr << "normal " << i << " is not finite\n";
				// normals->points[i]=new pcl::Normal();
			}
		}

		pfh_est.compute(*descriptors);
	}
	;

	void findCoherentNeighbours(PointCloud<PointXYZRGB>::Ptr sourceFeatures,
			PointCloud<PointXYZRGB>::Ptr targetFeatures,
			vector<vector<PointWithScore>> &coherentNeighbours) {
		coherentNeighbours.resize(sourceFeatures->size());

		pcl::KdTreeFLANN<pcl::PointXYZRGB> descriptor_kdtree;
		descriptor_kdtree.setInputCloud(targetFeatures);

		vector<int> k_indices(NEARESTNEIGHBORSTOSEARCH);
		vector<float> k_squared_distances(NEARESTNEIGHBORSTOSEARCH);

		for (size_t i = 0; i < sourceFeatures->size(); i++) {

			std::cout << (*sourceFeatures)[i];
			std::cout << std::endl;

			descriptor_kdtree.nearestKSearch(*sourceFeatures, i,
					NEARESTNEIGHBORSTOSEARCH, k_indices, k_squared_distances);
			for (int y = 0; y < NEARESTNEIGHBORSTOSEARCH; y++) {
				PointWithScore a(k_indices[y], k_squared_distances[y]);
				coherentNeighbours[i].push_back(a);
				/*std::cout << "Point " << i << " add coherent neighbor "
				 << k_indices[y] << " with distance "
				 << k_squared_distances[y] << std::endl;*/
			}
		}
	}

	void replaceColors(PointCloud<PointXYZRGB>::Ptr &src,
			PointCloud<PointXYZRGB>::Ptr &target,
			vector<vector<PointWithScore>> &coherentNeighbours) {
		for (int i = 0; i < src->size(); i++) {
			pcl::PointXYZRGB& ps = src->points[i];
			if (i > coherentNeighbours.size()
					|| coherentNeighbours[i].size() == 0) {
				std::cout << "Error Queue for point " << i
						<< " dos not exist \n";
			}
			std::sort(coherentNeighbours[i].begin(),coherentNeighbours[i].end());
			int targetpoint = coherentNeighbours[i].front().index;
			if (targetpoint < 0 || targetpoint >= target->size()) {
				std::cout << " ERROR targetpoint " << targetpoint
						<< " out of range" << std::endl;
				targetpoint = 1;
			}
			pcl::PointXYZRGB& pt = target->points[targetpoint];

			ps.rgba = pt.rgba;
		}
	}

	void refineScores(
			vector<vector<PointWithScore>> &coherentNeighboursSource,
			vector<vector<PointWithScore>> &coherentNeighboursTarget) {
		for(int i=0;i<coherentNeighboursSource.size();i++){
			std::sort(coherentNeighboursSource[i].begin(),coherentNeighboursSource[i].end());
			std::cout<<std::endl<<"Refine Score for point "<<i<<std::endl;
			for(int s=0;s<coherentNeighboursSource[i].size();s++){
				int targetIndex=coherentNeighboursSource[i][s].index;
				std::sort(coherentNeighboursTarget[targetIndex].begin(),coherentNeighboursTarget[targetIndex].end());
				std:: cout<<" Target "<< targetIndex<<":";
				//std::cout<<"Targetsize"<<coherentNeighboursTarget.size()<<","<<coherentNeighboursTarget[targetIndex].size();
				for(int t=0;t<coherentNeighboursTarget[targetIndex].size();t++){
					std::cout<<t<<", ";
					if(coherentNeighboursTarget[targetIndex][t].index==i){
						std::cout<<" Found corresponding forward backward search points\n";
						coherentNeighboursSource[i][s].score+=coherentNeighboursTarget[targetIndex][t].score+(NEARESTNEIGHBORSTOSEARCH+NEARESTNEIGHBORSTOSEARCH-s-t);
					}
				}
			}
		}
	}

public:
	typename PointCloud<PointT>::Ptr transform(
			const typename PointCloud<PointT>::Ptr source,
			const typename PointCloud<PointT>::Ptr target) {
		typename PointCloud<PointXYZRGB>::Ptr sourceFiltered(
				new PointCloud<PointXYZRGB>);
		typename PointCloud<PointXYZRGB>::Ptr targetFiltered(
				new PointCloud<PointXYZRGB>);
		PointCloud<Normal>::Ptr source_normals(new PointCloud<Normal>);
		PointCloud<Normal>::Ptr target_normals(new PointCloud<Normal>);
		PointCloud<PFHSignature125>::Ptr source_descriptors(
				new PointCloud<PFHSignature125>);
		PointCloud<PFHSignature125>::Ptr target_descriptors(
				new PointCloud<PFHSignature125>);

		PointWithScore a(5, 5);
		PointWithScore b(2, 2);
		std::cout << "a<b" << (a < b);

		filter(source, sourceFiltered);
		filter(target, targetFiltered);
		create_normals<Normal>(sourceFiltered, source_normals);
		create_normals<Normal>(targetFiltered, target_normals);
		/*compute_PFH_features(sourceFiltered, source_normals,
		 source_descriptors);
		 compute_PFH_features(targetFiltered, target_normals,
		 target_descriptors);*/
		vector<vector<PointWithScore>> coherentNeighboursSource;
		vector<vector<PointWithScore>> coherentNeighboursTarget;
		findCoherentNeighbours(sourceFiltered, targetFiltered,
				coherentNeighboursSource);
		findCoherentNeighbours(targetFiltered, sourceFiltered,
				coherentNeighboursTarget);
		refineScores(coherentNeighboursSource,coherentNeighboursTarget);
		replaceColors(sourceFiltered, targetFiltered, coherentNeighboursSource);

		return sourceFiltered;
	}

	void loadDefaultPresets() {
		std::cout << "Default presets loaded..." << std::endl;
	}
};

#endif
