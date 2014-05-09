#ifndef SNNTransformStrategy_HPP
#define SNNTransformStrategy_HPP

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
class CNNTransformStrategy : public TransformStrategy<PointT>
{

  //TODO: Move somewhere accessible from all transformation and NO CTRL_C
  // Downsample a point cloud by using a voxel grid.
  // See http://pointclouds.org/documentation/tutorials/voxel_grid.php
  private:

    class PointWithScore
    {
      public:
        int index;
        float score;

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
    };

    /** \brief Calculate normals for every point in the given cloud.
      * \param[in] cloud the input cloud
      * \param[in] normal_radius the parameter for the radius search
      * \param[out] normals the output cloud with surface normals
      */
    template<typename NormalT>
    void
    createNormals(typename PointCloud<PointT>::Ptr cloud,
                  typename PointCloud<NormalT>::Ptr normals,
                  float normal_radius = 0.1)
    {
      NormalEstimation<PointT, NormalT> nest;

      cout << "[PFHTransformationStrategy::createNormals] Input cloud "
        << cloud->points.size() << " points" << endl;

      nest.setInputCloud(cloud);
      nest.setSearchMethod(typename search::KdTree<PointT>::Ptr(new search::KdTree<PointT>));
      nest.setRadiusSearch(normal_radius);
      nest.compute(*normals);
    };

    void
    filter(typename PointCloud<PointT>::Ptr cloud,
           typename PointCloud<PointT>::Ptr cloud_filtered,
           float leaf_size = 0.01f)
    {
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
    };

    void
    computePFHFeatures(PointCloud<PointXYZRGB>::Ptr &points,
                       PointCloud<Normal>::Ptr &normals,
                       PointCloud<PFHSignature125>::Ptr &descriptors,
                       float feature_radius = 0.08)
    {
      PFHEstimation<PointXYZRGB, Normal, PFHSignature125> pfh_est;

      pfh_est.setSearchMethod(search::KdTree<PointXYZRGB>::Ptr(new search::KdTree<PointXYZRGB>));
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
    };

    /**
     * Find the the k nearest neighbors in the target cloud for every 
     * feature point in the source cloud.
     * The k nearest neighbors for a point with index i are stored
     * in coherent_neighbors[i] with their score. The score/rank of every
     * neighbor is determined by the squared distance to the corresponding
     * query point.
     */
    template<typename FeatureT>
    void
    findCoherentNeighbours(typename PointCloud<FeatureT>::Ptr source_features,
                           typename PointCloud<FeatureT>::Ptr target_features,
                           vector<vector<PointWithScore>> &coherent_neighbours,
                           int k = 5)
    {
      coherent_neighbours.resize(source_features->size());

      pcl::KdTreeFLANN<FeatureT> searchtree;
      searchtree.setInputCloud(target_features);

      vector<int> k_indices(k);
      vector<float> k_squared_distances(k);

      for (size_t i = 0; i < source_features->size(); i++) {

        std::cout << (*source_features)[i];
        std::cout << std::endl;

        searchtree.nearestKSearch(*source_features, i, k, k_indices, k_squared_distances);
        for (int y = 0; y < k; y++) {
          PointWithScore a(k_indices[y], k_squared_distances[y]);
          coherent_neighbours[i].push_back(a);
        }
      }
    }

    /**
     * Find the perfect neighbor for every point in the source features cloud.
     * This is possible, because the point indices of our demo clouds can be 
     * inverted.
     */
    void
    findPerfectCoherentNeighbours(PointCloud<PointXYZRGB>::Ptr source_features,
                                  PointCloud<PointXYZRGB>::Ptr target_features,
                                  vector<vector<PointWithScore>> &coherent_neighbours,int k=5)
    {
      coherent_neighbours.resize(source_features->size());
      for (size_t i = 0; i < source_features->size(); i++) {
        for (int y = 0; y < k; y++) {
          PointWithScore a(i, 0);
          coherent_neighbours[i].push_back(a);
        }
      }
    }

    void replaceColors(PointCloud<PointXYZRGB>::Ptr &src,
        PointCloud<PointXYZRGB>::Ptr &target,
        vector<vector<PointWithScore>> &coherentNeighbours)
    {
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
        vector<vector<PointWithScore>> &coherentNeighboursTarget,int k)
    {
      for (int i = 0; i < coherentNeighboursSource.size(); i++) {
        std::sort(coherentNeighboursSource[i].begin(), coherentNeighboursSource[i].end());
        //std::cout<<std::endl<<"Refine Score for point "<<i<<std::endl;
        for (int s = 0; s < coherentNeighboursSource[i].size(); s++) {
          int targetIndex = coherentNeighboursSource[i][s].index;
          std::sort(coherentNeighboursTarget[targetIndex].begin(), coherentNeighboursTarget[targetIndex].end());
          //std:: cout<<" Target "<< targetIndex<<":";
          //std::cout<<"Targetsize"<<coherentNeighboursTarget.size()<<","<<coherentNeighboursTarget[targetIndex].size();
          for (int t = 0; t < coherentNeighboursTarget[targetIndex].size(); t++) {
            //std::cout<<t<<", ";
            if (coherentNeighboursTarget[targetIndex][t].index == i) {
              std::cout<<" Found corresponding forward backward search points source: "<<i<<" target:"<<targetIndex <<" Score before:"<<coherentNeighboursSource[i][s].score;
              coherentNeighboursSource[i][s].score+=coherentNeighboursTarget[targetIndex][t].score+(k+k-s-t);
              std::cout<<" after "<<coherentNeighboursSource[i][s].score<<std::endl;
            }
          }
        }
      }
    }

  public:
    typename PointCloud<PointT>::Ptr transform(
        const typename PointCloud<PointT>::Ptr source,
        const typename PointCloud<PointT>::Ptr target,
        boost::shared_ptr<Configuration> configuration)
    {
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

      if(configuration->getFilterMethod()==VOXELGRIDFILTER){
    	  filter(source, sourceFiltered);
    	  filter(target, targetFiltered);
      }
      else{
    	  sourceFiltered=source;
      	  targetFiltered=target;
      }

      //createNormals<Normal>(sourceFiltered, source_normals);
      //createNormals<Normal>(targetFiltered, target_normals);
      /*computePFHFeatures(sourceFiltered, source_normals,
        source_descriptors);
        computePFHFeatures(targetFiltered, target_normals,
        target_descriptors);*/
      vector<vector<PointWithScore>> coherentNeighboursSource;
      vector<vector<PointWithScore>> coherentNeighboursTarget;

      int k = configuration->getNearestNeighborsToSearch();

      findCoherentNeighbours<PointT>(sourceFiltered, targetFiltered,
        coherentNeighboursSource, k);
      findCoherentNeighbours<PointT>(targetFiltered, sourceFiltered,
        coherentNeighboursTarget, k);
/*      findPerfectCoherentNeighbours(sourceFiltered, targetFiltered,
          coherentNeighboursSource);
      findPerfectCoherentNeighbours(sourceFiltered, targetFiltered,
          coherentNeighboursTarget); */
      refineScores(coherentNeighboursSource,coherentNeighboursTarget,k);
      replaceColors(sourceFiltered, targetFiltered, coherentNeighboursSource);

      return sourceFiltered;
    }

    void loadDefaultPresets() {
      std::cout << "Default presets loaded..." << std::endl;
    }
};

#endif
