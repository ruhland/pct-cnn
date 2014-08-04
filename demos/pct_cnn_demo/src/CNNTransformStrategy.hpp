#ifndef SNNTransformStrategy_HPP
#define SNNTransformStrategy_HPP

#define PCL_NO_PRECOMPILE

#include "TransformStrategy.hpp"

// STL
#include <queue>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

// Boost
#include <boost/timer.hpp>

// OpenCV
#include <opencv/cv.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/intensity.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/impl/intensity_gradient.hpp>
#include <pcl/features/rift.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace pcl {
  typedef union {
    struct {
      unsigned char Blue;
      unsigned char Green;
      unsigned char Red;
      unsigned char Alpha;
    };
    float float_value;
    uint32_t long_value;
  } RGBValue;
}

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
     *  \param[in] cloud the input cloud
     *  \param[in] normal_radius the parameter for the radius search
     *  \param[out] normals the output cloud with surface normals
     */
    template<typename NormalT>
      void createNormals(typename PointCloud<PointT>::Ptr cloud,
          typename PointCloud<NormalT>::Ptr normals,
          float normal_radius = 0.1)
      {
        NormalEstimation<PointT, NormalT> nest;

        cout << "[CNNTransformationStrategy::createNormals] Input cloud "
          << cloud->points.size() << " points" << endl;

        cout << "[CNNTransformationStrategy::createNormals] Setting search "
          << "radius: " << normal_radius << endl;

        nest.setInputCloud(cloud);
        nest.setSearchMethod(typename search::KdTree<PointT>::Ptr(new search::KdTree<PointT>));
        nest.setRadiusSearch(normal_radius);
        nest.compute(*normals);

        cout << "[CNNTransformationStrategy::createNormals] Found "
          << normals->size()  << " normals" << endl;

        for (int i = 0; i < normals->size(); i++) {
          if (!pcl::isFinite<pcl::Normal>((*normals)[i])) {
            cerr << "[CNNTransformStrategy::createNormals]"
              << " Normal " << i << " " << (*normals)[i] << " is not finite"
              << endl;
          }
        }
      };

    /** \brief Compute intensity gradients (local changes of color) for the 
     * given input cloud.
     * \param [in] points the input point cloud
     * \param [in] normals the input normals
     * \param [out] gradients the output intensity gradients
     * \param search_radius the radius for the radius search
     */
    void createGradients(PointCloud<PointXYZRGB>::Ptr points,
        PointCloud<Normal>::Ptr normals,
        PointCloud<IntensityGradient>::Ptr gradients,
        double search_radius = 0.1)
    {
      IntensityGradientEstimation<PointXYZRGB, Normal, IntensityGradient> gradient_est;

      gradient_est.setInputCloud(points);
      gradient_est.setInputNormals(normals);

      cout << "[CNNTransformStrategy::createGradients] "
        << "Starting intesity gradient computation..." << endl;

      search::KdTree<PointXYZRGB>::Ptr search_tree (new search::KdTree<PointXYZRGB>);
      gradient_est.setSearchMethod(search_tree);
      gradient_est.setRadiusSearch(search_radius);

      cout << "[CNNTransformStrategy::createGradients] "
        << "Setting input cloud: " <<  points->size() << " points" << endl;

      cout << "[CNNTransformStrategy::createGradients] "
        << "Setting input normals: " << normals->size() << " normals" << endl;

      cout << "[CNNTransformStrategy::createGradients] "
        << "Setting search radius: "
        << gradient_est.getRadiusSearch() << endl;

      cout << "[CNNTransformStrategy::createGradients] "
        << "Computing intensity gradients... " << endl;

      boost::timer t;
      gradient_est.compute(*gradients);

      cout << "[CNNTransformStrategy::createGradients] "
        << "Computation done. Computed " << gradients->size()
        << " gradients in " << t.elapsed() << " seconds. " << endl;
    }

    void filter(typename PointCloud<PointT>::Ptr cloud,
        typename PointCloud<PointT>::Ptr cloud_filtered,
        float leaf_size = 0.01f)
    {
      cout << "[CNNTransformationStrategy::filter] Input cloud: "
        << cloud->points.size() << " points" << endl;

      typename PointCloud<PointT>::Ptr tmp_ptr1(new PointCloud<PointT>);

      VoxelGrid<PointT> vox_grid;
      vox_grid.setInputCloud(cloud);
      vox_grid.setSaveLeafLayout(true);
      vox_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

      cout << "[CNNTransformationStrategy::filter] Creating a voxel grid. "
        << "Leaf size: [" << leaf_size << ", " << leaf_size
        << ", " << leaf_size << "]" << endl;

      // Skip the rest...
      // vox_grid.filter(*tmp_ptr1);
      vox_grid.filter(*cloud_filtered);

      cout << "[CNNTransformationStrategy::filter] Result of voxel grid"
        << " filtering:" << cloud_filtered->points.size()
        << " points remaining" << endl;

      return;
    };

    void computePFHFeatures(PointCloud<PointXYZRGB>::Ptr &points,
        PointCloud<Normal>::Ptr &normals,
        PointCloud<PFHSignature125>::Ptr &descriptors,
        float feature_radius = 0.08)
    {
      PFHEstimation<PointXYZRGB, Normal, PFHSignature125> pfh_est;

      pfh_est.setSearchMethod(search::KdTree<PointXYZRGB>::Ptr(new search::KdTree<PointXYZRGB>));
      pfh_est.setRadiusSearch(feature_radius);

      pfh_est.setInputCloud(points);
      pfh_est.setInputNormals(normals);

      cout << "[CNNTransformStrategy::computePFHFeatures]"
        << " Setting input cloud: " << points->size() << " points" << endl;
      cout << "[CNNTransformStrategy::computePFHFeatures]"
        << " Setting input normals: " << normals->size() << " normals"
        << endl;
      cout << "[CNNTransformStrategy::computePFHFeatures]"
        << " Feature radius = " << feature_radius << endl;
      pfh_est.compute(*descriptors);
    };

    template<typename FeatureT>
      void computeRIFTFeatures(PointCloud<PointXYZRGB>::Ptr &points,
          PointCloud<IntensityGradient>::Ptr &gradients,
          typename PointCloud<FeatureT>::Ptr &rift_descriptors,
          double search_radius = 0.1,
          int distance_bins = 4, int gradient_bins = 8)
      {
        RIFTEstimation<PointXYZRGB, IntensityGradient, FeatureT> rift_est;
        search::KdTree<PointXYZRGB>::Ptr search_tree (new search::KdTree<PointXYZRGB>);

        cout << "[CNNTransformStrategy::computeRIFTFeatures] "
          << "Starting RIFT features esitmation..." << endl;

        rift_est.setInputCloud(points);
        rift_est.setInputGradient(gradients);

        cout << "[CNNTransformStrategy::computeRIFTFeatures] "
          << "Setting input cloud: " << points->size() << " points" << endl;

        cout << "[CNNTransformStrategy::computeRIFTFeatures] "
          << "Setting input intensity gradients: " << gradients->size()
          << " points" << endl;

        rift_est.setRadiusSearch(search_radius);
        rift_est.setNrDistanceBins(distance_bins);
        rift_est.setNrGradientBins(gradient_bins);

        cout << "[CNNTransformStrategy::computeRIFTFeatures] "
          << "Setting search radius: "
          << rift_est.getRadiusSearch() << endl;
        cout << "[CNNTransformStrategy::computeRIFTFeatures] "
          << "Setting the number of distance bins: "
          << rift_est.getNrDistanceBins() << endl;
        cout << "[CNNTransformStrategy::computeRIFTFeatures] "
          << "Setting the number of gradient bins: "
          << rift_est.getNrGradientBins() << endl;

        cout << "[CNNTransformStrategy::computeRIFTFeatures] "
          << "Computing RIFT features..." << endl;

        boost::timer t;
        rift_est.compute(*rift_descriptors);

        cout << "[CNNTransformStrategy::computeRIFTFeatures] "
          << "Computation done. Computed " << rift_descriptors->size()
          << " features in " << t.elapsed() << " seconds." << endl;

        for (int i = 0; i < rift_descriptors->size(); i++) {
          for (float bucket : (*rift_descriptors)[i].histogram) {
            if (!isfinite(bucket)) {
              cerr << "[CNNTransformStrategy::computeRIFTFeatures] "
                << "RIFT descriptor " << i << ": " << (*rift_descriptors)[i]
                << " is invalid." << endl;
              break;
            }
          }
        }
      }

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
      findCoherentNeighbors(typename PointCloud<FeatureT>::Ptr source_features,
          typename PointCloud<FeatureT>::Ptr target_features,
          vector<vector<PointWithScore>> &coherent_neighbours,
          int k = 5)
      {
        coherent_neighbours.resize(source_features->size());

        cout << "[CNNTransformStrategy::findCoherentNeighbors] "
          << "Starting neighbor search..." << endl;

        pcl::KdTreeFLANN<FeatureT> searchtree;
        searchtree.setInputCloud(target_features);

        vector<int> k_indices(k);
        vector<float> k_squared_distances(k);

        boost::timer t;

        for (size_t i = 0; i < source_features->size(); i++) {
          searchtree.nearestKSearch(*source_features, i, k, k_indices, k_squared_distances);
          for (int y = 0; y < k; y++) {
            PointWithScore a(k_indices[y], 1/k_squared_distances[y]);
            coherent_neighbours[i].push_back(a);
          }
          sort(coherent_neighbours[i].begin(), coherent_neighbours[i].end());
        }

        cout << "[CNNTransformStrategy::findCoherentNeighbors] "
          << "Neighbor search done. Found " << k * source_features->size() 
          << " neighbors in " << t.elapsed() << " seconds" << endl;
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
        vector<vector<PointWithScore>> &coherent_neighbors)
    {
      RGBValue red;
      red.Red=0xff;
      red.Alpha=0;
      red.Blue=0;
      red.Green=0;
      for (int i = 0; i < src->size(); i++) {
        pcl::PointXYZRGB& ps = src->points[i];
        if (i > coherent_neighbors.size()
            || coherent_neighbors[i].size() == 0) {
          ps.rgba = red.long_value;
          continue;
        }
        std::sort(coherent_neighbors[i].begin(), coherent_neighbors[i].end());
        int targetpoint = coherent_neighbors[i].back().index;
        if (targetpoint < 0 || targetpoint >= target->size()) {
          std::cout << " ERROR targetpoint " << targetpoint
            << " out of range" << std::endl;
          targetpoint = 1;
        }
        pcl::PointXYZRGB& pt = target->points[targetpoint];

        ps.rgba = pt.rgba;
      }
    }

    float getMeanSquaredFaceError(vector<vector<PointWithScore>> &coherentNeighbors) {
      float error = 0;
      for(int s = 0; s < coherentNeighbors.size(); s++) {
        if(coherentNeighbors[s].size() == 0) {
          continue;
        }
        int t = coherentNeighbors[s].back().index;
        error += (s-t)*(s-t);
      }
      return error;
    }

    float refineScores(vector<vector<PointWithScore>> &coherentNeighboursSource,
        vector<vector<PointWithScore>> &coherentNeighboursTarget, int k,
        bool deleteWithNoMatch) {
      int missing_matches = 0;

      // Step through the source cloud points.
      for (int source_index = 0;
          source_index < coherentNeighboursSource.size();
          source_index++) {
        bool found_match = false;
        sort(coherentNeighboursSource[source_index].begin(),
            coherentNeighboursSource[source_index].end());
        // Step through each neighbor (in the target cloud) of the current source point.
        for (int source_neighbor_index = 0;
            source_neighbor_index
            < coherentNeighboursSource[source_index].size();
            source_neighbor_index++) {
          int target_index =
            coherentNeighboursSource[source_index][source_neighbor_index].index;
          sort(coherentNeighboursTarget[target_index].begin(),
              coherentNeighboursTarget[target_index].end());
          // Step through each neighbor (in the source cloud) of the current target point.
          for (int target_neighbor_index = 0;
              target_neighbor_index
              < coherentNeighboursTarget[target_index].size();
              target_neighbor_index++) {
            // Check whether the current source point appears in the neighbor list of the current target point.
            if (coherentNeighboursTarget[target_index][target_neighbor_index].index
                == source_index) {
              // cout << " redefine source: " << source_index << " (" << source_neighbor_index << ") target:" << target_index << " (" << target_neighbor_index << ") Score before:" << coherentNeighboursSource[source_index][source_neighbor_index].score;
              // Refine the score of the current point in the target cloud.
              // Add the position in the neighbor list to the score. (Higher Position => higher score)
              coherentNeighboursSource[source_index][source_neighbor_index].score *=
                (target_neighbor_index + 1);
              // cout << " after " << coherentNeighboursSource[source_index][source_neighbor_index].score << endl;
              found_match = true;
            }
          }
        }
        if (!found_match) {
          if (deleteWithNoMatch) {
            coherentNeighboursSource[source_index].clear();
          }
          missing_matches++;
        }
      }
      float missingmatches = (missing_matches * 100.0f)
        / coherentNeighboursSource.size();
      cout << "Found no matches for " << missing_matches << " points. ("
        << missingmatches << "%)" << endl;
      return missingmatches;
    }

  public:
    typename PointCloud<PointT>::Ptr transform(
        const typename PointCloud<PointT>::Ptr source,
        const typename PointCloud<PointT>::Ptr target,
        boost::shared_ptr<Configuration> configuration)
    {
      typename PointCloud<PointT>::Ptr result (new PointCloud<PointT>);
      float resultErrorRate = 400;

      PointCloud<PointXYZRGB>::Ptr sourceFiltered(new PointCloud<PointXYZRGB>);
      PointCloud<PointXYZRGB>::Ptr targetFiltered(new PointCloud<PointXYZRGB>);

      PointCloud<IntensityGradient>::Ptr source_gradients(new PointCloud<IntensityGradient>);
      PointCloud<IntensityGradient>::Ptr target_gradients(new PointCloud<IntensityGradient>);

      if (configuration->getFilterMethod() == VOXELGRIDFILTER) {
        filter(source, sourceFiltered, configuration->getFilterLeafSize());
        filter(target, targetFiltered, configuration->getFilterLeafSize());
      }
      else {
        sourceFiltered = source;
        targetFiltered = target;
      }

      for (float xdegree = 0; xdegree < 360.0f; xdegree+=configuration->getInt("degreesteps", 5)) {
        for (float ydegree = 0; ydegree < 360.0f; ydegree+=configuration->getInt("degreesteps", 5)) {
          for (float zdegree = 0; zdegree < 360.0f; zdegree+=configuration->getInt("degreesteps", 5)) {
            PointCloud<Normal>::Ptr source_normals(new PointCloud<Normal>);
            PointCloud<Normal>::Ptr target_normals(new PointCloud<Normal>);
            PointCloud<PFHSignature125>::Ptr source_descriptors(
                new PointCloud<PFHSignature125>);
            PointCloud<PFHSignature125>::Ptr target_descriptors(
                new PointCloud<PFHSignature125>);

            int k = configuration->getNearestNeighborsToSearch();

            vector<vector<PointWithScore>> coherentNeighboursSource;
            vector<vector<PointWithScore>> coherentNeighboursTarget;

            switch (configuration->getFeatureFormat()) {
              case PFHFEATURE:
                std::cout
                  << "[CNNTransformStrategy::transform] Using PFH features."
                  << std::endl;
                createNormals<Normal>(targetFiltered, target_normals,
                    configuration->getFloat("normalradius"));
                createNormals<Normal>(sourceFiltered, source_normals,
                    configuration->getFloat("normalradius"));
                computePFHFeatures(sourceFiltered, source_normals,
                    source_descriptors,
                    configuration->getFloat("pfhradius"));
                computePFHFeatures(targetFiltered, target_normals,
                    target_descriptors,
                    configuration->getFloat("pfhradius"));
                findCoherentNeighbors<PFHSignature125>(source_descriptors,
                    target_descriptors, coherentNeighboursSource, k);
                findCoherentNeighbors<PFHSignature125>(target_descriptors,
                    source_descriptors, coherentNeighboursTarget, k);
                break;
              case RIFTFEATURE:
                std::cout << "[CNNTransformStrategy::transform] Using RIFT features." << std::endl;
                createNormals<Normal>(sourceFiltered, source_normals, configuration->getFloat("normalradius"));
                createNormals<Normal>(targetFiltered, target_normals, configuration->getFloat("normalradius"));
                createGradients(sourceFiltered, source_normals, source_gradients);
                createGradients(targetFiltered, target_normals, target_gradients);
                computeRIFTFeatures<PFHSignature125>(sourceFiltered, source_gradients, source_descriptors);
                computeRIFTFeatures<PFHSignature125>(targetFiltered, target_gradients, target_descriptors);
                findCoherentNeighbors<PFHSignature125>(source_descriptors, target_descriptors, coherentNeighboursSource, k);
              case XYZRGBFEATURE:
                std::cout
                  << "[CNNTransformStrategy::transform] Using xyzrgb points directly as features."
                  << std::endl;
                findCoherentNeighbors<PointT>(sourceFiltered, targetFiltered,
                    coherentNeighboursSource, k);
                findCoherentNeighbors<PointT>(targetFiltered, sourceFiltered,
                    coherentNeighboursTarget, k);
                break;
              case PERFECTFEATURE:
                std::cout
                  << "[CNNTransformStrategy::transform] Using perfect point matching."
                  << std::endl;
                std::cout
                  << "  Warning: This can only be used with preconstructed sample faces."
                  << std::endl;
                findPerfectCoherentNeighbours(sourceFiltered, targetFiltered,
                    coherentNeighboursSource, k);
                findPerfectCoherentNeighbours(targetFiltered, sourceFiltered,
                    coherentNeighboursTarget, k);
                break;
              default:
                std::cout
                  << "[CNNTransformStrategy::transform] Feature Format not set."
                  << std::endl;
            }
            float errorrate=300.0f;
            for (int i = 0;
                i < configuration->getInt("refinescoresiteration", 1);
                i++) {
              float errorbefore = getMeanSquaredFaceError(
                  coherentNeighboursSource);
              errorrate=refineScores(coherentNeighboursSource, coherentNeighboursTarget,
                  k, configuration->getBool("markerrors"));
              refineScores(coherentNeighboursTarget, coherentNeighboursSource,
                  k, configuration->getBool("markerrors"));
              std::cout << "Face Error before: " << errorbefore
                << " after refine: "
                << getMeanSquaredFaceError(coherentNeighboursSource)
                << std::endl;
            }
            std::cout<<" Rotate target X:"<<xdegree<<" Y: "<<ydegree<<" Z: "<<zdegree<<" degree"<<std::endl;
            if(errorrate<resultErrorRate){
              result = sourceFiltered;
              replaceColors(result, targetFiltered, coherentNeighboursSource);
              resultErrorRate=errorrate;
            }
            DemoVisualizer::rotateXYZ(targetFiltered,xdegree,ydegree,	zdegree);
          }
        }
      }

      std::cout<<" Lowest Error rate:"<<resultErrorRate<<std::endl;
      return result;
    }
};

#endif
