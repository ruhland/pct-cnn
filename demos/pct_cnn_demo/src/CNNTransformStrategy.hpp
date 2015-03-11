#ifndef CNNTRANSFORMSTRATEGY_HPP_
#define CNNTRANSFORMSTRATEGY_HPP_

#define PCL_NO_PRECOMPILE

#include "TransformStrategy.hpp"

#include <queue>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include <boost/timer.hpp>

#include <opencv/cv.h>

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

#include "Util.hpp"

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

using namespace pcl::console;

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

        bool operator== (const PointWithScore &rp) const {
          return index == rp.index && score == rp.score;
        }

        friend std::ostream& operator<< (std::ostream& os, const PointWithScore& p) {
          os << "[ index:" << p.index << ", score:" << p.score << " ]";
          return os;
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

        INFO("Setting input cloud: %d points.", cloud->points.size());
        INFO("Setting search radius: %f.", normal_radius);

        nest.setInputCloud(cloud);
        nest.setSearchMethod(typename search::KdTree<PointT>::Ptr(new search::KdTree<PointT>));
        nest.setRadiusSearch(normal_radius);

        boost::timer t;

        nest.compute(*normals);

        INFO("Found %d normals. (%.2f sec)", normals->size(), t.elapsed());

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

    void voxelGridFilter(typename PointCloud<PointT>::Ptr cloud,
        typename PointCloud<PointT>::Ptr cloud_filtered,
        float leaf_size = 0.01f)
    {
      int points = cloud->size();
      INFO("Setting input cloud: %d points.", cloud->points.size());

      typename PointCloud<PointT>::Ptr tmp_ptr1(new PointCloud<PointT>);

      VoxelGrid<PointT> vox_grid;
      vox_grid.setInputCloud(cloud);
      vox_grid.setSaveLeafLayout(true);
      vox_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

      INFO("Creating voxel grid with leaf_size = %f.", leaf_size);

      vox_grid.filter(*cloud_filtered);

      boost::timer t;
      int points_new = cloud_filtered->size();

      INFO("Filtered %d/%d points (%.2f%%). (%.2f sec)",
          points-points_new, points, (float) (points-points_new)/points*100,
          t.elapsed());

      return;
    }

    void computePFHFeatures(PointCloud<PointXYZRGB>::Ptr const &points,
        PointCloud<Normal>::Ptr const &normals,
        PointCloud<PFHSignature125>::Ptr const &descriptors,
        float feature_radius = 0.08)
    {
      PFHEstimation<PointXYZRGB, Normal, PFHSignature125> pfh_est;

      pfh_est.setSearchMethod(search::KdTree<PointXYZRGB>::Ptr(new search::KdTree<PointXYZRGB>));
      pfh_est.setRadiusSearch(feature_radius);

      pfh_est.setInputCloud(points);
      pfh_est.setInputNormals(normals);

      INFO("Setting input cloud: %d points.", points->size());
      INFO("Setting input normals: %d normals.", normals->size());
      INFO("Setting feature radius: %f.", feature_radius);

      boost::timer t;
      pfh_est.compute(*descriptors);

      INFO("Computed %d descriptors (%.2f sec)",
          descriptors->size(), t.elapsed());
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
      findNearestNeighbors(typename PointCloud<FeatureT>::Ptr source_features,
          typename PointCloud<FeatureT>::Ptr target_features,
          vector<vector<PointWithScore>> &coherent_neighbours, int k)
      {
        coherent_neighbours.resize(source_features->size());

        INFO("Using %d source features.", source_features->size());
        INFO("Using %d target features.", target_features->size());

        pcl::KdTreeFLANN<FeatureT> searchtree;
        searchtree.setInputCloud(target_features);

        vector<int> k_indices(k);
        vector<float> k_squared_distances(k);

        boost::timer t;

        for (size_t i = 0; i < source_features->size(); i++) {
          searchtree.nearestKSearch(*source_features, i, k, k_indices, k_squared_distances);
          for (int y = 0; y < k; y++) {
            PointWithScore a(k_indices[y], 1.0 / k_squared_distances[y]);
            coherent_neighbours[i].push_back(a);
          }
          sort(coherent_neighbours[i].begin(), coherent_neighbours[i].end());
        }

        INFO("Found %d  neighbors (%.2f sec)", k * source_features->size(),
            t.elapsed());
      }

    /**
     * Find the perfect neighbor for every point in the source features cloud.
     * This is possible, because the point indices of our demo clouds can be 
     * inverted.
     */
    void
      findPerfectCoherentNeighbours(PointCloud<PointXYZRGB>::Ptr source_features,
          PointCloud<PointXYZRGB>::Ptr target_features,
          vector<vector<PointWithScore>> &coherent_neighbours, int k)
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
        vector<vector<PointWithScore>> &coherent_neighbors,
        bool mark_non_perfect = false)
    {
      RGBValue red;
      red.long_value = 0;
      red.Red =  0xff;

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
          ERROR("targetpoint %d out of range.", targetpoint);
          targetpoint = 1;
        }

        pcl::PointXYZRGB& pt = target->points[targetpoint];

        if (mark_non_perfect && targetpoint != i) {
          ps.rgba = 0xff0000ff;
        } else {
          ps.rgba = pt.rgba;
        }
      }
    }

    float getMeanSquaredFaceError(const vector<vector<PointWithScore>> &coherent_neighbors) {
      float error = 0;
      float counter = 0;
      for(int s = 0; s < coherent_neighbors.size(); s++) {
        if(coherent_neighbors[s].size() == 0)
          continue;
        int t = coherent_neighbors[s].back().index;
        error += (s-t)*(s-t);
        counter++;
      }
      return error/counter;
    }

    float refineScores(vector<vector<PointWithScore>> &source_neighbours,
        vector<vector<PointWithScore>> &target_neighbours, int k,
        bool delete_with_no_match = false)
    {
      int missing = 0;

      // Step through the vectors of nearest neighbours (in the target cloud)
      // of every source cloud point.
      for (auto &s_neighbours : source_neighbours) {
        bool found_match = false;
        // Sort the nearest neighbours according to their score.
        sort(s_neighbours.begin(), s_neighbours.end());
        // Step through each target cloud point in the vector of nearest
        // neighbours.
        for (auto &s_neighbour : s_neighbours) {
          auto &t_neighbours = target_neighbours[s_neighbour.index];
          sort(t_neighbours.begin(), t_neighbours.end());
          // Step through each neighbour point (in the source cloud)
          // of the current target point.
          for (auto &t_neighbour : t_neighbours) {
            // Check whether the current source point appears in the neighbor
            // list of the current target point.
            auto &source_ref = source_neighbours[t_neighbour.index];
            if (source_ref == s_neighbours) {
              // Refine the score of the current point in the target cloud.
              // Add the position in the neighbor list to the score.
              // (Higher Position => higher score)
              int pos = find(t_neighbours.begin(), t_neighbours.end(), t_neighbour) - t_neighbours.begin();
              s_neighbour.score *= (pos + 2);
              found_match = true;
            }
          }
        }
        if (!found_match) {
          if (delete_with_no_match)
            s_neighbours.clear();
          missing++;
        }
      }
      float mpercent = (missing * 100.0f) / source_neighbours.size();
      INFO("Found no matches for %d points. (%0.2f %%)", missing, mpercent);

      return mpercent;
    }

  public:
    typename PointCloud<PointT>::Ptr transform(
        const typename PointCloud<PointT>::Ptr source,
        const typename PointCloud<PointT>::Ptr target,
        boost::shared_ptr<Configuration> configuration)
    {
      typename PointCloud<PointT>::Ptr result (new PointCloud<PointT>);
      // TODO: I have no idea why we need this result_error_rate. Remove?
      float result_error_rate = 400;

      PointCloud<PointXYZRGB>::Ptr source_filtered(new PointCloud<PointXYZRGB>);
      PointCloud<PointXYZRGB>::Ptr target_filtered(new PointCloud<PointXYZRGB>);

      PointCloud<IntensityGradient>::Ptr source_gradients(new PointCloud<IntensityGradient>);
      PointCloud<IntensityGradient>::Ptr target_gradients(new PointCloud<IntensityGradient>);

      if (configuration->getFilterMethod() == VOXELGRIDFILTER) {
        voxelGridFilter(source, source_filtered, configuration->getFilterLeafSize());
        voxelGridFilter(target, target_filtered, configuration->getFilterLeafSize());
      } else {
        source_filtered = source;
        target_filtered = target;
      }

      int step = configuration->getInt("degreesteps", 5);
      float r_norm = configuration->getFloat("normalradius");
      float r_pfh = configuration->getFloat("pfhradius");

      for (float angle_x = 0; angle_x < 360; angle_x += step) {
        for (float angle_y = 0; angle_y < 360; angle_y += step) {
          for (float angle_z = 0; angle_z < 360; angle_z += step) {
            PointCloud<Normal>::Ptr source_normals(new PointCloud<Normal>);
            PointCloud<Normal>::Ptr target_normals(new PointCloud<Normal>);
            PointCloud<PFHSignature125>::Ptr source_descriptors(
                new PointCloud<PFHSignature125>);
            PointCloud<PFHSignature125>::Ptr target_descriptors(
                new PointCloud<PFHSignature125>);

            int k = configuration->getNearestNeighborsToSearch();

            vector<vector<PointWithScore>> cnn_source;
            vector<vector<PointWithScore>> cnn_target;

            switch (configuration->getFeatureFormat()) {
              case PFHFEATURE:
                INFO("Using PFH features");
                createNormals<Normal>(target_filtered, target_normals, r_norm);
                createNormals<Normal>(source_filtered, source_normals, r_norm);
                computePFHFeatures(source_filtered, source_normals,
                    source_descriptors, r_pfh);
                computePFHFeatures(target_filtered, target_normals,
                    target_descriptors, r_pfh);
                findNearestNeighbors<PFHSignature125>(source_descriptors,
                    target_descriptors, cnn_source, k);
                findNearestNeighbors<PFHSignature125>(target_descriptors,
                    source_descriptors, cnn_target, k);
                break;
              case RIFTFEATURE:
                INFO("Using RIFT features.");
                createNormals<Normal>(source_filtered, source_normals, r_norm);
                createNormals<Normal>(target_filtered, target_normals, r_norm);
                createGradients(source_filtered, source_normals, source_gradients);
                createGradients(target_filtered, target_normals, target_gradients);
                computeRIFTFeatures<PFHSignature125>(source_filtered, source_gradients, source_descriptors);
                computeRIFTFeatures<PFHSignature125>(target_filtered, target_gradients, target_descriptors);
                findNearestNeighbors<PFHSignature125>(source_descriptors, target_descriptors, cnn_source, k);
                break;
              case XYZRGBFEATURE:
                INFO("Using XYZRGP values as features.");
                findNearestNeighbors<PointT>(source_filtered, target_filtered,
                    cnn_source, k);
                findNearestNeighbors<PointT>(target_filtered, source_filtered,
                    cnn_target, k);
                break;
              case PERFECTFEATURE:
                INFO("Using perfect point matching.");
                INFO("This should only be used with the given sample faces.");
                findPerfectCoherentNeighbours(source_filtered, target_filtered,
                    cnn_source, k);
                findPerfectCoherentNeighbours(target_filtered, source_filtered,
                    cnn_target, k);
                break;
              default:
                INFO("Feature type not set.");
            }
            float errorrate = 300;
            int iterations = configuration->getInt("refinescoresiteration");

            if (iterations == 0) {
              INFO("Skipping forward-backward search.");
              INFO("Face MSE: %f", getMeanSquaredFaceError(cnn_source));
            }

            for (int i = 0; i < iterations; i++) {
              float errorbefore = getMeanSquaredFaceError(
                  cnn_source);
              errorrate = refineScores(cnn_source, cnn_target, k,
                  configuration->getBool("markerrors"));
              refineScores(cnn_target, cnn_source, k,
                  configuration->getBool("markerrors"));
              INFO("Face MSE before: %f, after: %f", errorbefore, 
                  getMeanSquaredFaceError(cnn_source));
            }
            INFO("Rotate target x:%f, y:%f, z:%f", angle_x, angle_y, angle_z);
            if(errorrate < result_error_rate){
              result = source_filtered;
              bool mark_non_perfect = configuration->getBool("marknonperfect");
              replaceColors(result, target_filtered, cnn_source, mark_non_perfect);
              result_error_rate = errorrate;
            }
            DemoVisualizer::rotateXYZ(target_filtered, angle_x,angle_y, angle_z);
          }
        }
      }

      INFO("Lowest error rate: %f", result_error_rate);

      return result;
    }
};

#endif
