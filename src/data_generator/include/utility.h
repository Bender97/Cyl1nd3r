#pragma once
#ifndef UTILITY_H
#define UTILITY_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>


#include <pcl_ros/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/ml.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <array>
#include <thread>
#include <random>
#include <chrono>

#include "opencv2/opencv.hpp"
#include <geometry_msgs/PolygonStamped.h>

#include <boost/filesystem.hpp>


#include "macro_utility.h"
#include "GridParams.h"
#include "Stats.h"
#include "DatasetStats.h"
#include "IOUtils.h"
#include "CellManagement.h"
#include "GridManagement.h"
#include "Features.h"
#include "Camera.h"

typedef pcl::PointXYZI PointType;


class ParamServer
{
public:
    // ROS Node Handle
    ros::NodeHandle nh;

    // Topics
    std::string pointCloudTopic;
    std::string leftImageTopic;
    std::string trueTraversabilityTopic;
    std::string predictedTraversabilityTopic;
    std::string integratedCloudsTopic;

    // Frames
    std::string lidarFrame;
    std::string baselinkFrame;
    std::string odometryFrame;
    std::string mapFrame;

    // Paths
    std::string base_dir_path;
    std::string data_path;
    std::string normalization_config_path;
    std::string linear_svm_path, poly_svm_path, rbf_svm_path;
    std::string out_stats_path;

    // Cameras
    Camera cameraLeft;
    bool show_grid_projection_on_image_flag;
    // camera projection matrices
    std::vector<double> param_P2;
    std::vector<double> param_Tr;

    // Grid
    GridParams grid;
    int min_points_in_bucket;
    int min_points_in_bucket_to_project_to_camera;
    int min_neighbors_to_propagate_labels;
    int predicted_label_weight;
    int curvity_buckets_num_singlescan, curvity_buckets_num_multiscan;
    int cloudsQueue_fixed_size;
    cv::Point2f gridBottomRight;

    // Features
    std::vector<short int> bucket;
    std::vector<short int> bucket_singlescan;
    std::vector<short int> bucket_multiscan;

    // SVM
    cv::Mat features_matrix, predictions_vector;
    std::vector<int> cellIsPredictable;

    // Traning parameters
    float training_ratio;
    int kfolds;
    float term_criteria;
    std::vector<int> skip_cols_with_index;
    bool use_linear_svm, use_poly_svm, use_rbf_svm;
    bool save_linear_svm, save_poly_svm, save_rbf_svm;
    std::vector<double> linearSVM_Nu, linearSVM_Gamma;
    std::vector<double> polySVM_Nu, polySVM_Gamma, polySVM_Degree;
    std::vector<double> rbfSVM_Nu, rbfSVM_Gamma;

    // Online System
    std::string svm_model_path;
    std::vector<int> feats_indexes;
    cv::Point3f scene_normal;


    float global_map_side;
    float max_point_height;

    int timing_verbosity;

    ParamServer()
    {

        // Topics
        nh.param<std::string>("trav_analysis/pointCloudTopic", pointCloudTopic, "/velodyne_points");
        nh.param<std::string>("trav_analysis/leftImageTopic", leftImageTopic, "/semantickitti/camera_color_left/image_raw");
        nh.param<std::string>("trav_analysis/trueTraversabilityTopic", trueTraversabilityTopic, "trav_analysis/trueTraversability");
        nh.param<std::string>("trav_analysis/predictedTraversabilityTopic", predictedTraversabilityTopic, "trav_analysis/predictedTraversability");
        nh.param<std::string>("trav_analysis/integratedClouds", integratedCloudsTopic, "trav_analysis/integratedClouds");

        // Frames
        nh.param<std::string>("trav_analysis/lidarFrame", lidarFrame, "velodyne");
        nh.param<std::string>("trav_analysis/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("trav_analysis/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("trav_analysis/mapFrame", mapFrame, "map");

        // Paths
        nh.param<std::string>("trav_analysis/baseDirPath", base_dir_path, "trav_analysis/");
        nh.param<std::string>("trav_analysis/dataPath", data_path, "data.txt");
        nh.param<std::string>("trav_analysis/normalizationConfigPath", normalization_config_path, "normalization_config.txt");
        nh.param<std::string>("trav_analysis/linearSVMPath", linear_svm_path, "linear_svm.yml");
        nh.param<std::string>("trav_analysis/polySVMPath", poly_svm_path, "poly_svm.yml");
        nh.param<std::string>("trav_analysis/rbfSVMPath", rbf_svm_path, "rbf_svm.yml");
        nh.param<std::string>("trav_analysis/outStatsPath", out_stats_path, "nopath");
        //assert(base_dir_path.length()>0);
        //assert(base_dir_path[base_dir_path.length()-1] == '/');

        // Cameras
        nh.param<std::vector<double>>("trav_analysis/P2", param_P2, std::vector<double>());
        nh.param<std::vector<double>>("trav_analysis/Tr", param_Tr, std::vector<double>());
        nh.param<bool>("trav_analysis/showGridProjectionOnImage_flag", show_grid_projection_on_image_flag, false);
        assert(param_P2.size()==12);    // P2 matrix has shape 3 x 4
        assert(param_Tr.size()==16);    // Tr matrix has shape 4 x 4    (the last row must be 0 0 0 1)
        cameraLeft = Camera(param_Tr, param_P2, grid);

        // Grid
        nh.param<int>("trav_analysis/minPointsInBucket", min_points_in_bucket, 5);
        nh.param<int>("trav_analysis/minPointsInBucketToProjectToCamera", min_points_in_bucket_to_project_to_camera, 10);
        nh.param<int>("trav_analysis/minNeighborsToPropagateLabel", min_neighbors_to_propagate_labels, 4);
        nh.param<int>("trav_analysis/predictedLabelWeight", predicted_label_weight, 3);
        nh.param<float>("trav_analysis/radiusOfAttention",  grid.radius_of_attention, 12.0f);
        nh.param<float>("trav_analysis/mapResolution",      grid.resolution, 1.5f);
        nh.param<float>("trav_analysis/internalCellResolution", grid.internal_resolution, 1.5f);
        assert(grid.radius_of_attention>0);      // otherwise, trivial behaviour
        assert(grid.resolution>0);               // map_resolution is used as denominator
        grid.half_resolution                     = grid.resolution / 2.0f;
        grid.resolution_squared                  = NUM2SQ(grid.resolution);
        grid.num_cells_per_side                  = (int) ( 2.0f * grid.radius_of_attention / grid.resolution);
        grid.half_num_cells_per_side             = (float) grid.num_cells_per_side / 2.0f;
        grid.num_cells_per_side_squared          = (int) pow(grid.num_cells_per_side, 2);
        grid.internal_num_cells_per_side         = (int) ceilf(grid.resolution / grid.internal_resolution);
        grid.internal_num_cells_per_side_squared = NUM2SQ(grid.internal_num_cells_per_side);
        Features::gridParams = &grid;
        gridBottomRight.x = (float) grid.half_num_cells_per_side * grid.resolution;
        gridBottomRight.y = gridBottomRight.x;

        // Vehicle
        nh.param<float>("trav_analysis/vehicleXMin", grid.vehicle_x_min, .0f);
        nh.param<float>("trav_analysis/vehicleXMax", grid.vehicle_x_max, .0f);
        nh.param<float>("trav_analysis/vehicleYMin", grid.vehicle_y_min, .0f);
        nh.param<float>("trav_analysis/vehicleYMax", grid.vehicle_y_max, .0f);

        // Features
        nh.param<int>("trav_analysis/curvityBucketsNum_singlescan", curvity_buckets_num_singlescan, 20);
        nh.param<int>("trav_analysis/curvityBucketsNum_multiscan", curvity_buckets_num_multiscan, 60);
        bucket_singlescan.resize(curvity_buckets_num_singlescan + 1);
        bucket_multiscan.resize(curvity_buckets_num_multiscan + 1);

        // Training Parameters
        nh.param<float>("trav_analysis/trainingRatio", training_ratio, 1.5f);
        nh.param<int>("trav_analysis/kfolds", kfolds, 10);
        nh.param<float>("trav_analysis/termCriteria", term_criteria, 1e-6);
        nh.param<std::vector<int>>("trav_analysis/skipColsWithIndex", skip_cols_with_index, std::vector<int>());
        nh.param<bool>("trav_analysis/useLinearSVM", use_linear_svm, false);
        nh.param<bool>("trav_analysis/usePolySVM", use_poly_svm, false);
        nh.param<bool>("trav_analysis/useRbfSVM", use_rbf_svm, false);
        nh.param<bool>("trav_analysis/saveLinearSVM", save_linear_svm, false);
        nh.param<bool>("trav_analysis/savePolySVM", save_poly_svm, false);
        nh.param<bool>("trav_analysis/saveRbfSVM", save_rbf_svm, false);
        nh.param<std::vector<double>>("trav_analysis/linearSVM_Nu", linearSVM_Nu, std::vector<double>());
        nh.param<std::vector<double>>("trav_analysis/linearSVM_Gamma", linearSVM_Gamma, std::vector<double>());
        nh.param<std::vector<double>>("trav_analysis/polySVM_Nu", polySVM_Nu, std::vector<double>());
        nh.param<std::vector<double>>("trav_analysis/polySVM_Gamma", polySVM_Gamma, std::vector<double>());
        nh.param<std::vector<double>>("trav_analysis/polySVM_Degree", polySVM_Degree, std::vector<double>());
        nh.param<std::vector<double>>("trav_analysis/rbfSVM_Nu", rbfSVM_Nu, std::vector<double>());
        nh.param<std::vector<double>>("trav_analysis/rbfSVM_Gamma", rbfSVM_Gamma, std::vector<double>());
        assert(training_ratio>.0f && training_ratio<1.0f);
        assert(kfolds>=2);

        // Online System
        nh.param<std::string>("trav_analysis/svmModelPath", svm_model_path, "rbf_svm.yml");
        nh.param<int>("trav_analysis/cloudsQueueFixedSize", cloudsQueue_fixed_size, 0);


        nh.param<float>("trav_analysis/globalMapSide", global_map_side, 20.0f);
        nh.param<float>("trav_analysis/maxPointHeight", max_point_height, 2.0f);

        nh.param<int>("trav_analysis/timingVerbosity", timing_verbosity, 0);

        usleep(100);

    }



    bool columnIsRequested(int col) {
        for (const auto c: skip_cols_with_index) if (col==c) return false;
        return true;
    }



    void getGridInLidarFrame(pcl::PointCloud<PointType>::Ptr gridIn, pcl::PointCloud<PointType>::Ptr gridOut, tf::TransformListener *listener ) const {
        gridIn->header.stamp = 0;
        gridIn->header.frame_id = mapFrame;
        pcl_ros::transformPointCloud(lidarFrame, *gridIn, *gridOut, (*listener));
    }



    void integrateClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& velodyneRGBCloud,
                          std::vector<int> &enqueuedCloudsSizes,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sum) const {

//        std::cout << "integrating start time: " << ros::Time::now() << std::endl;

        int i, final_size, old_data_size,  offset;

        /// compute final size of the integrated cloud
        final_size = (int) velodyneRGBCloud->points.size();
        for (i=0; i<cloudsQueue_fixed_size; ++i) final_size += enqueuedCloudsSizes[i];

        /// init temp data (cannot modify sum, we need that first part of the data!)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp;
        temp.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        temp->points.resize(final_size);

        /// copy new points up to velo size
        for (i=0; i<(int)velodyneRGBCloud->points.size(); ++i) temp->points[i] = velodyneRGBCloud->points[i];

        /// fill integrated cloud with the previous data
        offset = (int) velodyneRGBCloud->points.size();
        old_data_size = final_size - (int) velodyneRGBCloud->points.size();
        for (i=0; i<old_data_size; ++i) temp->points[offset + i] = sum->points[i];

        /// shift clouds size
        for (i=cloudsQueue_fixed_size-1; i>0; i--) enqueuedCloudsSizes[i] = enqueuedCloudsSizes[i-1];
        if (cloudsQueue_fixed_size>=1) enqueuedCloudsSizes[0] = (int) velodyneRGBCloud->points.size();

        sum->points.resize(final_size);
        *sum = *temp;

//        std::cout << "integrating end time: " << ros::Time::now() << std::endl;
    }



    /// calculate normal of the whole scene
    void computeScenePlaneNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, Features &feature) {

        feature.matA1 = cv::Mat::zeros(cv::Size(3, 3), CV_32F);
        feature.matD1 = cv::Mat::zeros(cv::Size(1, 3), CV_32F);
        feature.matV1 = cv::Mat::zeros(cv::Size(3, 3), CV_32F);

        feature.computeCorrelationMatrixComponents(cloud);
        cv::eigen(feature.matA1, feature.matD1, feature.matV1);

        scene_normal.x = feature.matV1.at<float>(2, 0);
        scene_normal.y = feature.matV1.at<float>(2, 1);
        scene_normal.z = std::abs(feature.matV1.at<float>(2, 2));
    }

    bool getCellFeatures(std::vector<pcl::PointXYZRGB *> &field_cell,
                              Features &feature, bool singlescan_flag,
                              PointType &belonging_cell_center,
                              cv::Mat &hsv_image,
                              PointType &cell,
                              DatasetStats &dataset_stats) {

        if (field_cell.size() < (size_t) min_points_in_bucket) {
            dataset_stats.empty_cell_amount++;
            return false;
        }

//        feature.label = (short int) belonging_cell_center.intensity;

        if (feature.label==UNKNOWN_CELL_LABEL) {
            dataset_stats.unknown_points_skipped++;
            return false;
        }

        bucket = (singlescan_flag ? bucket_singlescan : bucket_multiscan);
        int buckets_num = (singlescan_flag ? curvity_buckets_num_singlescan : curvity_buckets_num_multiscan);


        if (!feature.computeGeometricalFeatures(field_cell, scene_normal, belonging_cell_center, bucket, buckets_num)) {
            if (feature.label == TRAV_CELL_LABEL)
                dataset_stats.road_points_skipped_count++;
            else
                dataset_stats.non_road_points_skipped_count++;
            return false;
        }

        if (feature.label == TRAV_CELL_LABEL)
            dataset_stats.road_points_included_count++;
        else
            dataset_stats.non_road_points_included_count++;

        cameraLeft.computeColorFeatures(feature, hsv_image, cell);

        return true;
    }

    bool getCellFeaturesPred(std::vector<pcl::PointXYZRGB *> &field_cell,
                             Features &feature, bool singlescan_flag,
                             PointType &belonging_cell_center,
//                             cv::Mat &hsv_image,
                             PointType &cell,
                             DatasetStats &dataset_stats) {

        if (field_cell.size() < (size_t) min_points_in_bucket) return false;

        bucket = (singlescan_flag ? bucket_singlescan : bucket_multiscan);
        int buckets_num = (singlescan_flag ? curvity_buckets_num_singlescan : curvity_buckets_num_multiscan);

        if (!feature.computeGeometricalFeatures(field_cell, scene_normal,belonging_cell_center, bucket, buckets_num)) {
            return false;
        }

//        cameraLeft.computeColorFeatures(feature, hsv_image, cell);

        return true;
    }

    static float getAvgElevationOfPointsInCell(std::vector<pcl::PointXYZRGB *> cell) {
        float z_sum = .0f;
        for (auto &point: cell) z_sum += point->z;
        return ( z_sum / ( (float) cell.size() ) );
    }

    bool updateLidarToMapTransform( tf::TransformListener &listener, tf::StampedTransform &transform ) const {
        try{listener.lookupTransform(mapFrame, lidarFrame, ros::Time(0), transform); }
        catch (tf::TransformException &ex){ ROS_ERROR_STREAM("Transform Failure."); return false; }
        return true;
    }



    void transformIntegratedCloudsToLidarFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &integratedClouds, pcl::PointCloud<pcl::PointXYZRGB>::Ptr integratedClouds_lidar, uint64_t stamp, tf::TransformListener &listener) {
        integratedClouds->header.frame_id = mapFrame;
        integratedClouds->header.stamp = stamp;
        pcl_ros::transformPointCloud(lidarFrame,
                                     ros::Time(0),
                                     *integratedClouds,
                                     mapFrame,
                                     *integratedClouds_lidar,
                                     listener);
    }

    template<typename GenericPointCloudPtr>
    sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, GenericPointCloudPtr thisCloud, ros::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::PointCloud2 tempCloud;
        pcl::toROSMsg(*thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        if (thisPub->getNumSubscribers() != 0)
            thisPub->publish(tempCloud);
        return tempCloud;
    }


    void updateGridBottomRight_map(tf::StampedTransform &transform ) {
        gridBottomRight.x = (float) transform.getOrigin().x() - grid.half_num_cells_per_side * grid.resolution - grid.half_resolution;
        gridBottomRight.y = (float) transform.getOrigin().y() - grid.half_num_cells_per_side * grid.resolution - grid.half_resolution;
    }
};



#endif
