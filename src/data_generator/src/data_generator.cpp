#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <iostream>
#include <map>
#include <regex>
#include <tf/transform_listener.h>

#include "CameraInfo.h"
#include "macro.h"
#include "Reader.h"

float range=1, angle = 0;
float rad_angle;
float x, y, z=0.0f;
std::vector<uint32_t> colors;

std::vector<CameraInfo> cameras(6);

// set the coordinate of each cylinder section (since in lidar frame, they're immutable)
void initGridCloud(std::vector<float> &cloud, std::vector<uint32_t> &labels) {
    cloud.clear();
    for (;angle<360; angle+=angle_step) {
        for (range=min_range; range<max_range; range+=range_step) {
            rad_angle = angle * M_PI / 180.0f;
            x = range * std::cos(rad_angle);
            y = range * std::sin(rad_angle);
            cloud.push_back(x);         // x
            cloud.push_back(y);         // y
            cloud.push_back(-2.0f);     // z
            cloud.push_back(1.0f);      // r
        }
    }

}

// set a point's color based on the sector of the cylinder the point belongs to
void sortInGrid_getLabels(std::vector<float> &cloud, std::vector<uint32_t> &labels) {
    labels.clear();
    labels.resize((int) cloud.size()/4);

    for (size_t i=0, cont=0; i<cloud.size(); i+=4, cont++) {
        x = cloud[i];
        y = cloud[i+1];
//        float z = cloud[i+2];

        float distance = sqrtf(x*x + y*y);
        if (distance >= max_range || distance <= min_range) continue;

        int range_idx = (int) round((distance - min_range) / range_step);

        float angle_idx = ((float) std::atan2(y, x)) * 180.0f / M_PI;
        if (angle_idx<0) angle_idx+=360.0f;

        angle_idx /= angle_step;

        int idx = (int) angle_idx + range_idx * tot_ranges;

        labels[cont] = colors[idx];

    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "nuscenes_data_player");
    ros::NodeHandle n;

    tf::StampedTransform MapTrans;
    tf::TransformBroadcaster tfBroadcaster;
    sensor_msgs::PointCloud2 lidar_msg;
    sensor_msgs::Image camera_front_msg;

    ros::Publisher pointcloud_pub   = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1000);
    ros::Publisher prev_pointcloud_pub   = n.advertise<sensor_msgs::PointCloud2>("/prev_velodyne_points", 1000);
    ros::Publisher grid_pub         = n.advertise<sensor_msgs::PointCloud2>("/grid_points", 1000);
    ros::Publisher camera_front_pub = n.advertise<sensor_msgs::Image>("/camera_front", 1000);

    std::vector<float>      grid_cloud, lidar_cloud;
    std::vector<uint32_t>   grid_labels, lidar_labels;
    std::vector<std::string> scanpaths, labelpaths;
    std::vector<tf::Transform> poses;

//    tf::TransformListener listener;
//    tf::StampedTransform transform;

    // initialize
    build_msg_Fields(lidar_msg);
    initGridCloud(grid_cloud, grid_labels);
    MapTrans.frame_id_ = "map";
    MapTrans.child_frame_id_ = "velodyne";
    // read from recipe file
    readPaths(recipepath, scanpaths, labelpaths, poses, cameras);


    // compute all colors
    int tot_cells = (int) std::roundf(((max_range - min_range)*range_step) * (360.0f / angle_step));
    colors.resize(tot_cells);
    for (auto &color: colors) color =
                (( (int8_t) (100 + (double)std::rand() / RAND_MAX * 155) )<< 16 ) +
                (( (int8_t) (100 + (double)std::rand() / RAND_MAX * 155)) << 8) +
                   (int8_t) (100 + (double)std::rand() / RAND_MAX * 155);



    ros::Rate polling(1);
    while(camera_front_pub.getNumSubscribers()==0) polling.sleep();

    ros::Rate loop_rate(1);
    int count = 0;

    tf::Transform prev_t, curr_t, t;
    tf::Vector3 trans;
    tf::Matrix3x3 rot;
    float tempx, tempy, tempz;
    std::vector<float>      prev_cloud;
    std::vector<uint32_t>   prev_labels;

    while (ros::ok())
    {
        ros::Time stamp =  ros::Time::now();

        std::string lidarpath = scanpaths[count];
        loadLidarScans(lidarpath, lidar_cloud);

        std::string labelpath = labelpaths[count];
        loadLabels(labelpath, lidar_labels);

        sortInGrid_getLabels(lidar_cloud, grid_labels);

//        cloudToMsg(lidar_cloud, grid_labels, lidar_msg, count, stamp);
        cloudToMsg(lidar_cloud, lidar_labels, lidar_msg, count, stamp);
        pointcloud_pub.publish(lidar_msg);

        cloudToMsg(grid_cloud, colors, lidar_msg, count, stamp);
        grid_pub.publish(lidar_msg);

        // publish an image (to be visible from rviz)
//        camera_front_msg = readImageFile(cameras[0].getPath(count)).value();
//        camera_front_msg.header.frame_id = "camera_front";
//        camera_front_msg.header.stamp = stamp;
//        camera_front_pub.publish(camera_front_msg);

        // paint clouds to image (using cv::imshow)
        cameras[0].paintToImage(count, grid_cloud, colors, cam_front_window_name);
        cameras[1].paintToImage(count, lidar_cloud, lidar_labels, cam_front_left_window_name);

        // send vehicle ego pose
        MapTrans.stamp_ = stamp;
        MapTrans.setData(poses[count]);
        tfBroadcaster.sendTransform(MapTrans);

        if (count>0) {
            prev_t = poses[count-1];
            curr_t.setOrigin(- poses[count].getOrigin());
            curr_t.setBasis( poses[count].getBasis().transpose());
//            prev_t.setOrigin(- poses[count-1].getOrigin());
//            prev_t.setBasis( poses[count-1].getBasis().transpose());
//            curr_t = poses[count];

//            t = prev_t * curr_t;
//            trans = t.getOrigin();
//            rot = t.getBasis();

            trans = prev_t.getOrigin();
            rot = prev_t.getBasis();

            for (size_t i=0; i<prev_cloud.size(); i+=4) {
                x = prev_cloud[i]; y = prev_cloud[i+1]; z = prev_cloud[i+2];
                prev_cloud[ i ] = rot[0][0]*x + rot[0][1]*y + rot[0][2]*z + trans[0];
                prev_cloud[i+1] = rot[1][0]*x + rot[1][1]*y + rot[1][2]*z + trans[1];
                prev_cloud[i+2] = rot[2][0]*x + rot[2][1]*y + rot[2][2]*z + trans[2];
            }

            trans = curr_t.getOrigin();
            rot = curr_t.getBasis();

            for (size_t i=0; i<prev_cloud.size(); i+=4) {
                x = prev_cloud[i] + trans[0]; y = prev_cloud[i+1] + trans[1]; z = prev_cloud[i+2] + trans[2];
                prev_cloud[ i ] = rot[0][0]*x + rot[0][1]*y + rot[0][2]*z;
                prev_cloud[i+1] = rot[1][0]*x + rot[1][1]*y + rot[1][2]*z;
                prev_cloud[i+2] = rot[2][0]*x + rot[2][1]*y + rot[2][2]*z;
            }

            cloudToMsg(prev_cloud, prev_labels, lidar_msg, count, stamp);
            prev_pointcloud_pub.publish(lidar_msg);
            
        }

        ROS_INFO("published cloud %d", count);

        ros::spinOnce();

        loop_rate.sleep();

        prev_cloud = lidar_cloud;
        prev_labels = lidar_labels;

        ++count;
    }


    return 0;
}