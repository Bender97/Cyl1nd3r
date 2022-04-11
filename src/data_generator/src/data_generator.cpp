#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/Image.h>
#include <sstream>
#include <fstream>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <regex>
#include "macro.h"
#include<opencv2/opencv.hpp>

#include "transform_utils.h"


float range=1, angle = 0;
float rad_angle;
float x, y, z=0.0f;
std::vector<uint32_t> colors;

//CameraInfo cam_front;
//CameraInfo cam_front_left;
//CameraInfo cam_front_right;
//CameraInfo cam_back;
//CameraInfo cam_back_left;
//CameraInfo cam_back_right;

std::vector<CameraInfo> cameras(6);


void loadLidarScans(std::string &lidarpath, std::vector<float> &scans) {
    std::ifstream fin(lidarpath, std::ios::binary);
    scans.clear();
    uint8_t skipCounter = 0;
    float f;
    while (fin.read(reinterpret_cast<char*>(&f), sizeof(float))) {
        // skip 5th value of each point
        if (skipCounter < 4) {
            scans.push_back(f);
            skipCounter++;
        } else {
            skipCounter = 0;
        }
    }
    fin.close();
}

void loadLabels(std::string &labelpath, std::vector<uint32_t> &labels) {
    uint8_t f;
    std::ifstream fin(labelpath, std::ios::binary);
    labels.clear();
    while (fin.read(reinterpret_cast<char*>(&f), sizeof(uint8_t))) {
        labels.push_back(getRGB[f]);
    }
    fin.close();
}

void readPaths(std::string recipepath, std::vector<std::string> &scanpaths, std::vector<std::string> &labelpaths,
               std::vector<tf::Transform> &poses) {
    scanpaths.clear();
    labelpaths.clear();
    poses.clear();

    for (int i=0; i<6; i++) cameras[i].reset();

    std::string line;
    std::stringstream ss;

    tf::Transform tr;

    std::ifstream fin(recipepath);

    /** struct:
     * scanpath
     * label path
     * pose transform
     *  cam_path
     *  cam_intrinsics
     *  cam_cs
     *  cam_pr
     *  cam_p1
     *  cam_c1
     * (.. repeat ..)
     * **/


    while(true) {
        if (!std::getline(fin, line)) break;
        scanpaths.push_back(line);

        std::getline(fin, line);
        labelpaths.push_back(line);

        std::getline(fin, line);
        fillTransformationMatrix(line, tr);
        poses.push_back(tr);


        for (int i=0; i<6; i++) cameras[i].loadData(fin);


    }
    fin.close();

//    cam_front.summary();
}

boost::optional<sensor_msgs::Image>
readImageFile(const std::string &filePath) noexcept
{
    cv::Mat image;
    try {
        image = imread(filePath.c_str(), cv::IMREAD_COLOR);
        sensor_msgs::ImagePtr msg =
                cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        return boost::optional<sensor_msgs::Image>(*msg);

    } catch (const std::exception& e) {
        std::cout << (e.what()) << std::endl;
    }

    return boost::none;
}

void cloudToMsg(std::vector<float> &cloud, std::vector<uint32_t> &labels, sensor_msgs::PointCloud2 &msg, int seq, ros::Time &stamp ) {
    msg.header.seq = seq;
    msg.header.stamp = stamp;
    msg.width = cloud.size()/4;
    msg.data.clear();
    for (int i=0; i<cloud.size(); i++) {
        floatToBytes.value = cloud[i];
        msg.data.push_back(floatToBytes.byte[0]);
        msg.data.push_back(floatToBytes.byte[1]);
        msg.data.push_back(floatToBytes.byte[2]);
        msg.data.push_back(floatToBytes.byte[3]);
        if (i%4==3) {

            uint32_tToBytes.value = labels[i/4];

            msg.data.push_back(uint32_tToBytes.byte[0]);
            msg.data.push_back(uint32_tToBytes.byte[1]);
            msg.data.push_back(uint32_tToBytes.byte[2]);
            msg.data.push_back(uint32_tToBytes.byte[3]);
        }
    }

    msg.row_step = msg.data.size();
}

void initGridCloud(std::vector<float> &cloud, std::vector<uint32_t> &labels) {
    cloud.clear();
    //labels.clear();

    for (;angle<360; angle+=angle_step) {
        for (range=min_range; range<max_range; range+=range_step) {
            rad_angle = angle * M_PI / 180.0f;
            x = range * std::cos(rad_angle);
            y = range * std::sin(rad_angle);
            cloud.push_back(x);
            cloud.push_back(y);
            cloud.push_back(z);
            cloud.push_back(1.0f);
            //labels.push_back(31);
        }
    }

}


void updateLabels(std::vector<float> &cloud, std::vector<uint32_t> &labels) {
    labels.clear();
    labels.resize((int) cloud.size()/4);

    for (int i=0, cont=0; i<cloud.size(); i+=4, cont++) {
        x = cloud[i];
        y = cloud[i+1];
//        float z = cloud[i+2];

        float distance = sqrtf(x*x + y*y);
        if (distance >= max_range || distance <= min_range) continue;

        int range_idx = (int) round((distance - min_range) / range_step);

        float angle_idx = ((float) std::atan2(y, x)) * 180.0f / M_PI;
        if (angle_idx<0) angle_idx+=360.0f;

        angle_idx /= angle_step;

//        int idx = angle_idx * tot_angles + range_idx;
        int idx = (int) angle_idx + range_idx * tot_ranges;

        labels[cont] = colors[idx];

    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    tf::StampedTransform MapTrans;
    tf::TransformBroadcaster tfBroadcaster;
    MapTrans.frame_id_ = "map";
    MapTrans.child_frame_id_ = "velodyne";

    ros::NodeHandle n;
    int rate;

    n.param<int>("data_generator/rate", rate, 2);


    ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 1000);
    ros::Publisher grid_pub = n.advertise<sensor_msgs::PointCloud2>("/grid_points", 1000);
    ros::Publisher camera_front_pub = n.advertise<sensor_msgs::Image>("/camera_front", 1000);
    sensor_msgs::PointCloud2 msg;
    sensor_msgs::PointCloud2 grid_msg;
    build_msg_Fields(msg);

    std::vector<float>  grid_cloud;
    std::vector<uint32_t>    grid_labels;
    build_msg_Fields(grid_msg);
    initGridCloud(grid_cloud, grid_labels);

    std::vector<std::string> scanpaths, labelpaths;

    std::vector<tf::Transform> poses;

    readPaths(recipepath, scanpaths, labelpaths, poses);


    std::vector<float> cloud;
    std::vector<uint32_t> labels;
    sensor_msgs::Image img;

    // compute all colors

    int tot_cells = ((max_range - min_range)*range_step) * (360.0f / angle_step);

    std::cout << "TOT_CELLS: " << tot_cells << std::endl;

    colors.resize(tot_cells);
    for (auto &color: colors) color =
                (( (int8_t) (100 + (double)std::rand() / RAND_MAX * 155) )<< 16 ) +
                (( (int8_t) (100 + (double)std::rand() / RAND_MAX * 155)) << 8) +
                   (int8_t) (100 + (double)std::rand() / RAND_MAX * 155);

    ros::Rate loop_rate(rate);
    int count = 0;


    ros::Rate polling(1);
    while(camera_front_pub.getNumSubscribers()==0) polling.sleep();

    std::string cam_front_window_name = "cam_front";
    std::string cam_front_left_window_name = "cam_front_left";

    while (ros::ok())
    {


        ros::Time stamp =  ros::Time::now();

        std::string lidarpath = scanpaths[count];
        loadLidarScans(lidarpath, cloud);

        std::string labelpath = labelpaths[count];
        loadLabels(labelpath, labels);

        updateLabels(cloud, grid_labels);

        cloudToMsg(cloud, grid_labels, msg, count, stamp);
        pointcloud_pub.publish(msg);


        cloudToMsg(grid_cloud, colors, grid_msg, count, stamp);
        grid_pub.publish(grid_msg);


        auto camera_front_msg = readImageFile(cameras[0].getPath(count)).value();
        camera_front_msg.header.frame_id = "camera_front";
        camera_front_msg.header.stamp = stamp;
        camera_front_pub.publish(camera_front_msg);


        MapTrans.stamp_ = stamp;

        cameras[0].paintToImage(count, grid_cloud, colors, cam_front_window_name);
        cameras[1].paintToImage(count, cloud, labels, cam_front_left_window_name);


        MapTrans.setData(poses[count]);

        tfBroadcaster.sendTransform(MapTrans);

        ROS_INFO("published cloud %d", count);


        ros::spinOnce();

        loop_rate.sleep();

        ++count;
    }


    return 0;
}