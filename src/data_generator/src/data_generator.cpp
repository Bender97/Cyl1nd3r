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

std::vector<std::string> cam_front_paths;
std::vector<std::string> cam_front_left_paths;
std::vector<std::string> cam_front_right_paths;
std::vector<std::string> cam_back_paths;
std::vector<std::string> cam_back_left_paths;
std::vector<std::string> cam_back_right_paths;

float max_range = 21.0f;//mt
float min_range = 1.0f;//mt
float range_step = 2.0f;// mt
float angle_step = 20.0f; //[degrees]

int tot_ranges = (int) ((max_range - min_range) / range_step);
int tot_angles = (int) (360 / angle_step);

float range=1, angle = 0;
float rad_angle;
float x, y, z=0.0f;
std::vector<uint32_t> colors;

std::vector<tf::Transform> cs_record;
std::vector<tf::Transform> pr_record;
std::vector<tf::Transform> p1_record;
std::vector<tf::Transform> c1_record;

std::vector<tf::Matrix3x3> camera_intrinsics;

void fillTransformationMatrix(std::string &line, tf::Transform &tr) {
    std::stringstream ss(line);
    float vals[12];
    std::string str;

    for (int i=0; i<12; i++) {
        std::getline(ss, str, ' ');
        vals[i] = stof(str);
    }

    tr.setBasis( tf::Matrix3x3(vals[0], vals[1], vals[2], vals[4], vals[5], vals[6], vals[8], vals[9], vals[10]));
    tr.setOrigin( tf::Vector3(vals[3], vals[7], vals[11]));

}

void fillCameraIntrinsicsMatrix(std::string &line, tf::Matrix3x3 &ci) {
    std::stringstream ss(line);
    float vals[9];
    std::string str;

    for (int i=0; i<9; i++) {
        std::getline(ss, str, ' ');
        vals[i] = stof(str);
    }

    ci.setValue( vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7], vals[8]);
}

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

void loadLabels(std::string &labelpath, std::vector<uint8_t> &labels) {
    uint8_t f;
    std::ifstream fin(labelpath, std::ios::binary);
    labels.clear();
    while (fin.read(reinterpret_cast<char*>(&f), sizeof(uint8_t))) {
        labels.push_back(f);
    }
    fin.close();
}

void readPaths(std::string recipepath, std::vector<std::string> &scanpaths, std::vector<std::string> &labelpaths,
               std::vector<tf::Transform> &poses) {
    scanpaths.clear();
    labelpaths.clear();
    cam_front_paths.clear();
    cam_front_left_paths.clear();
    cam_front_right_paths.clear();
    cam_back_paths.clear();
    cam_back_left_paths.clear();
    cam_back_right_paths.clear();
    poses.clear();

    cs_record.clear();
    pr_record.clear();
    p1_record.clear();
    c1_record.clear();

    std::string line;
    std::stringstream ss;

    tf::Transform tr;
    tf::Matrix3x3 ci;

    std::ifstream fin(recipepath);
    while(true) {
        if (!std::getline(fin, line)) break;
        scanpaths.push_back(line);

        std::getline(fin, line);
        labelpaths.push_back(line);

        std::getline(fin, line);
        cam_front_paths.push_back(line);

        std::getline(fin, line);
        cam_front_left_paths.push_back(line);

        std::getline(fin, line);
        cam_front_right_paths.push_back(line);

        std::getline(fin, line);
        cam_back_paths.push_back(line);

        std::getline(fin, line);
        cam_back_left_paths.push_back(line);

        std::getline(fin, line);
        cam_back_right_paths.push_back(line);

        std::getline(fin, line);
        fillTransformationMatrix(line, tr);
        poses.push_back(tr);

        // camera intrinsics
        std::getline(fin, line);
        fillCameraIntrinsicsMatrix(line, ci);
        camera_intrinsics.push_back(ci);

        // records
        std::getline(fin, line);
        fillTransformationMatrix(line, tr);
        cs_record.push_back(tr);

        std::getline(fin, line);
        fillTransformationMatrix(line, tr);
        pr_record.push_back(tr);

        std::getline(fin, line);
        fillTransformationMatrix(line, tr);
        p1_record.push_back(tr);

        std::getline(fin, line);
        fillTransformationMatrix(line, tr);
        c1_record.push_back(tr);

    }
    fin.close();
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

void cloudToMsg(std::vector<float> &cloud, std::vector<uint8_t> &labels, sensor_msgs::PointCloud2 &msg, int seq, ros::Time &stamp ) {
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

            int id = labels[(i-3) >> 2];
            if (id>0 && id<=31) uint32_tToBytes.value = getRGB[ id ];
            else uint32_tToBytes.value = 0xff0000;

            msg.data.push_back(uint32_tToBytes.byte[0]);
            msg.data.push_back(uint32_tToBytes.byte[1]);
            msg.data.push_back(uint32_tToBytes.byte[2]);
            msg.data.push_back(uint32_tToBytes.byte[3]);
        }
    }

    msg.row_step = msg.data.size();
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
    std::vector<uint8_t> labels;
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


        auto camera_front_msg = readImageFile(cam_front_paths[count]).value();
        camera_front_msg.header.frame_id = "camera_front";
        camera_front_msg.header.stamp = stamp;
        camera_front_pub.publish(camera_front_msg);


        MapTrans.stamp_ = stamp;

        tf::Vector3 point;
        cv::Mat img = cv::imread(cam_front_paths[count]);

        float tempx, tempy, tempz;
        tf::Matrix3x3 rot;
        tf::Vector3 trans;

        tf::Matrix3x3 ci;

        for (int p=0, cont=0; p<cloud.size(); p+=4, cont++) {

            x = cloud[p]; y = cloud[p+1]; z = cloud[p+2];
            if (p==0) std::cout << "0) points [0]: " << x << " " << y << " " << z << std::endl;

//            point = cs_record[count].operator*(point);
//            point = pr_record[count].operator*(point);
//            point = p1_record[count].operator*(point);
//            point = c1_record[count].operator*(point);

            rot = cs_record[count].getBasis(); trans = cs_record[count].getOrigin();
            tempx = x * rot[0][0] + y * rot[0][1] + z * rot[0][2] + trans[0];
            tempy = x * rot[1][0] + y * rot[1][1] + z * rot[1][2] + trans[1];
            tempz = x * rot[2][0] + y * rot[2][1] + z * rot[2][2] + trans[2];
            x = tempx; y = tempy; z = tempz;

            if (p==0) std::cout << "1) points [0]: " << x << " " << y << " " << z << std::endl;

            rot = pr_record[count].getBasis(); trans = pr_record[count].getOrigin();
            tempx = x * rot[0][0] + y * rot[0][1] + z * rot[0][2] + trans[0];
            tempy = x * rot[1][0] + y * rot[1][1] + z * rot[1][2] + trans[1];
            tempz = x * rot[2][0] + y * rot[2][1] + z * rot[2][2] + trans[2];
            x = tempx; y = tempy; z = tempz;

            if (p==0) std::cout << "2) points [0]: " << x << " " << y << " " << z << std::endl;

            rot = p1_record[count].getBasis().transpose(); trans = p1_record[count].getOrigin();
            x -= trans[0]; y-= trans[1]; z -=trans[2];
            tempx = x * rot[0][0] + y * rot[0][1] + z * rot[0][2]; // - trans[0];
            tempy = x * rot[1][0] + y * rot[1][1] + z * rot[1][2]; // - trans[1];
            tempz = x * rot[2][0] + y * rot[2][1] + z * rot[2][2]; // - trans[2];
            x = tempx; y = tempy; z = tempz;

            if (p==0) std::cout << "3) points [0]: " << x << " " << y << " " << z << std::endl;

            rot = c1_record[count].getBasis().transpose(); trans = c1_record[count].getOrigin();
            x -= trans[0]; y-= trans[1]; z -=trans[2];
            tempx = x * rot[0][0] + y * rot[0][1] + z * rot[0][2]; // - trans[0];
            tempy = x * rot[1][0] + y * rot[1][1] + z * rot[1][2]; // - trans[1];
            tempz = x * rot[2][0] + y * rot[2][1] + z * rot[2][2]; // - trans[2];
            x = tempx; y = tempy; z = tempz;

            if (p==0) std::cout << "4) points [0]: " << x << " " << y << " " << z << std::endl;


            if (tempz<3) continue;

            ci = camera_intrinsics[count];
            tempx = x * ci[0][0] + y * ci[0][1] + z * ci[0][2];
            tempy = x * ci[1][0] + y * ci[1][1] + z * ci[1][2];
            tempz = x * ci[2][0] + y * ci[2][1] + z * ci [2][2];

            tempx /= tempz;
            tempy /= tempz;

            if (p==0) std::cout << "5) points [0]: " << tempx << " " << tempy << " " << tempz << std::endl;

            if (p==0) std::cout << "6) mask   [0]: " << !(tempx<0 || tempx>=img.cols || tempy<0 || tempy>=img.rows) << std::endl;

            if (tempx<0 || tempx>=img.cols || tempy<0 || tempy>=img.rows) continue;

            auto &color = img.at<cv::Vec3b>((int)tempy,(int)tempx);
            uint32_tToBytes.value = getRGB[ labels[cont] ];
            //else uint32_tToBytes.value = 0xff0000;
            color[0] = uint32_tToBytes.byte[0];
            color[1] = uint32_tToBytes.byte[1];
            color[2] = uint32_tToBytes.byte[2];

        }

        cv::imshow("name", img);
        cv::waitKey(1);


        MapTrans.setData(poses[count]);

        tfBroadcaster.sendTransform(MapTrans);

        ROS_INFO("published cloud %d", count);


        ros::spinOnce();

        loop_rate.sleep();

        ++count;
    }


    return 0;
}