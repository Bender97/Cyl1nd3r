//
// Created by fusy on 07/04/22.
//

#ifndef DATA_GENERATOR_MACRO_H
#define DATA_GENERATOR_MACRO_H

#include <tf/transform_datatypes.h>
#include "transform_utils.h"
#include<opencv2/opencv.hpp>


/**************************** GENERAL DATA ****************************/

std::string recipepath = "/home/fusy/Documents/bin2pcd/ros_ws/playback_recipe.txt";
//std::string recipepath = "/home/fusy/Documents/bin2pcd/ros_ws/src/data_generator/playback_recipe.txt";

float max_range = 21.0f;    //mt
float min_range = 1.0f;     //mt
float range_step = 2.0f;    // mt
float angle_step = 20.0f;   //[degrees]

int tot_ranges = (int) ((max_range - min_range) / range_step);
int tot_angles = (int) (360 / angle_step);

/**********************************************************************/

uint32_t getRGB[] = {
        0x000000 , // 0 : noise
        0x4682b4 , // 1 : animal
        0x0000e6 , // 2 : human.pedestrian.adult, blue
        0x87ceeb , // 3 : human.pedestrian.child Skyblue,
        0x6495ed , // 4 : human.pedestrian.construction_worker  Cornflowerblue
        0xdb7093 , // 5 : human.pedestrian.personal_mobility  Palevioletred
        0x000080 , // 6 : human.pedestrian.police_officer  Navy,
        0xf08080 , // 7 : human.pedestrian.stroller Lightcoral
        0x8a2be2 , // 8 : human.pedestrian.wheelchair Blueviolet
        0x708090 , // 9 : movable_object.barrier Slategrey
        0xd2691e , // 10: movable_object.debris Chocolate
        0x696969 , // 11: movable_object.pushable_pullable Dimgrey
        0x2f4f4f , // 12: movable_object.trafficcone Darkslategrey
        0xbc8f8f , // 13: static_object.bicycle_rack Rosybrown
        0xdc143c , // 14: vehicle.bicycle Crimson
        0xff7f50 , // 15: vehicle.bus.bendy Coral
        0xff4500 , // 16: vehicle.bus.rigid Orangered
        0xff9e00 , // 17: vehicle.car Orange
        0xe99646 , // 18: vehicle.construction Darksalmon
        0xff5300 , // 19: vehicle.emergency.ambulance
        0xffd700 , // 20: vehicle.emergency.police Gold
        0xff3d63 , // 21: vehicle.motorcycle Red
        0xff8c00 , // 22: vehicle.trailer Darkorange
        0xff6347 , // 23: vehicle.truck Tomato
        0x00cfbf , // 24: flat.driveable_surface nuTonomy green
        0xaf004b , // 25: flat.other
        0x4b004b , // 26: flat.sidewalk
        0x70b43c , // 27: flat.terrain
        0xdeb887 , // 28: static.manmade Burlywood
        0xffe4c4 , // 29: static.other Bisque
        0x00af00 , // 20: static.vegetation Green
        0xfff0f5 // 31: vehicle.ego
};

class Point {
public:
    double x, y, z;
    Point(double _x, double _y, double _z) {
        x = _x;
        y = _y;
        z = _z;
    }
    Point() {};
};

// Convert float32 to 4 bytes
union
{
    float value;
    uint8_t byte[4];
} floatToBytes;
union
{
    uint32_t value;
    uint8_t byte[4];
} uint32_tToBytes;

void build_msg_Fields(sensor_msgs::PointCloud2 &msg) {
    sensor_msgs::PointField field;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = 0;
    field.count = 1;
    field.name = std::string("x");
    msg.fields.push_back(field);

    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = 4;
    field.count = 1;
    field.name = std::string("y");
    msg.fields.push_back(field);

    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = 8;
    field.count = 1;
    field.name = std::string("z");
    msg.fields.push_back(field);

    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = 12;
    field.count = 1;
    field.name = std::string("intensity");
    msg.fields.push_back(field);

    field.datatype = sensor_msgs::PointField::UINT32;
    field.offset = 16;
    field.count = 1;
    field.name = std::string("rgb");
    msg.fields.push_back(field);
//
    msg.is_dense = true;
    msg.header.frame_id = std::string("velodyne");;
    msg.height = 1;
    msg.is_bigendian = false;
    msg.point_step = sizeof(float) * 5;
//    msg.point_step = sizeof(float) * 4;
}


class CameraInfo {
    std::vector<std::string> cam_paths;
    std::vector<tf::Transform> cs_record;
    std::vector<tf::Transform> pr_record;
    std::vector<tf::Transform> p1_record;
    std::vector<tf::Transform> c1_record;
    std::vector<tf::Matrix3x3> camera_intrinsics;
    std::string line;   // used to read things, just a temp string
    tf::Transform tr;   // ditto
    tf::Matrix3x3 ci;   // ditto

public:

    void reset() {
        cam_paths.clear();
        cs_record.clear();
        pr_record.clear();
        p1_record.clear();
        c1_record.clear();
        camera_intrinsics.clear();
    }

    void summary() {
        std::cout << "cam paths: " << cam_paths.size() << ", records: " << cs_record.size() << std::endl;
    }

    void insertCamPath(std::ifstream &fin) {
        std::getline(fin, line);
        cam_paths.push_back(line);
    }

    void insertCsRecord(std::ifstream &fin) {
        std::getline(fin, line);
        fillTransformationMatrix(line, tr);
        cs_record.push_back(tr);
    }

    void insertPrRecord(std::ifstream &fin) {
        std::getline(fin, line);
        fillTransformationMatrix(line, tr);
        pr_record.push_back(tr);
    }

    void insertC1Record(std::ifstream &fin) {
        std::getline(fin, line);
        fillTransformationMatrix(line, tr);
        c1_record.push_back(tr);
    }

    void insertP1Record(std::ifstream &fin) {
        std::getline(fin, line);
        fillTransformationMatrix(line, tr);
        p1_record.push_back(tr);
    }

    void insertCiRecord(std::ifstream &fin) {
        std::getline(fin, line);
        fillCameraIntrinsicsMatrix(line, ci);
        camera_intrinsics.push_back(ci);
    }

    void loadData(std::ifstream &fin) {
        insertCamPath(fin);     // camera image path
        insertCiRecord(fin);    // camera instrinsics
        insertCsRecord(fin);    // records
        insertPrRecord(fin);
        insertP1Record(fin);
        insertC1Record(fin);
    }

    bool projectPoint(float x, float y, float z, int &px, int &py, int count, int rows, int cols) {
        float tempx, tempy, tempz;
        tf::Matrix3x3 rot;
        tf::Vector3 trans;

        rot = cs_record[count].getBasis(); trans = cs_record[count].getOrigin();
        tempx = x * rot[0][0] + y * rot[0][1] + z * rot[0][2] + trans[0];
        tempy = x * rot[1][0] + y * rot[1][1] + z * rot[1][2] + trans[1];
        tempz = x * rot[2][0] + y * rot[2][1] + z * rot[2][2] + trans[2];
        x = tempx; y = tempy; z = tempz;


        rot = pr_record[count].getBasis(); trans = pr_record[count].getOrigin();
        tempx = x * rot[0][0] + y * rot[0][1] + z * rot[0][2] + trans[0];
        tempy = x * rot[1][0] + y * rot[1][1] + z * rot[1][2] + trans[1];
        tempz = x * rot[2][0] + y * rot[2][1] + z * rot[2][2] + trans[2];
        x = tempx; y = tempy; z = tempz;


        rot = p1_record[count].getBasis().transpose(); trans = p1_record[count].getOrigin();
        x -= (float) trans[0]; y -= (float) trans[1]; z -= (float) trans[2];
        tempx = x * rot[0][0] + y * rot[0][1] + z * rot[0][2];
        tempy = x * rot[1][0] + y * rot[1][1] + z * rot[1][2];
        tempz = x * rot[2][0] + y * rot[2][1] + z * rot[2][2];
        x = tempx; y = tempy; z = tempz;


        rot = c1_record[count].getBasis().transpose(); trans = c1_record[count].getOrigin();
        x -= (float) trans[0]; y -= (float) trans[1]; z -= (float) trans[2];
        tempx = x * rot[0][0] + y * rot[0][1] + z * rot[0][2];
        tempy = x * rot[1][0] + y * rot[1][1] + z * rot[1][2];
        tempz = x * rot[2][0] + y * rot[2][1] + z * rot[2][2];
        x = tempx; y = tempy; z = tempz;

        if (tempz<3) return false;

        ci = camera_intrinsics[count];
        tempx = x * ci[0][0] + y * ci[0][1] + z * ci[0][2];
        tempy = x * ci[1][0] + y * ci[1][1] + z * ci[1][2];
        tempz = x * ci[2][0] + y * ci[2][1] + z * ci[2][2];


        px = (int) std::roundf(tempx / tempz);
        py = (int) std::roundf(tempy / tempz);


        if (px<0 || px>=cols || py<0 || py>=rows) return false;

        return true;
    }

    std::string getPath(int count) {
        assert(count>=0 && count < cam_paths.size());
        return cam_paths[count];
    }

    void paintToImage(int count, std::vector<float> &cloud, std::vector<uint8_t> &labels, std::string &window_name) {
        cv::Mat img = cv::imread(cam_paths[count]);

        int px, py;
        int skipped_points=0;

        for (int p=0, cont=0; p<cloud.size(); p+=4, cont++) {

            if ( ! projectPoint(cloud[p], cloud[p+1], cloud[p+2], px, py, count, img.rows, img.cols) ) {
                skipped_points++;
                continue;
            }
            // draw the circle
            uint32_tToBytes.value = getRGB[ labels[cont] ];
            cv::circle(img, cv::Point(px, py), 5, cv::Scalar(uint32_tToBytes.byte[0],uint32_tToBytes.byte[1],uint32_tToBytes.byte[2]),-1,8,0);

        }

        cv::imshow(window_name, img);
        cv::waitKey(1);
    }
};

#endif //DATA_GENERATOR_MACRO_H
