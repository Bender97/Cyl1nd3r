//
// Created by fusy on 12/04/22.
//

#ifndef DATA_GENERATOR_CAMERAINFO_H
#define DATA_GENERATOR_CAMERAINFO_H

#include <tf/transform_datatypes.h>
#include <opencv2/opencv.hpp>
#include "macro.h"
#include "transform_utils.h"

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

    // variables to project points
    tf::Matrix3x3 cs_record_rot, pr_record_rot, p1_record_rot, c1_record_rot;          // ditto
    tf::Vector3 cs_record_trans, pr_record_trans, p1_record_trans, c1_record_trans;          // ditto
    float x, y, z, tempx, tempy, tempz;  // ditto
    int rows, cols;
    int px, py;

public:

    void reset();

    void insertCamPath(std::ifstream &fin);

    void insertCsRecord(std::ifstream &fin);

    void insertPrRecord(std::ifstream &fin);

    void insertC1Record(std::ifstream &fin);

    void insertP1Record(std::ifstream &fin);

    void insertCiRecord(std::ifstream &fin);

    void loadData(std::ifstream &fin);

    bool projectPoint();

    std::string getPath(size_t count);

    void paintToImage(int count, std::vector<float> &cloud, std::vector<uint32_t> &labels, const std::string &window_name);
};


#endif //DATA_GENERATOR_CAMERAINFO_H
