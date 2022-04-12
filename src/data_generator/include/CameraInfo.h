//
// Created by fusy on 12/04/22.
//

#ifndef DATA_GENERATOR_CAMERAINFO_H
#define DATA_GENERATOR_CAMERAINFO_H

#include <tf/transform_datatypes.h>
#include "transform_utils.h"
#include <opencv2/opencv.hpp>
#include "macro.h"

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

    void reset();

    void insertCamPath(std::ifstream &fin);

    void insertCsRecord(std::ifstream &fin);

    void insertPrRecord(std::ifstream &fin);

    void insertC1Record(std::ifstream &fin);

    void insertP1Record(std::ifstream &fin);

    void insertCiRecord(std::ifstream &fin);

    void loadData(std::ifstream &fin);

    bool projectPoint(float x, float y, float z, int &px, int &py, int count, int rows, int cols);

    std::string getPath(int count);

    void paintToImage(int count, std::vector<float> &cloud, std::vector<uint32_t> &labels, std::string &window_name);
};


#endif //DATA_GENERATOR_CAMERAINFO_H
