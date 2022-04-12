//
// Created by fusy on 12/04/22.
//

#ifndef DATA_GENERATOR_READER_H
#define DATA_GENERATOR_READER_H

#include <tf/transform_datatypes.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "CameraInfo.h"
#include "macro.h"

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
               std::vector<tf::Transform> &poses, std::vector<CameraInfo> &cameras) {
    scanpaths.clear();
    labelpaths.clear();
    poses.clear();

    for (int i=0; i<6; i++) cameras[i].reset();

    std::string line;
    std::stringstream ss;

    tf::Transform tr;

    std::ifstream fin(recipepath);
    if (!fin) { std::cout << "ERROR OPENING recipe file: " << recipepath << std::endl; return; }
    /** struct of the recipe file:
     * scanpath  \n  label_path  \n  pose_transform
     * [ cam_path  \n  cam_intrinsics  \n  cam_cs  \n  cam_pr  \n  cam_p1  \n  cam_c1 ] ( ..repeat.. )
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

#endif //DATA_GENERATOR_READER_H
