//
// Created by fusy on 12/04/22.
//

#include "CameraInfo.h"

void CameraInfo::reset() {
    cam_paths.clear();
    cs_record.clear();
    pr_record.clear();
    p1_record.clear();
    c1_record.clear();
    camera_intrinsics.clear();
}

void CameraInfo::insertCamPath(std::ifstream &fin) {
    std::getline(fin, line);
    cam_paths.push_back(line);
}

void CameraInfo::insertCsRecord(std::ifstream &fin) {
    std::getline(fin, line);
    fillTransformationMatrix(line, tr);
    cs_record.push_back(tr);
}

void CameraInfo::insertPrRecord(std::ifstream &fin) {
    std::getline(fin, line);
    fillTransformationMatrix(line, tr);
    pr_record.push_back(tr);
}

void CameraInfo::insertC1Record(std::ifstream &fin) {
    std::getline(fin, line);
    fillTransformationMatrix(line, tr);
    c1_record.push_back(tr);
}

void CameraInfo::insertP1Record(std::ifstream &fin) {
    std::getline(fin, line);
    fillTransformationMatrix(line, tr);
    p1_record.push_back(tr);
}

void CameraInfo::insertCiRecord(std::ifstream &fin) {
    std::getline(fin, line);
    fillCameraIntrinsicsMatrix(line, ci);
    camera_intrinsics.push_back(ci);
}

void CameraInfo::loadData(std::ifstream &fin) {
    insertCamPath(fin);     // camera image path
    insertCiRecord(fin);    // camera instrinsics
    insertCsRecord(fin);    // records
    insertPrRecord(fin);
    insertP1Record(fin);
    insertC1Record(fin);
}

bool CameraInfo::projectPoint(float x, float y, float z, int &px, int &py, int count, int rows, int cols) {
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

std::string CameraInfo::getPath(int count) {
    assert(count>=0 && count < cam_paths.size());
    return cam_paths[count];
}

void CameraInfo::paintToImage(int count, std::vector<float> &cloud, std::vector<uint32_t> &labels, std::string &window_name) {
    cv::Mat img = cv::imread(cam_paths[count]);

    int px, py;
    int skipped_points=0;

    for (int p=0, cont=0; p<cloud.size(); p+=4, cont++) {

        if ( ! projectPoint(cloud[p], cloud[p+1], cloud[p+2], px, py, count, img.rows, img.cols) ) {
            skipped_points++;
            continue;
        }
        // draw the circle
        //uint32_tToBytes.value = getRGB[ labels[cont] ];
        uint32_tToBytes.value = labels[cont];
        cv::circle(img, cv::Point(px, py), 5, cv::Scalar(uint32_tToBytes.byte[2],uint32_tToBytes.byte[1],uint32_tToBytes.byte[0]),-1,8,0);

    }

    cv::imshow(window_name, img);
    cv::waitKey(1);
}