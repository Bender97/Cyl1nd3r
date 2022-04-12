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

bool CameraInfo::projectPoint() {

    tempx = x * cs_record_rot[0][0] + y * cs_record_rot[0][1] + z * cs_record_rot[0][2] + cs_record_trans[0];
    tempy = x * cs_record_rot[1][0] + y * cs_record_rot[1][1] + z * cs_record_rot[1][2] + cs_record_trans[1];
    tempz = x * cs_record_rot[2][0] + y * cs_record_rot[2][1] + z * cs_record_rot[2][2] + cs_record_trans[2];
    x = tempx; y = tempy; z = tempz;

    tempx = x * pr_record_rot[0][0] + y * pr_record_rot[0][1] + z * pr_record_rot[0][2] + pr_record_trans[0];
    tempy = x * pr_record_rot[1][0] + y * pr_record_rot[1][1] + z * pr_record_rot[1][2] + pr_record_trans[1];
    tempz = x * pr_record_rot[2][0] + y * pr_record_rot[2][1] + z * pr_record_rot[2][2] + pr_record_trans[2];
    x = tempx; y = tempy; z = tempz;

    x -= (float) p1_record_trans[0]; y -= (float) p1_record_trans[1]; z -= (float) p1_record_trans[2];
    tempx = x * p1_record_rot[0][0] + y * p1_record_rot[0][1] + z * p1_record_rot[0][2];
    tempy = x * p1_record_rot[1][0] + y * p1_record_rot[1][1] + z * p1_record_rot[1][2];
    tempz = x * p1_record_rot[2][0] + y * p1_record_rot[2][1] + z * p1_record_rot[2][2];
    x = tempx; y = tempy; z = tempz;

    x -= (float) c1_record_trans[0]; y -= (float) c1_record_trans[1]; z -= (float) c1_record_trans[2];
    tempx = x * c1_record_rot[0][0] + y * c1_record_rot[0][1] + z * c1_record_rot[0][2];
    tempy = x * c1_record_rot[1][0] + y * c1_record_rot[1][1] + z * c1_record_rot[1][2];
    tempz = x * c1_record_rot[2][0] + y * c1_record_rot[2][1] + z * c1_record_rot[2][2];
    x = tempx; y = tempy; z = tempz;

    if (tempz<3) return false;

    tempx = x * ci[0][0] + y * ci[0][1] + z * ci[0][2];
    tempy = x * ci[1][0] + y * ci[1][1] + z * ci[1][2];
    tempz = x * ci[2][0] + y * ci[2][1] + z * ci[2][2];


    px = (int) std::roundf(tempx / tempz);
    py = (int) std::roundf(tempy / tempz);


    if (px<0 || px>=cols || py<0 || py>=rows) return false;

    return true;
}

std::string CameraInfo::getPath(size_t count) {
    assert(count>=0 && count < cam_paths.size());
    return cam_paths[count];
}

void CameraInfo::paintToImage(int count, std::vector<float> &cloud, std::vector<uint32_t> &labels, const std::string &window_name) {
    cv::Mat img = cv::imread(cam_paths[count]);

    cs_record_rot = cs_record[count].getBasis(); cs_record_trans = cs_record[count].getOrigin();
    pr_record_rot = pr_record[count].getBasis(); pr_record_trans = pr_record[count].getOrigin();
    p1_record_rot = p1_record[count].getBasis().transpose(); p1_record_trans = p1_record[count].getOrigin();
    c1_record_rot = c1_record[count].getBasis().transpose(); c1_record_trans = c1_record[count].getOrigin();
    ci = camera_intrinsics[count];
    rows = img.rows; cols = img.cols;

    for (size_t p=0, cont=0; p<cloud.size(); p+=4, cont++) {

        x=cloud[p], y=cloud[p+1], z=cloud[p+2];
        if ( ! projectPoint() ) {
            continue;
        }
        // draw the circle
        //uint32_tToBytes.value = getRGB[ labels[cont] ];
        uint32_tToBytes.value = labels[cont];
        cv::circle(img, cv::Point(px, py), 5,
                   cv::Scalar(uint32_tToBytes.byte[2],uint32_tToBytes.byte[1],uint32_tToBytes.byte[0]),
                   -1,8,0);

    }

    cv::imshow(window_name, img);
    cv::waitKey(1);
}