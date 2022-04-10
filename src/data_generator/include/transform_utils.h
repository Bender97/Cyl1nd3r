//
// Created by fusy on 10/04/22.
//

#ifndef DATA_GENERATOR_TRANSFORM_UTILS_H
#define DATA_GENERATOR_TRANSFORM_UTILS_H

#include <tf/transform_datatypes.h>

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

#endif //DATA_GENERATOR_TRANSFORM_UTILS_H
