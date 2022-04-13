//
// Created by fusy on 12/04/22.
//

#ifndef DATA_GENERATOR_CLOUDINTEGRATOR_H
#define DATA_GENERATOR_CLOUDINTEGRATOR_H

#include <iostream>
#include <vector>
#include <tf/LinearMath/Transform.h>

class CloudIntegrator {
public:

    std::vector<float>      cloud_stack;
    std::vector<uint32_t>   labels_stack;
    std::vector<int>        clouds_end;
    const int max_points_per_cloud = 40000;
    int clouds_to_stack;

    CloudIntegrator(const int _clouds_to_stack);

    void stack_a_cloud(std::vector<float> &cloud, std::vector<uint32_t> &labels, tf::Transform &prev_pose, tf::Transform &curr_pose);


};


#endif //DATA_GENERATOR_CLOUDINTEGRATOR_H
