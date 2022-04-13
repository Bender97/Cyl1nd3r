//
// Created by fusy on 12/04/22.
//

#include "CloudIntegrator.h"


CloudIntegrator::CloudIntegrator(const int _clouds_to_stack) {
    clouds_to_stack = _clouds_to_stack;

    std::cout << "Setting clouds_to_stack: " << clouds_to_stack << std::endl;

    cloud_stack.resize((clouds_to_stack * max_points_per_cloud) << 2);
    labels_stack.resize(clouds_to_stack * max_points_per_cloud);
    clouds_end.resize(clouds_to_stack, -1);
}

void CloudIntegrator::stack_a_cloud(std::vector<float> &cloud, std::vector<uint32_t> &labels, tf::Transform &prev_pose, tf::Transform &curr_pose) {

    tf::Vector3 trans;
    tf::Matrix3x3 rot;
    float x, y, z;

    // first insertion: just copy first cloud and update clouds_end[-1]
    if (clouds_end[clouds_to_stack-1] == -1) {
        for (size_t i=0; i<cloud.size(); i++)   cloud_stack[i]  = cloud[i];
        for (size_t i=0; i<labels.size(); i++)  labels_stack[i] = cloud[i];
        clouds_end[clouds_to_stack-1] = cloud.size();
    }
    else {    // not the first insertion
        // 1. Crop the cloud_stack by deleting old points

        int offset;
        for(offset=0;clouds_end[offset]<0; offset++);
        offset = clouds_end[offset];

        //if (clouds_end[0]>0) { //this means we have stacked at least clouds_to_stack clouds
            for (size_t a=0, b=offset; b<clouds_end[clouds_to_stack-1]; a++, b++) {
                cloud_stack[a] = cloud_stack[b];
            }

            int labels_start = offset>>2;
            int labels_end   = clouds_end[clouds_to_stack-1] >> 2;

            for (size_t a=0, b=labels_start; b< labels_end ; a++, b++) {
                labels_stack[a] = labels_stack[b];
            }
        //}

        // 4. Shift old clouds_end
        offset = clouds_end[0];
        for (size_t i=0; i<clouds_end.size()-1; i++) clouds_end[i] = clouds_end[i+1] - offset;

        // 2. Transform to new pose the old_points
        trans   = prev_pose.getOrigin();
        rot     = prev_pose.getBasis();

        for (size_t i=0; i<clouds_end[clouds_to_stack-2]; i+=4) {
            x = cloud_stack[i]; y = cloud_stack[i+1]; z = cloud_stack[i+2];
            cloud_stack[ i ] = rot[0][0]*x + rot[0][1]*y + rot[0][2]*z + trans[0];
            cloud_stack[i+1] = rot[1][0]*x + rot[1][1]*y + rot[1][2]*z + trans[1];
            cloud_stack[i+2] = rot[2][0]*x + rot[2][1]*y + rot[2][2]*z + trans[2];
        }

        trans   = -curr_pose.getOrigin();
        rot     =  curr_pose.getBasis().transpose();

        for (size_t i=0; i<clouds_end[clouds_to_stack-2]; i+=4) {
            x = cloud_stack[i] + trans[0]; y = cloud_stack[i+1] + trans[1]; z = cloud_stack[i+2] + trans[2];
            cloud_stack[ i ] = rot[0][0]*x + rot[0][1]*y + rot[0][2]*z;
            cloud_stack[i+1] = rot[1][0]*x + rot[1][1]*y + rot[1][2]*z;
            cloud_stack[i+2] = rot[2][0]*x + rot[2][1]*y + rot[2][2]*z;
        }

        // 3. Add new cloud's points
        for (size_t a=0, b=clouds_end[clouds_to_stack-2]; a<cloud.size(); a++, b++) {
            cloud_stack[b] = cloud[a];
        }
        for (size_t a=0, b=(clouds_end[clouds_to_stack-2] >> 2); a<labels.size(); a++, b++) {
            labels_stack[b] = labels[a];
        }


        // 5. Update the last clouds_end
        clouds_end[clouds_to_stack-1] = clouds_end[clouds_to_stack-2] + cloud.size();

    }
}