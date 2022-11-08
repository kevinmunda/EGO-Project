#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>

#include <typeinfo>
#include <vector>
#include <string>

#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <time.h>

#include "json.hpp"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// VARIABLES
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct gestureType {
    Eigen::MatrixXd right_arm_poses;
    Eigen::MatrixXd left_arm_poses;
    std::vector<int> executionTimes;
    bool repeat;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Splits a string given a separator
inline std::vector<std::string> splitString(std::string strData, char separator){
    std::vector<std::string> outputArray;
    std::stringstream streamData(strData);
    std::string val;
    while(std::getline(streamData, val, separator))
        outputArray.push_back(val);
    return outputArray;
}

// Computes interpolation between a and b given a factor f
inline float lerp(float a, float b, float f){
    return a + f*(b-a);
}

// Computes interpolation between arm poses, given a range
inline std::vector<double> interpolatePoses(Eigen::MatrixXd arm_poses, int range, int rows, int cols){
    std::vector<double> poses_interpolated;
    float tmp;
    int i, j, n;
    for(i = 0; i < cols; i++){
        if(i == 0){
            for(j = 0; j < rows; j++)
                poses_interpolated.push_back(arm_poses(j, 0));
        }
        else{
            for(n = 1; n <= range; n++){
                float f = (float)n/(float)range;
                for(j = 0; j < rows; j++){
                    tmp = lerp(arm_poses(j, i-1), arm_poses(j, i), f);
                    poses_interpolated.push_back(tmp);
                }       
            }
        }
    }
    return poses_interpolated;
}

// Read the JSON file with the information about the gesture to execute and interpolate poses
inline gestureType readJson(std::string gesture_name){
    std::string gesture_path;
    gestureType gesture;
    std::vector<double> tmp_right, tmp_left;
    int count = 0;
    int rows = 7;
    int range = 3;

    // Select the right file
    if(gesture_name == "greeting")
        gesture_path = "/home/kevin/catkin_ws/src/project_ego/ego_poses/" + gesture_name + ".json";
    else{
        ROS_ERROR("Command not found");
        exit(1);
    }
    
    // Transfer data from JSON file to variables
    std::fstream file(gesture_path);
    auto json = nlohmann::json::parse(file);
    file.close();

    for(auto phase: json["phases"]){
        count++;
        for(auto tmp_coord: phase["rightArmPose"]){
            double val = (double)tmp_coord;
            tmp_right.push_back(val);
        }
        for(auto tmp_coord: phase["leftArmPose"]){
            double val = (double)tmp_coord;
            tmp_left.push_back(val);
        }
        gesture.executionTimes.push_back((int)phase["executionTime"]);
    }
    
    int cols = count;
    gesture.right_arm_poses = Eigen::Map<Eigen::MatrixXd>(&tmp_right[0], rows, cols);
    gesture.left_arm_poses = Eigen::Map<Eigen::MatrixXd>(&tmp_left[0], rows, cols);

    tmp_right = interpolatePoses(gesture.right_arm_poses, range, rows, cols);
    tmp_left = interpolatePoses(gesture.left_arm_poses, range, rows, cols);
    
    int cols_interpolated = (cols - 1) * range + 1;
    gesture.right_arm_poses = Eigen::Map<Eigen::MatrixXd>(&tmp_right[0], rows, cols_interpolated);
    gesture.left_arm_poses = Eigen::Map<Eigen::MatrixXd>(&tmp_left[0], rows, cols_interpolated);
    
    gesture.repeat = (bool)(int)json["repeat"];
    
    return gesture;
}

#endif