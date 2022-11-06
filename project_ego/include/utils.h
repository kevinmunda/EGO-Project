#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>

#include <vector>
#include <string>

#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <time.h>

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

#endif