#include <ros/ros.h>

#include <iostream>
#include <fstream>

#include <vector>
#include <string>

#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <time.h>

#include <utils.h>

// MESSAGE HEADERS
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>

class gestureManager{
    public:
        gestureManager(ros::NodeHandle nh);
        ~gestureManager();

        // CALLBACKS
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
        void gestureCommandCallback(const std_msgs::String::ConstPtr& msg);

        // FUNCTIONS
        void moveArms(std::string gesture);

        // PUBLISHERS
        ros::Publisher right_arm_command_pub;
        ros::Publisher left_arm_command_pub;

        // SUBSCRIBERS
        ros::Subscriber laser_sub;
        ros::Subscriber link_states_sub;
        ros::Subscriber gesture_command_sub;

        // MESSAGES
        geometry_msgs::Pose right_arm_pose_msg;
        geometry_msgs::Pose left_arm_pose_msg;
        
        Eigen::Quaterniond right_EE_link_quat_shoulder; // Orientation of EE w.r.t. shoulder
        Eigen::Vector3d right_EE_link_pos_shoulder;     // Position of EE w.r.t. shoulder
};