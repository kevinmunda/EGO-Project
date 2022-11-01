#include <ros/ros.h>

#include <iostream>
#include <fstream>

#include <typeinfo>
#include <vector>
#include <string>

#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <math.h>

// MESSAGE HEADERS
#include <ego_msgs/EgoTwist2DUnicycle.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>

class EgoClass{
    public:
        EgoClass(ros::NodeHandle nh);
        ~EgoClass();

        // CALLBACKS
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);

        // FUNCTIONS
        std::vector<std::string> splitString(std::string strData, char separator);
        void publishTwist();
        void moveArms(std::string gesture);

        // PUBLISHERS
        ros::Publisher segway_des_vel_pub;
        ros::Publisher right_arm_command_pub;
        ros::Publisher left_arm_command_pub;

        // SUBSCRIBERS
        ros::Subscriber laser_sub;
        ros::Subscriber link_states_sub;

        // MESSAGES
        ego_msgs::EgoTwist2DUnicycle twist_msg;
        geometry_msgs::Pose right_arm_pose_msg;
        geometry_msgs::Pose left_arm_pose_msg;
        
        // VARIABLES
        float forwardVelocity;  // Linear velocity
        float yawRate;          // Angular velocity
        
        Eigen::Quaterniond right_EE_link_quat_shoulder; // Orientation of EE w.r.t. shoulder
};