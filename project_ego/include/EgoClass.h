#include <ros/ros.h>
#include <typeinfo>
#include <iostream>
#include <vector>

// MESSAGE HEADERS
#include <ego_msgs/EgoTwist2DUnicycle.h>
#include <sensor_msgs/LaserScan.h>

class EgoClass{
    public:
        EgoClass(ros::NodeHandle nh);
        ~EgoClass();

        // CALLBACKS
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

        // FUNCTIONS
        void publishTwist();

        // PUBLISHERS
        ros::Publisher segway_des_vel_pub;

        // SUBSCRIBERS
        ros::Subscriber laser_sub;

        // MESSAGES
        ego_msgs::EgoTwist2DUnicycle twist_msg;

        // VARIABLES
        float forwardVelocity;
        float yawRate;
};