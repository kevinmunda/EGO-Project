#include <ros/ros.h>
#include <typeinfo>
#include <iostream>
#include <vector>

#include <ego_msgs/EgoTwist2DUnicycle.h>
#include <sensor_msgs/LaserScan.h>


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    //ROS_INFO("LaserScan size: %d", ranges.size());
    std::vector<float> ranges = msg -> ranges; 
    
    for(float scan: ranges){
        if(scan < 1.0){
            //forwardVelocity = 0.0;
            //yawRate = 0.0;
        }
    }
}


int main(int argc, char **argv){
    // Initialize ROS system and create the node handle
    ros::init(argc, argv, "move_ego");
    ros::NodeHandle nh("~");

    // Variables to get parameters from command line
    float forwardVelocity = 0.0;
    float yawRate = 0.0;

    // Publisher for desired robot velocity
    ros::Publisher segway_des_vel_pub = nh.advertise<ego_msgs::EgoTwist2DUnicycle>(
        "/segway_des_vel", 10);

    // Subscribers
    ros::Subscriber laser_sub = nh.subscribe("/scan", 10, laserCallback); 
    
    // Define a rate object that will keep track of how long it has been since the last call to Rate::sleep()
    ros::Rate rate(100);

    // Messages
    ego_msgs::EgoTwist2DUnicycle msg;

    // Parametri da passare allo script da terminale
    // e.g.: _forwardVelocity:=0.5
    nh.getParam("forwardVelocity", forwardVelocity);
    nh.getParam("yawRate", yawRate);

    ROS_INFO("Got Forward Velocity: %f", forwardVelocity);
    ROS_INFO("Got Yaw Rate: %f", yawRate);

    while(ros::ok()){
        msg.ForwardVelocity = forwardVelocity;
        msg.YawRate = yawRate;
        
        // Publish message to EGO
        segway_des_vel_pub.publish(msg);

        ros::spinOnce();
        
        rate.sleep();
    }
}