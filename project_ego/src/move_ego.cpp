#include <ros/ros.h>
#include <ego_msgs/EgoTwist2DUnicycle.h>

int main(int argc, char **argv){
    // Initialize ROS system and create the node handle
    ros::init(argc, argv, "move_ego");
    ros::NodeHandle n;

    // Publisher for wheel torques
    ros::Publisher segway_des_vel_pub = n.advertise<ego_msgs::EgoTwist2DUnicycle>(
        "/segway_des_vel", 10);
    
    // Define a rate object that will keep track of how long it has been since the last call to Rate::sleep()
    ros::Rate rate(10);

    ego_msgs::EgoTwist2DUnicycle msg;

    while(ros::ok()){
        msg.ForwardVelocity = 0.5;
        msg.YawRate = 0.1;

        //ROS_INFO_STREAM("Forward Velocity: " << msg.ForwardVelocity);
        //ROS_INFO_STREAM("Yaw Rate: " << msg.YawRate);
        
        segway_des_vel_pub.publish(msg);

        rate.sleep();
    }
}