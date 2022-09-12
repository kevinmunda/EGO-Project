#include <ros/ros.h>
#include <ego_msgs/Wheel_Torque.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv){
    // Initialize ROS system and become a node
    ros::init(argc, argv, "move_wheels");
    ros::NodeHandle n;

    // Publisher for wheel torques
    ros::Publisher left_wheel_torque_pub = n.advertise<std_msgs::Float64>(
        "ego/left_wheel_controller/command", 1);
    
    ros::Rate rate(1);

    while(ros::ok()){
        std_msgs::Float64 msg;
        msg.data = float(11);
        
        left_wheel_torque_pub.publish(msg);
        
        ROS_INFO_STREAM(
            "Torque commands: \n" <<
            "Left wheel torque: " << msg.data << "\n"
        );

        rate.sleep();
    }
}