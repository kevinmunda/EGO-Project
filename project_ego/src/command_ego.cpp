#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>

int main(int argc, char **argv){
    // Initialize ROS system and create the node handle
    ros::init(argc, argv, "command_ego");
    ros::NodeHandle nh("~");

    // Variables
    std::string command;

    // Define a rate object that will keep track of how long it has been since the last call to Rate::sleep()
    ros::Rate rate(100);

    // Messages
    std_msgs::String gesture_command;

    // Publishers
    ros::Publisher gesture_command_pub = nh.advertise<std_msgs::String>("/gesture_command", 10);

    while(ros::ok()){
        std::cout << "To exit write 'stop' " << std::endl;
        std::cout << "Insert the command for EGO: \n" << std::endl;
        std::cin >> command;
        if(command == "stop")
            exit(1);
        else{
            std::cout << "You selected: " << command << std::endl;
            gesture_command.data = command;
            gesture_command_pub.publish(gesture_command);
        }

        rate.sleep();
    }
}