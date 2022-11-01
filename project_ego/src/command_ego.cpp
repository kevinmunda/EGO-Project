#include <ros/ros.h>
#include <iostream>

int main(int argc, char **argv){
    // Initialize ROS system and create the node handle
    ros::init(argc, argv, "command_ego");
    ros::NodeHandle nh("~");

    // Variables
    std::string command;

    // Define a rate object that will keep track of how long it has been since the last call to Rate::sleep()
    ros::Rate rate(100);

    // Messages
    
    while(ros::ok()){
        std::cout << "Insert the command for EGO: \n" << std::endl;
        std::cin >> command;
        std::cout << "You selected: " << command << std::endl;
        std::cout << "Press CTRL+C to select another command" << std::endl;

        rate.sleep();
    }
}