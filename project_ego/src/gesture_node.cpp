#include <gestureManager.h>

int main(int argc, char **argv){
    // Initialize ROS system and create the node handle
    ros::init(argc, argv, "gesture_node");
    ros::NodeHandle nh("~");

    // Define an object of the class gestureManager
    gestureManager gm(nh);
    
    // Define a rate object that will keep track of how long it has been since the last call to Rate::sleep()
    ros::Rate rate(30);
    
    while(ros::ok()){
        
        ros::spinOnce();
        
        rate.sleep();
    }
}