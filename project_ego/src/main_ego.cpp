#include <EgoClass.h>

int main(int argc, char **argv){
    // Initialize ROS system and create the node handle
    ros::init(argc, argv, "my_ego");
    ros::NodeHandle nh("~");

    // Define an object of the class EgoClass
    EgoClass ego(nh);
    
    // Define a rate object that will keep track of how long it has been since the last call to Rate::sleep()
    ros::Rate rate(30);
    
    // Parametri da passare allo script da terminale
    // e.g.: _forwardVelocity:=0.5
    nh.getParam("forwardVelocity", ego.forwardVelocity);
    nh.getParam("yawRate", ego.yawRate);

    //ROS_INFO("Got Forward Velocity: %f", ego.forwardVelocity);
    //ROS_INFO("Got Yaw Rate: %f", ego.yawRate);

    while(ros::ok()){
        
        //ego.moveArms("greeting");
        ros::spinOnce();
        
        rate.sleep();
    }
}