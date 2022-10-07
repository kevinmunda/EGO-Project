#include <EgoClass.h>

EgoClass::EgoClass(ros::NodeHandle nh){
    // VARIABLES
    forwardVelocity = 0.0;
    yawRate = 0.0;

    // PUBLISHERS
    segway_des_vel_pub = nh.advertise<ego_msgs::EgoTwist2DUnicycle>(
        "/segway_des_vel", 10);

    // SUBSCRIBERS
    laser_sub = nh.subscribe("/scan", 10, &EgoClass::laserCallback, this);
}

EgoClass::~EgoClass(){
  ROS_INFO_STREAM("Egoclass deleted");
}

void EgoClass::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    //ROS_INFO("LaserScan size: %d", ranges.size());
    std::vector<float> ranges = msg -> ranges; 
    for(float scan: ranges){
        if(scan < 1.0){
            forwardVelocity = 0.0;
            yawRate = 0.0;
        }
    }
}

void EgoClass::publishTwist(){
    twist_msg.ForwardVelocity = forwardVelocity;
    twist_msg.YawRate = yawRate;
    segway_des_vel_pub.publish(twist_msg);
}
