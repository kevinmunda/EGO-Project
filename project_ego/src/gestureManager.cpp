#include <gestureManager.h>

// Constructor
gestureManager::gestureManager(ros::NodeHandle nh){
    // PUBLISHERS
    right_arm_command_pub = nh.advertise<geometry_msgs::Pose>("/right_arm/command1", 100);
    left_arm_command_pub = nh.advertise<geometry_msgs::Pose>("/left_arm/command1", 100);

    // SUBSCRIBERS
    //laser_sub = nh.subscribe("/scan", 10, &gestureManager::laserCallback, this);
    //link_states_sub = nh.subscribe("/gazebo/link_states", 10, &gestureManager::linkStatesCallback, this);
    gesture_command_sub = nh.subscribe("/gesture_command", 10, &gestureManager::gestureCommandCallback, this);

    // VARIABLES
    gestures = {"greeting", "stop"};
}

// Destructor
gestureManager::~gestureManager(){
  ROS_INFO_STREAM("Egoclass deleted");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CALLBACKS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
void gestureManager::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    //ROS_INFO("LaserScan size: %d", ranges.size());
    std::vector<float> ranges = msg -> ranges; 
    for(float scan: ranges){
        if(scan < 1.0){
            forwardVelocity = 0.0;
            yawRate = 0.0;
        }
    }
}
*/

/*
void gestureManager::linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){
    // Names and poses of the links in the scene
    std::vector<std::string> link_names = msg -> name;
    std::vector<geometry_msgs::Pose> link_poses = msg -> pose;
    
    // Retrieve info about link's orientation 
    std::vector<std::string>::iterator it = std::find(link_names.begin(), link_names.end(), "ego_robot::right_flange_EE");
    int index = std::distance(link_names.begin(), it);
    // Obtain the orientation of the link w.r.t. gazebo world RF
    Eigen::Quaterniond right_EE_link_quat(link_poses[index].orientation.w, link_poses[index].orientation.x, 
                                        link_poses[index].orientation.y, link_poses[index].orientation.z);

    Eigen::Quaterniond right_EE_link_pos(0, link_poses[index].position.x, 
                                        link_poses[index].position.y, link_poses[index].position.z);

    right_EE_link_quat.normalize();
    // Orientation of EGO shoulder RF w.r.t. gazebo world RF at rest
    Eigen::Quaterniond right_shoulder_link_quat(0.762, 0.641, 0.04, 0.07);
    right_shoulder_link_quat.normalize();
    // Get the orientation of EGO EE RF w.r.t. EGO shoulder RF
    right_EE_link_quat_shoulder = right_shoulder_link_quat.inverse() * right_EE_link_quat * right_shoulder_link_quat;
    Eigen::Quaterniond right_EE_link_pos_shoulder_tmp = right_shoulder_link_quat.inverse() * right_EE_link_pos * right_shoulder_link_quat;
    right_EE_link_pos_shoulder = right_EE_link_pos_shoulder_tmp.vec();
    //std::cout << "Vector: " << right_EE_link_quat_shoulder.vec() << std::endl << "Real part: " << right_EE_link_quat_shoulder.w() << std::endl;
    //std::cout << "Position Vector: " << right_EE_link_pos_shoulder << std::endl;
}
*/

void gestureManager::gestureCommandCallback(const std_msgs::String::ConstPtr& msg){
    gesture_received = msg -> data;
    moveArms(gesture_received);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void gestureManager::moveArms(std::string gesture_name){
    //Variables
    gestureType gesture;
    bool phaseConcluded = false;
    time_t start;
    int seconds;
    
    if(std::find(gestures.begin(), gestures.end(), gesture_name) == gestures.end()){
        ROS_ERROR("GESTURE NOT FOUND");
        return;
    }

    gesture = readJson(gesture_name);
    
    do{
        for(int i = 0; i < gesture.right_arm_poses.cols(); i++){
            phaseConcluded = false;
            right_arm_pose_msg.position.x = gesture.right_arm_poses(0, i);
            right_arm_pose_msg.position.y = gesture.right_arm_poses(1, i);
            right_arm_pose_msg.position.z = gesture.right_arm_poses(2, i);
            right_arm_pose_msg.orientation.x = gesture.right_arm_poses(3, i);
            right_arm_pose_msg.orientation.y = gesture.right_arm_poses(4, i);
            right_arm_pose_msg.orientation.z = gesture.right_arm_poses(5, i);
            right_arm_pose_msg.orientation.w = gesture.right_arm_poses(6, i);

            left_arm_pose_msg.position.x = gesture.left_arm_poses(0, i);
            left_arm_pose_msg.position.y = gesture.left_arm_poses(1, i);
            left_arm_pose_msg.position.z = gesture.left_arm_poses(2, i);
            left_arm_pose_msg.orientation.x = gesture.left_arm_poses(3, i);
            left_arm_pose_msg.orientation.y = gesture.left_arm_poses(4, i);
            left_arm_pose_msg.orientation.z = gesture.left_arm_poses(5, i);
            left_arm_pose_msg.orientation.w = gesture.left_arm_poses(6, i);
            
            seconds = gesture.executionTimes[i];

            time(&start);
            while(!phaseConcluded && ros::ok()){
                ros::spinOnce();
                if(gesture_name != "stop" && gesture_received == "stop")
                    return;
                right_arm_command_pub.publish(right_arm_pose_msg);
                left_arm_command_pub.publish(left_arm_pose_msg);
                if(time(0)-start >= seconds){
                    phaseConcluded = true;
                    //std::cout << time(0)-start << std::endl;
                }
            }
        }
    } while(gesture.repeat && ros::ok());

    /*
    float tolerance = 0.05;
    
    while(!phaseConcluded && ros::ok()){
        // Need to call the callback linkStatesCallback to obtain current value of orientation of EE
        right_arm_command_pub.publish(right_arm_pose_msg);
        ros::spinOnce();
        sleep(0.5);
        count = 0;
        if(fabs(right_arm_pose_msg.orientation.x) >= 0.1 && fabs(right_arm_pose_msg.orientation.x) - fabs(right_EE_link_quat_shoulder.x()) <= tolerance)
            count++;
        if(fabs(right_arm_pose_msg.orientation.y) >= 0.1 && fabs(right_arm_pose_msg.orientation.y) - fabs(right_EE_link_quat_shoulder.y()) <= tolerance)
            count++;
        if(fabs(right_arm_pose_msg.orientation.z) >= 0.1 && fabs(right_arm_pose_msg.orientation.z) - fabs(right_EE_link_quat_shoulder.z()) <= tolerance)
            count++;
        if(fabs(right_arm_pose_msg.orientation.w) >= 0.1 && fabs(right_arm_pose_msg.orientation.w) - fabs(right_EE_link_quat_shoulder.w()) <= tolerance)
            count++; 
        if(count >= 2){
            phaseConcluded = true;
        sleep(1);
        }
    }
    */
}