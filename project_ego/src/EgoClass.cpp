#include <EgoClass.h>

// Constructor
EgoClass::EgoClass(ros::NodeHandle nh){
    // VARIABLES
    forwardVelocity = 0.0;
    yawRate = 0.0;

    // PUBLISHERS
    segway_des_vel_pub = nh.advertise<ego_msgs::EgoTwist2DUnicycle>("/segway_des_vel", 10);
    right_arm_command_pub = nh.advertise<geometry_msgs::Pose>("/right_arm/command1", 100);
    left_arm_command_pub = nh.advertise<geometry_msgs::Pose>("/left_arm/command1", 100);

    // SUBSCRIBERS
    laser_sub = nh.subscribe("/scan", 10, &EgoClass::laserCallback, this);
    link_states_sub = nh.subscribe("/gazebo/link_states", 10, &EgoClass::linkStatesCallback, this);
    gesture_command_sub = nh.subscribe("/gesture_command", 10, &EgoClass::gestureCommandCallback, this);
}

// Destructor
EgoClass::~EgoClass(){
  ROS_INFO_STREAM("Egoclass deleted");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CALLBACKS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

void EgoClass::linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){
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

void EgoClass::gestureCommandCallback(const std_msgs::String::ConstPtr& msg){
    std::string gesture = msg -> data;
    moveArms(gesture);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void EgoClass::publishTwist(){
    twist_msg.ForwardVelocity = forwardVelocity;
    twist_msg.YawRate = yawRate;
    segway_des_vel_pub.publish(twist_msg);
}

void EgoClass::moveArms(std::string gesture){
    //Variables
    std::string gesture_path;
    std::string row;
    
    Eigen::MatrixXd right_arm_poses;
    bool phaseConcluded = false;

    float tolerance = 0.05;
    
    if(gesture == "greeting")
        gesture_path = "/home/kevin/catkin_ws/src/project_ego/ego_poses/" + gesture + ".txt";
    else{
        ROS_ERROR("Command not found");
        exit(1);
    }
    
    std::fstream file(gesture_path);
    int count = 0;
    std::vector<double> tmp_d;
    while(getline(file, row)){
        std::vector<std::string> tmp_str = splitString(row, ' ');
        for(std::string str: tmp_str)
            tmp_d.push_back(std::stod(str));
        count++;
    }
    file.close();

    // Eigen::Map maps the array to a COLUMN-MAJOR matrix
    int rows = 7;
    int cols = count;
    right_arm_poses = Eigen::Map<Eigen::MatrixXd>(&tmp_d[0], rows, cols);

    int i = 0;
    int range = 3;
    float tmp;
    std::vector<double> tmp_interpolated;
    
    for(i = 0; i < cols; i++){
        if(i == 0){
            tmp_interpolated.push_back(right_arm_poses(0, 0));
            tmp_interpolated.push_back(right_arm_poses(1, 0));
            tmp_interpolated.push_back(right_arm_poses(2, 0));
            tmp_interpolated.push_back(right_arm_poses(3, 0));
            tmp_interpolated.push_back(right_arm_poses(4, 0));
            tmp_interpolated.push_back(right_arm_poses(5, 0));
            tmp_interpolated.push_back(right_arm_poses(6, 0));
        }
        else{
            for(int n = 1; n <= range; n++){
                float f = (float)n/(float)range;
                tmp = lerp(right_arm_poses(0, i-1), right_arm_poses(0, i), f);
                tmp_interpolated.push_back(tmp);
                tmp = lerp(right_arm_poses(1, i-1), right_arm_poses(1, i), f);
                tmp_interpolated.push_back(tmp);
                tmp = lerp(right_arm_poses(2, i-1), right_arm_poses(2, i), f);
                tmp_interpolated.push_back(tmp);
                tmp = lerp(right_arm_poses(3, i-1), right_arm_poses(3, i), f);
                tmp_interpolated.push_back(tmp);
                tmp = lerp(right_arm_poses(4, i-1), right_arm_poses(4, i), f);
                tmp_interpolated.push_back(tmp);
                tmp = lerp(right_arm_poses(5, i-1), right_arm_poses(5, i), f);
                tmp_interpolated.push_back(tmp);
                tmp = lerp(right_arm_poses(6, i-1), right_arm_poses(6, i), f);
                tmp_interpolated.push_back(tmp);       
            }
        }
    }

    int cols_interpolated = (cols - 1) * range + 1;
    right_arm_poses = Eigen::Map<Eigen::MatrixXd>(&tmp_interpolated[0], rows, cols_interpolated);
    //std::cout << right_arm_poses << std::endl;
    
    time_t start;
    int seconds = 5;
    // Start execution of movement
    for(int i = 0; i < cols_interpolated; i++){
        phaseConcluded = false;
        right_arm_pose_msg.position.x = right_arm_poses(0, i);
        right_arm_pose_msg.position.y = right_arm_poses(1, i);
        right_arm_pose_msg.position.z = right_arm_poses(2, i);
        right_arm_pose_msg.orientation.x = right_arm_poses(3, i);
        right_arm_pose_msg.orientation.y = right_arm_poses(4, i);
        right_arm_pose_msg.orientation.z = right_arm_poses(5, i);
        right_arm_pose_msg.orientation.w = right_arm_poses(6, i);

        /*
        time(&start);
        while(!phaseConcluded && ros::ok()){
            right_arm_command_pub.publish(right_arm_pose_msg);
            if(time(0)-start >= seconds){
                phaseConcluded = true;
                std::cout << time(0)-start << std::endl;
            }
        */
        
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
    }
    std::cout << "FINE" << std::endl;
}