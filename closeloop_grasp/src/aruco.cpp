/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */
/* Modified by: Fu-Jen Chu, Ruinian Xu*/

#include <ros/ros.h>
#include <ros/duration.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <dynamixel_msgs/JointState.h>

#include <time.h>
#include <unistd.h>
#include <math.h>
#include <unistd.h>
#include <cstdlib> 
#include <iostream>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions> 
#include <tf/tf.h>

// include headers for moveit group
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

void getCommandFromMeta(std::string &command, std::string &x, std::string &y);
void getCommandFromMeta_commandOnly(std::string &command);
void getCommandFromMeta_locationOnly(std::string &x, std::string &y);
std::vector<double> current_joint_values;
bool mutex = 0;
double load_8 = 0;

const double pi = std::acos(-1);
//const double pi = 3.1415926535897;
const double gripper_open_value = -0.7; //open value of 8th motor
const double gripper_close_value = 0.2; //close value of 8th motor
const double paw_torque_compensate = 0.1;
const double height_table = -0.1;

void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg);
void traj_end_mutex(const std_msgs::Bool::ConstPtr& msg);
void update_state(const dynamixel_msgs::JointState::ConstPtr& msg);
void mutex_traj();
void mutex_rotate(double target_angle);
void openGrabber(ros::Publisher &pub_8);
void closeGrabber(ros::Publisher &pub_8);
void rotateGripper(ros::Publisher &pub_7, double angle);
const std::vector<std::string> listNamedPoses(moveit::planning_interface::MoveGroup &group);
void gotoNamedTarget(moveit::planning_interface::MoveGroup &group, std::string target, bool constraint_on, ros::Publisher &pub_8, ros::Publisher &pub_9);
void addObject2Scene(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher);
void getConstraint(moveit::planning_interface::MoveGroup &group, std::string target);
std::vector<std::string> split(const std::string& str, const std::string& delim);
geometry_msgs::Pose move(moveit::planning_interface::MoveGroup &group);
geometry_msgs::Pose read_aruco_tag();
void show_error(geometry_msgs::Pose pose_desired, geometry_msgs::Pose pose_actual);

int main(int argc, char **argv)
{

    /*****************************************************************
    *                         Arm initialization                     *
    *****************************************************************/
    // ros initialization
    ros::init(argc, argv, "arm_kinematics");
    ros::NodeHandle node_handle;  
    ros::Publisher pub_7 = node_handle.advertise<std_msgs::Float64>("/finalarm_position_controller_7/command", 1, true);
    ros::Publisher pub_8 = node_handle.advertise<std_msgs::Float64>("/finalarm_position_controller_8/command", 1, true);
    ros::Publisher pub_9 = node_handle.advertise<std_msgs::Float64>("/finalarm_position_controller_9/command", 1, true);
    ros::Subscriber sub_multi_joint_states = node_handle.subscribe<sensor_msgs::JointState>("/joint_states", 1, joint_state_handler);
    ros::Subscriber sub_endtime = node_handle.subscribe<std_msgs::Bool>("/finalarm_joint_trajectory_action_controller/mutex", 5, traj_end_mutex);
    ros::Subscriber sub_joint_8_state = node_handle.subscribe<dynamixel_msgs::JointState>("/finalarm_position_controller_8/state", 1, update_state);


    ros::AsyncSpinner spinner(1);
    spinner.start();

    // kinematic_state & kinematic_model loading & planner
    moveit::planning_interface::MoveGroup group_arm("arm");
    // moveit::planning_interface::MoveGroup group_actuator("arm4_full");
    group_arm.setPlannerId("BKPIECEkConfigDefault");//ForageRRTkConfigDefault//LBKPIECEkConfigDefault//RRTstarkConfigDefault//BKPIECEkConfigDefault//RRTstarkConfigDefault
    // relationship is as follow
    // robot_state::RobotStatePtr kinematic_state:       group.getCurrentState() 
    // robot_model::RobotModelPtr kinematic_model:       group.getCurrentState()->getRobotModel()
    // robot_state::JointModelGroup* joint_model_group:  group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName())
    
    // We will use the planning_scene_interface class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 

    // clear commands from META
    //std::ofstream myfileOut ("/home/fujenchu/projects/robotArm/socket/commands.txt", std::ios::trunc);
    //myfileOut << ""; myfileOut.close();

    /*****************************************************************
    *                         ROS info output                        *
    *****************************************************************/
    // get the joint names to verify if loaded successfully 
    const std::vector<std::string> &joint_names = group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName())->getJointModelNames();
    //std::cout << "Loading model..  joints in arm model:" << std::endl; 
    //for (size_t i = 0; i < 8 ; i++) std::cout << joint_names[i] << std::endl;

    const std::vector<std::string> &link_names = group_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_arm.getName())->getLinkModelNames();
    //for (size_t i = 0; i < 8 ; i++) std::cout << link_names[i] << std::endl;

    ROS_INFO("Model frame: %s", group_arm.getCurrentState()->getRobotModel()->getModelFrame().c_str());
    ROS_INFO("Reference Planning frame: %s", group_arm.getPlanningFrame().c_str());
    ROS_INFO("EndEffectorLink simulator: %s", group_arm.getEndEffectorLink().c_str());
    //ROS_INFO("EndEffectorLink actuator: %s", group_actuator.getEndEffectorLink().c_str());
    //ROS_INFO("EndEffector: %s", group.getEndEffector().c_str());

    /*****************************************************************
    *                       Visualization setup                      *
    *****************************************************************/
    // for visualization
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    /*****************************************************************
    *                     Adding objects to world                    *
    *****************************************************************/
    /* First put an object into the scene*/
    /* Advertise the collision object message publisher*/
    ros::Publisher collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    while(collision_object_publisher.getNumSubscribers() < 1) {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }
    addObject2Scene(group_arm, planning_scene_interface ,collision_object_publisher);
  
    /*****************************************************************
    *                        List stored poses                       *
    *****************************************************************/
    // list the pre-defined pose in SRDF file
    const std::vector<std::string> namedTargets = listNamedPoses(group_arm);

    /*****************************************************************
    *                      Specify initial pose                      *
    *****************************************************************/
    // namedTargets stores the names of pre-defined poses(joint values)
    // select the name (ex. "home"), gotoNamedTarget function will find plan to the specific pose

    std::string target = ""; 
    int targetNum = 0;
    std::cout<<"select target pose above: "; std::cin >> targetNum;
    if(targetNum == 0) target = "Home";
    else target = "Home";
    gotoNamedTarget(group_arm, target, 0, pub_8, pub_9);

    /*
    std::string command = "";
    std::string pos_x = "";
    std::string pos_y = "";
    std::cout<< "Handy is waiting for command! (you might need to run the server)"<<std::endl;
    */
    
    int flag = 1;
    while (flag) {
    
        /*****************************************************************
        *             Read the eef pose accordint to aruco tag           *
        *****************************************************************/
        //1. Go to the specified position
        //2. Read the eef pose according to aruco tag
        //3. Show the error
        //4. Come back to the home position
        geometry_msgs::Pose pose_desired, pose_actual;
        //1
        pose_desired = move(group_arm);

        //2
        pose_actual = read_aruco_tag();

        //3
        show_error(pose_desired, pose_actual);

        //4
        gotoNamedTarget(group_arm, target, 0, pub_8, pub_9);

        std::cout<<"Would you like to continue? 0.No 1.Yes"<<std::endl;
        std::cin >> flag;
    }

    ros::spin();
    return 0;
}

const std::vector<std::string> listNamedPoses(moveit::planning_interface::MoveGroup &group){
    // list of all stored pose name in SRDF 
    const std::vector<std::string> namedTargets = group.getNamedTargets();
    std::cout<<"stored position in SRDF: "<<std::endl;
    for(int i = 0; i < namedTargets.size(); i++){
        std::cout<<i<<": ";
        std::cout<<namedTargets[i]<<std::endl;
    }

    return namedTargets;
}

void getCommandFromMeta(std::string &command, std::string &x, std::string &y){
    std::ifstream myfileIn ("/home/fujenchu/projects/robotArm/socket/commands.txt");
    getline (myfileIn,command);

    if(command != ""){ // got sth
        getline (myfileIn,x);
        getline (myfileIn,y);
        std::ofstream myfileOut ("/home/fujenchu/projects/robotArm/socket/commands.txt", std::ios::trunc);
        myfileOut << "";
        myfileOut.close();
        std::cout << "receive a command = " << command << std::endl; 
        std::cout<<"x = "<<x<<std::endl;
        std::cout<<"y = "<<y<<std::endl;
    }
    myfileIn.close();
}

void getCommandFromMeta_commandOnly(std::string &command){
    std::ifstream myfileIn ("/home/fujenchu/projects/robotArm/socket/commands.txt");
    getline (myfileIn,command);

    if(command != ""){ // got sth
        std::cout << "receive a command = " << command << std::endl;
        std::ofstream myfileOut ("/home/fujenchu/projects/robotArm/socket/commands.txt", std::ios::trunc);
        myfileOut << "";
        myfileOut.close();
    }
    myfileIn.close();
}

void getCommandFromMeta_locationOnly(std::string &x, std::string &y){
    std::ifstream myfileIn ("/home/fujenchu/projects/robotArm/toy-opencv-mat-socket-server-master_pcl/bbs/locations.txt");

    getline (myfileIn,x);
    getline (myfileIn,y);
    std::cout<<"x = "<<x<<std::endl;
    std::cout<<"y = "<<y<<std::endl;
    
    myfileIn.close();
}

void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg)
{
    current_joint_values.clear();
    for(std::size_t i = 0; i < msg->position.size(); ++i) {
        current_joint_values.push_back(msg->position[i]);
    }
}

void traj_end_mutex(const std_msgs::Bool::ConstPtr& msg){
    if (msg->data){
        mutex = 1;
    }
}

void update_state(const dynamixel_msgs::JointState::ConstPtr& msg){
    //update load for 8th motor
    load_8 = msg->load;
}

void mutex_traj(){
    clock_t start_time = clock();
    while(1){
        if(mutex){
            break;
        }
        else if ((clock() - start_time) / CLOCKS_PER_SEC > 6.0)
            break;
    }
    mutex = 0;
}

void mutex_rotate(double target_angle){
    clock_t start_time = clock();
    while(1){
        if (std::abs(target_angle - current_joint_values[6]) < 0.01)
            break;
        else if ((clock() - start_time) / CLOCKS_PER_SEC > 4.0)
            break;
    }
}

void openGrabber(ros::Publisher &pub_8){
    std_msgs::Float64 val_8;
    val_8.data = gripper_open_value;
    pub_8.publish(val_8);
    sleep(1.5);
}

void rotateGripper(ros::Publisher &pub_7, double angle) {
    std_msgs::Float64 val_7;

    std::cout<<" current value of seventh joint: "<<current_joint_values[6]<<std::endl;
    std::cout<<" compensate value: "<<std::abs(angle - 180.00) / 180.00 * pi<<std::endl;

    if ((current_joint_values[6] + (angle / 90.00 * pi / 2)) > 1.8){
        val_7.data = current_joint_values[6] - (std::abs(angle - 180.00) / 180.00 * pi);
        pub_7.publish(val_7);
        mutex_rotate(val_7.data);
    }
    else if ((current_joint_values[6] + (angle / 90.00 * pi / 2)) < -1.8){
        val_7.data = current_joint_values[6] + (std::abs(angle - 180.00) / 180.00 * pi);
        pub_7.publish(val_7);
        mutex_rotate(val_7.data);
    }
    else{
        val_7.data = current_joint_values[6] + (angle / 90.00 * 1.8);
        pub_7.publish(val_7);
        mutex_rotate(val_7.data);
    } 
}

void closeGrabber(ros::Publisher &pub_8){
    std_msgs::Float64 val_8 ;
    val_8.data = gripper_close_value;
    pub_8.publish(val_8);

    /*
    double sleep_time = std::abs(angle - current_joint_values.back()) / 0.85 * 4.0;
    sleep(sleep_time);
    */
    /*
    while(1){
        if(load_8 > 0.4){
            val_8.data = current_joint_values.back() - 0.1;
            pub_8.publish(val_8);
            break;
        }
    }
    */
    clock_t start_time = clock();
    while(1){
        if(std::abs(load_8) > 0.4){
            val_8.data = current_joint_values[7] + paw_torque_compensate;
            pub_8.publish(val_8);
            break;
        }
        if((double)(clock() - start_time)/CLOCKS_PER_SEC > 5)
        	break;
    }
}

void gotoNamedTarget(moveit::planning_interface::MoveGroup &group, std::string target, bool constraint_on, ros::Publisher &pub_8, ros::Publisher &pub_9){
    ROS_INFO("TASK: Go to %s pose", target.c_str());

    // get joint values of stored pose by name
    std::vector<double> group_variable_values;
    std::map<std::string, double> positions = group.getNamedTargetValues(target);
    for (std::map<std::string, double> ::iterator it=positions.begin(); it!=positions.end(); ++it){
      std::cout << it->first << " => " << it->second << '\n';
      group_variable_values.push_back(it->second);
    }
 
    // get constraints
    if(constraint_on) getConstraint(group, target);

    // plan
    moveit::planning_interface::MoveGroup::Plan my_plan;
    group.setJointValueTarget(group_variable_values);
    bool success = group.plan(my_plan);
    //compensate_slark(my_plan);
    /*
    sleep(5);

    ros::NodeHandle node_handle;
    ros::Publisher pub_8 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_8/command", 1, true);
    closeGrabber(pub_8,gripper_close_value_paw);
    sleep(5);
    */

    // set planning time to default
    group.setPlanningTime(10.0);
    group.clearPathConstraints();

    // visualization
    ROS_INFO("Visualizing plan home (joint space goal) %s",success?"":"FAILED");
    //sleep(5.0);    
 
    // execution
    ROS_INFO("Execution if plan successfully");
    if(success) group.execute(my_plan);
    mutex_traj();

    if (target == "Home"){
    	std_msgs::Float64 val_8;
    	std_msgs::Float64 val_9;
		val_8.data = gripper_open_value;
		val_9.data = -gripper_open_value;
	    pub_8.publish(val_8);
	    pub_9.publish(val_9);
    }
    
    // show current Joints (the real robot joints, not the planned joints)
    std::vector<double> currentJoints = group.getCurrentJointValues();
    /*
    std::cout<< "current joint values:"<<std::endl;
    for(size_t i = 0; i<currentJoints.size(); i++) std::cout<< currentJoints[i]<<" ";
    std::cout<<std::endl;
    */

}

void addObject2Scene(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher){
    /* Define the object message */
    moveit_msgs::CollisionObject object;

    /* The header must contain a valid TF frame */
    object.header.frame_id = group.getPlanningFrame();
    /* The id of the object */
    object.id = "box";

    /* A default pose */
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x =  0.5;
    pose.position.y =  0.5;
    pose.position.z = -0.165;//-0.13;//-0.1;//-0.07

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.9;
    primitive.dimensions[1] = 0.9;
    primitive.dimensions[2] = 0.05;

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);


    /* An attach operation requires an ADD */
    object.operation = object.ADD;

    /* Publish and sleep (to view the visualized results) */
    collision_object_publisher.publish(object);

    std::vector<moveit_msgs::CollisionObject> collision_objects;  
    collision_objects.push_back(object);  

    // Now, let's add the collision object into the world
    ROS_INFO("Add an object into the world");  
    planning_scene_interface.addCollisionObjects(collision_objects);

    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();

}

void getConstraint(moveit::planning_interface::MoveGroup &group, std::string target){
    ROS_INFO("get constraints");

    // switch wont take string
    int targetNum = 0;
    if(target == "home") targetNum = 0;
    else if(target == "holding") targetNum = 1;
    else targetNum = -1;

    // avoid cross init
    moveit_msgs::OrientationConstraint ocm; 
    moveit_msgs::Constraints test_constraints;
    switch(targetNum){
        case 0: // no constraints
          break;

        case 1: // hoding position  
          ocm.link_name = "link_8";  
          ocm.header.frame_id = "base_link";
          ocm.orientation.z = 1.0;
          ocm.absolute_x_axis_tolerance = 0.1; //0.1
          ocm.absolute_y_axis_tolerance = 0.1; //0.1
          ocm.absolute_z_axis_tolerance = 0.5; //0.1
          ocm.weight = 1.0;

          test_constraints.orientation_constraints.push_back(ocm);  
          group.setPathConstraints(test_constraints);
          group.setPlanningTime(10.0);
          break;

        default: // no constraints
          break;
    }
}

std::vector<std::string> split(const std::string& str, const std::string& delim)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}

geometry_msgs::Pose move(moveit::planning_interface::MoveGroup &group){
	/*****************************************************************
    *                          Go somewhere                          *
    *****************************************************************/
    ROS_INFO("Setup pose to go");
    geometry_msgs::Pose pose;
    //position x,y,z got form camera
    double x, y, z;
    std::cout<<"input x (ex. 0.4):"; std::cin>>x;
    std::cout<<"input y (ex. 0.2):"; std::cin>>y;
    std::cout<<"input z (ex. 0.1 or 0.0):"; std::cin>>z;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    double angle;
    std::cout<<"input rotate angle for wrist(ex. 90 degree):"; std::cin>>angle;
    angle = angle / 180.0 * M_PI;
    tf::Quaternion q = tf::createQuaternionFromRPY(0, -M_PI, angle);
    q.normalize();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    // set target pose
    group.setPoseTarget(pose);
    // plan
    moveit::planning_interface::MoveGroup::Plan plan;
    bool success = group.plan(plan);
    //compensate_slark(my_plan); //compensate the slark for 2nd motor
    // visualization
    ROS_INFO("Visualizing plan of appraching object (pose goal) %s",success?"":"FAILED");    
    // execute
    if(success) group.execute(plan);
    mutex_traj();

    return pose;
}

//return the actual eef pose
geometry_msgs::Pose read_aruco_tag(){
	Eigen::Matrix4f H_base_2_aruco;

	//get the homogeneous transformation from base to aruco tag
    std::ifstream file_aruco("/home/ruinianxu/shared_folder/sharedFromWindows/aruco_M_r_wrist.txt");
    std::string line;
    std::string delimiter = " ";
    std::string idx_tag;
    //skip the first two lines
    for(int i=1; i<4; i++){
        getline(file_aruco, line);
        if (i == 2){
            idx_tag = line;
        }
    }
    int row = 0;
    if (file_aruco.is_open()){
        while(getline(file_aruco, line)){
            std::vector<std::string> elements;
            elements = split(line, delimiter);
            float f1, f2, f3, f4;
            f1 = atof(elements[0].c_str());
            f2 = atof(elements[1].c_str());
            f3 = atof(elements[2].c_str());
            f4 = atof(elements[3].c_str());
            H_base_2_aruco(row, 0) = f1;
            H_base_2_aruco(row, 1) = f2;
            H_base_2_aruco(row, 2) = f3;
            H_base_2_aruco(row, 3) = f4;
            row++;
        }
        file_aruco.close();
    }
    
	//transform from aruco tag to link_eef
    Eigen::Matrix4f H_aruco_2_eef;
    if (idx_tag == "3"){
        H_aruco_2_eef << -3.67320510e-06,  -1.00000000e+00,  -9.74400021e-21,           0.0369,
                         -3.67320510e-06,   1.34924357e-11,   1.00000000e+00,   0.00000000e+00,
                         -1.00000000e+00,   3.67320510e-06,  -3.67320510e-06,         -0.13275,
                         0.00000000e+00,   0.00000000e+00,   0.00000000e+00,    1.00000000e+00;
    }
    else if (idx_tag == "4"){
        H_aruco_2_eef << -9.94420009e-01,   7.30541730e-06,   1.05493344e-01,          -0.0001,
                         -1.05493344e-01,   4.44820248e-06,  -9.94420009e-01,         -0.01825,
                         -7.73390890e-06,  -1.00000000e+00,  -3.65270865e-06,         -0.13275,
                          0.00000000e+00,   0.00000000e+00,   0.00000000e+00,   1.00000000e+00;
    }

    //get the homogeneous tf from base to eef
    Eigen::Matrix4f H_base_2_eef;
    H_base_2_eef = H_aruco_2_eef * H_base_2_aruco;

    Eigen::Matrix3f rot_base_2_eef;
    rot_base_2_eef = H_base_2_eef.block(0,0,3,3);
    Eigen::Quaternionf q(rot_base_2_eef);

    geometry_msgs::Pose pose;
    pose.position.x = H_base_2_eef(0,3);
    pose.position.y = H_base_2_eef(1,3);
    pose.position.z = H_base_2_eef(2,3);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

void show_error(geometry_msgs::Pose pose_desired, geometry_msgs::Pose pose_actual){
	double error_x, error_y, error_z;

	error_x = pose_actual.position.x - pose_desired.position.x;
	error_y = pose_actual.position.y - pose_desired.position.y;
	error_z = pose_actual.position.z - pose_desired.position.z;

	std::cout<<"Error for x: "<<error_x<<std::endl;
	std::cout<<"Error for y: "<<error_y<<std::endl;
	std::cout<<"Error for z: "<<error_z<<std::endl;
}