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
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <dynamixel_msgs/JointState.h>
#include <dynamixel_msgs/PositionVelocity.h>

#include <time.h>
#include <chrono>
#include <unistd.h>
#include <math.h>
#include <unistd.h>
#include <cstdlib> 
#include <iostream>
#include <cmath>
#include <iostream>
#include <fstream>
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

std::vector<double> current_joint_values;
std::vector<double> detection_result;
bool mutex = 0;

const double gripper_open_value = -0.7; //open value of 8th motor
const double gripper_close_value = 0.3; //close value of 8th motor

const double gripper_giant_close_value = 0.0;
const double gripper_slight_close_value = 0.1;
const double gripper_mild_close_value = 0.2;
const double gripper_hard_close_value = 0.3;
const double gripper_extreme_close_value = 0.4;

void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg);
void detection_handler(const std_msgs::Float64MultiArray::ConstPtr& msg);
void traj_end_mutex(const std_msgs::Bool::ConstPtr& msg);
void update_state(const dynamixel_msgs::JointState::ConstPtr& msg);
void mutex_traj();
void mutex_rotate(double target_angle);
void openGrabber(ros::Publisher &pub_8, ros::Publisher &pub_9);
void closeGrabber(ros::Publisher &pub_8, ros::Publisher &pub_9);
void rotateGripper(ros::Publisher &pub_7, double angle);
void place(moveit::planning_interface::MoveGroup &group, double x, double y, double z, ros::Publisher &pub_8, ros::Publisher &pub_9, bool time_count);
const std::vector<std::string> listNamedPoses(moveit::planning_interface::MoveGroup &group);
void gotoNamedTarget(moveit::planning_interface::MoveGroup &group, std::string target, bool constraint_on);
void addObject2Scene(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher);
void getConstraint(moveit::planning_interface::MoveGroup &group, std::string target);

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
    ros::Subscriber sub_detection = node_handle.subscribe<std_msgs::Float64MultiArray>("/result", 1, detection_handler);


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
    // addObject2Scene(group_arm, planning_scene_interface ,collision_object_publisher);
  
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

    //set 8th and 9th motor to zero to avoid collision
    std_msgs::Float64 val_8;
    std_msgs::Float64 val_9;
    val_8.data = gripper_open_value;
    val_9.data = -gripper_open_value;
    pub_8.publish(val_8);
    pub_9.publish(val_9);

    gotoNamedTarget(group_arm, target, 0);

    /*
    std::string command = "";
    std::string pos_x = "";
    std::string pos_y = "";
    std::cout<< "Handy is waiting for command! (you might need to run the server)"<<std::endl;
    */

    bool time_count = false;
    // std::cout<<"Would you like to count the time for pick and place? Type:0(No) 1(Yes)"<<std::endl;
    // std::cin>>time_count;
    
    while (1) {
    
        /*****************************************************************
        *                 Attempt to pick up and place                   *
        *****************************************************************/
        // specify the target_pose
        // the robot will attempt to:
        // 1. go to the place to place object
        // 2. open gripper
        // 3. come back to home position

        if (detection_result.size() != 0){
            double x, y, z;
            x = detection_result[0];
            y = detection_result[1];
            z = detection_result[2];
        
            closeGrabber(pub_8, pub_9);

            place(group_arm, x, y, z, pub_8, pub_9, time_count);

            gotoNamedTarget(group_arm, target, 0);

            break;
        }
        
    }

    ros::spin();
    return 0;
}

void detection_handler(const std_msgs::Float64MultiArray::ConstPtr& msg){
    detection_result.clear();
    for (std::size_t i = 0; i < msg->data.size(); ++i) {
        detection_result.push_back(msg->data[i]);
    }
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

void openGrabber(ros::Publisher &pub_8, ros::Publisher &pub_9){
    std_msgs::Float64 val_8;
    std_msgs::Float64 val_9;
    val_8.data = gripper_open_value;
    val_9.data = -gripper_open_value;
    pub_8.publish(val_8);
    pub_9.publish(val_9);
    sleep(1.5);
}

void rotateGripper(ros::Publisher &pub_7, double angle) {
    std_msgs::Float64 val_7;

    std::cout<<" current value of seventh joint: "<<current_joint_values[6]<<std::endl;
    std::cout<<" compensate value: "<<std::abs(angle - 180.00) / 180.00 * M_PI<<std::endl;

    if ((current_joint_values[6] + (angle / 90.00 * M_PI / 2)) > 1.8){
        val_7.data = current_joint_values[6] - (std::abs(angle - 180.00) / 180.00 * M_PI);
        pub_7.publish(val_7);
        mutex_rotate(val_7.data);
    }
    else if ((current_joint_values[6] + (angle / 90.00 * M_PI / 2)) < -1.8){
        val_7.data = current_joint_values[6] + (std::abs(angle - 180.00) / 180.00 * M_PI);
        pub_7.publish(val_7);
        mutex_rotate(val_7.data);
    }
    else{
        val_7.data = current_joint_values[6] + (angle / 90.00 * 1.8);
        pub_7.publish(val_7);
        mutex_rotate(val_7.data);
    } 
}

void closeGrabber(ros::Publisher &pub_8, ros::Publisher &pub_9){
    ROS_INFO("Close gripper");
    std_msgs::Float64 val_8 ;
    std_msgs::Float64 val_9 ;
    val_8.data = gripper_mild_close_value; 
    val_9.data = -gripper_open_value;
    pub_8.publish(val_8);
    pub_9.publish(val_9);

    sleep(2.5);
}

void place(moveit::planning_interface::MoveGroup &group, double x, double y, double z, ros::Publisher &pub_8, ros::Publisher &pub_9, bool time_count){

    /*****************************************************************
    *                       Specify target pose                      *
    *****************************************************************/
    ROS_INFO("Setup target_pose for approaching");
    geometry_msgs::Pose target_pose_place;

    // Record start time
    auto start = std::chrono::high_resolution_clock::now();

    target_pose_place.position.x = x;//0.4;
    target_pose_place.position.y = y;//0.0;
    target_pose_place.position.z = z + 0.1;//0.2;
    std::cout<<M_PI<<std::endl;
    tf::Quaternion qat_place = tf::createQuaternionFromRPY(0, -M_PI, 0);
    qat_place.normalize();
    target_pose_place.orientation.x = qat_place.x();//0.577;//0.49; // two-sided gribber
    target_pose_place.orientation.y = qat_place.y();//0.577;//0.49; // two-sided gribber
    target_pose_place.orientation.z = qat_place.z();//0.577;//0.49;
    target_pose_place.orientation.w = qat_place.w(); 

    ROS_INFO("Go to target_pose for pickup");
    // set target pose
    group.setPoseTarget(target_pose_place);
    // plan
    moveit::planning_interface::MoveGroup::Plan my_plan_place;
    bool success_place = group.plan(my_plan_place);
    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success_place?"":"FAILED");    
    // execute
    if(success_place) group.execute(my_plan_place);
    mutex_traj();
    
    /*****************************************************************
    *                          Open grabber                          *
    *****************************************************************/
    ROS_INFO("open grabber");
    openGrabber(pub_8, pub_9);

    // Record end time
    auto finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = finish - start;
    std::cout << "Elapsed time: " << elapsed.count() << " s\n";
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

void gotoNamedTarget(moveit::planning_interface::MoveGroup &group, std::string target, bool constraint_on){

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
    group.setPlanningTime(5.0);
    group.clearPathConstraints();

    // visualization
    ROS_INFO("Visualizing plan home (joint space goal) %s",success?"":"FAILED");
    //sleep(5.0);    
 
    // execution
    ROS_INFO("Execution if plan successfully");
    if(success) group.execute(my_plan);
    mutex_traj();
    
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
    pose.position.z = -0.10;//-0.123;//-0.13;//-0.1;//-0.07

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
          group.setPlanningTime(5.0);
          break;

        default: // no constraints
          break;
    }
}