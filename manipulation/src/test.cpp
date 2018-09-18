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
/* Modified by: Ruinian Xu*/

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

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
#include <unsupported/Eigen/MatrixFunctions> //log

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
void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg);
void compensatePlan(moveit::planning_interface::MoveGroup::Plan &plan, double joint_8_value);
void openGrabber(ros::Publisher &pub_8, double angle);
void closeGrabber(ros::Publisher &pub_8, double angle);
void rotateGripper(ros::Publisher &pub_7, double angle);
void pickup_angle(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose &target_pose1, ros::Publisher &pub_7, ros::Publisher &pub_8, double angle);
void attemptPickup(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose &target_pose1, int direction, ros::Publisher &pub_8);
void attemptPlace(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose &target_pose2, int direction, ros::Publisher &pub_8);
const std::vector<std::string> listNamedPoses(moveit::planning_interface::MoveGroup &group);
void gotoNamedTarget(moveit::planning_interface::MoveGroup &group, std::string target, bool constraint_on);
void addObject2Scene(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher);
void getConstraint(moveit::planning_interface::MoveGroup &group, std::string target);
void adjust(geometry_msgs::Pose &target_pose);
void adjust_Jacobian(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose &target_pose, double diff_threshold);
void Jacobian_hybrid(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose &target_pose, double diff_threshold);
void rot2axisangle();
Eigen::Quaterniond r2q(Eigen::Matrix3d rotation);
double distance_se3(Eigen::Vector3d initialLocation, Eigen::Quaterniond Ini_orientation, Eigen::Vector3d finalLocation, Eigen::Quaterniond Final_orientation, double threshold);


int main(int argc, char **argv)
{

    /*****************************************************************
    *                         Arm initialization                     *
    *****************************************************************/
    // ros initialization
    ros::init(argc, argv, "arm_kinematics");
    ros::NodeHandle node_handle;  
    ros::Publisher pub_8 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_8/command", 1, true);
    ros::Subscriber sub = node_handle.subscribe<sensor_msgs::JointState>("/joint_states", 1, joint_state_handler);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // kinematic_state & kinematic_model loading & planner
    moveit::planning_interface::MoveGroup group("arm");
    // moveit::planning_interface::MoveGroup group_actuator("arm4_full");
    group.setPlannerId("LBKPIECEkConfigDefault");//ForageRRTkConfigDefault//LBKPIECEkConfigDefault//RRTstarkConfigDefault
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
    const std::vector<std::string> &joint_names = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName())->getJointModelNames();
    //std::cout << "Loading model..  joints in arm model:" << std::endl; 
    //for (size_t i = 0; i < 8 ; i++) std::cout << joint_names[i] << std::endl;

    const std::vector<std::string> &link_names = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName())->getLinkModelNames();
    //for (size_t i = 0; i < 8 ; i++) std::cout << link_names[i] << std::endl;

    ROS_INFO("Model frame: %s", group.getCurrentState()->getRobotModel()->getModelFrame().c_str());
    ROS_INFO("Reference Planning frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("EndEffectorLink simulator: %s", group.getEndEffectorLink().c_str());
    //ROS_INFO("EndEffectorLink actuator: %s", group_actuator.getEndEffectorLink().c_str());
    //ROS_INFO("EndEffector: %s", group.getEndEffector().c_str());

    /*****************************************************************
    *                       Visualization setup                      *
    *****************************************************************/
    // for visualization
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    /*test jacobian*/
    /*
ROS_INFO("Go to target_pose1");
    geometry_msgs::Pose target_pose_test;

    // target pose constraints 
    double x1, y1, z1;
    std::cout<<"input x (ex. 0.4):"; std::cin>>x1;
    std::cout<<"input y (ex. 0.2):"; std::cin>>y1;
    std::cout<<"input z (ex. 0.1 or 0.0):"; std::cin>>z1;
    target_pose_test.position.x = x1;//0.4;
    target_pose_test.position.y = y1;//0.0;
    target_pose_test.position.z = z1;//0.2;


    int direction;
    std::cout<<"select pickup direction.. (0: from top(rotate 0); 1: from top(rotate 45); 2: from top(rotate 90); 3: from top(rotate 135 degree); 4: from side): ";std::cin>>direction; 

    switch(direction){
        case 0:
          std::cout<<"from top(rotate 0 degree) selected"<<std::endl;
          //target_pose1.orientation.y = 0.70; // yellow gribber
          //target_pose1.orientation.z = sqrt(1-0.70*0.70); // yellow gribber

          target_pose_test.orientation.x = 0.49;//0.577;//0.49; // two-sided gribber
          target_pose_test.orientation.y = 0.49;//0.577;//0.49; // two-sided gribber
          target_pose_test.orientation.z = 0.49;//0.577;//0.49;
          target_pose_test.orientation.w = sqrt(1-0.49*0.49-0.49*0.49-0.49*0.49);//2.027;//sqrt(1-0.49*0.49-0.49*0.49-0.49*0.49); // two-sided gribber
          break;
        case 1:  
          std::cout<<"from top(rotate 45 degree) selected"<<std::endl;
          //target_pose1.orientation.y = 0.70; // yellow gribber
          //target_pose1.orientation.z = sqrt(1-0.70*0.70); // yellow gribber

          target_pose_test.orientation.x = 0.66; // two-sided gribber
          target_pose_test.orientation.y = 0.24; // two-sided gribber
          target_pose_test.orientation.w = 0.66;
          target_pose_test.orientation.z = sqrt(1-0.66*0.66-0.24*0.24-0.66*0.66); // two-sided gribber 
          break;
        case 2:
          std::cout<<"from top(rotate 90 degree) selected"<<std::endl;
          //target_pose1.orientation.y = 0.70; // yellow gribber
          //target_pose1.orientation.z = sqrt(1-0.70*0.70); // yellow gribber

          target_pose_test.orientation.x = 0.70; // two-sided gribber
          target_pose_test.orientation.y = 0; // two-sided gribber
          target_pose_test.orientation.w = 0.70;
          target_pose_test.orientation.z = sqrt(1-0.70*0.70-0.70*0.70); // two-sided gribber 
          break;
        case 3:
          std::cout<<"from top(rotate 135 degree) selected"<<std::endl;
          //target_pose1.orientation.y = 0.70; // yellow gribber
          //target_pose1.orientation.z = sqrt(1-0.70*0.70); // yellow gribber

          target_pose_test.orientation.x = 0.24; // two-sided gribber
          target_pose_test.orientation.y = 0.66; // two-sided gribber
          target_pose_test.orientation.z = 0.67;
          target_pose_test.orientation.w = sqrt(1-0.24*0.24-0.66*0.66-0.67*0.67); // two-sided gribber 
          break;
        case 4:
          std::cout<<"from side selected"<<std::endl; 
          //target_pose1.orientation.z = 1; // yellow gribber
 
          target_pose_test.orientation.x = 0.70; // two-sided gribber
          target_pose_test.orientation.z = sqrt(1-0.70*0.70); // two-sided gribber
          break;  
        default:
          target_pose_test.orientation.y = 0.70;
          target_pose_test.orientation.z = sqrt(1-0.70*0.70);
          break;
    }
    adjust_Jacobian(group, target_pose_test, 0.03);
*/

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
    addObject2Scene(group, planning_scene_interface ,collision_object_publisher);
  
    /*****************************************************************
    *                        List stored poses                       *
    *****************************************************************/
    // list the pre-defined pose in SRDF file
    const std::vector<std::string> namedTargets = listNamedPoses(group);

    /*****************************************************************
    *                      Specify initial pose                      *
    *****************************************************************/
    // namedTargets stores the names of pre-defined poses(joint values)
    // select the name (ex. "home"), gotoNamedTarget function will find plan to the specific pose
    std::string target = ""; int targetNum = 0;
    std::cout<<"select target pose above: "; std::cin >> targetNum;
    if(targetNum == 0) target = "Home";
    else if(targetNum == 1) target = "holding";
    else target = "home";
    gotoNamedTarget(group, target, 0);

    /*****************************************************************
    *                       Specify target pose                      *
    *****************************************************************/
    ROS_INFO("Go to target_pose1");
    geometry_msgs::Pose target_pose1;


    std::string command = "";
    std::string pos_x = "";
    std::string pos_y = "";
    std::cout<< "Handy is waiting for command! (you might need to run the server)"<<std::endl;
    //while(true){
    //    getCommandFromMeta(command, pos_x, pos_y);
    //    if(command != "") break;
    //}
    /*
    while(true){
        getCommandFromMeta_locationOnly(pos_x, pos_y);
        if(pos_x != "" && pos_y != "") break;
    }
    */

    // target pose constraints 
    double x, y, z;
    int direction;
    
    int flag = 1;
    while(flag)
    {
      std::cout<<"input x (ex. 0.4):"; std::cin>>x;
      std::cout<<"input y (ex. 0.2):"; std::cin>>y;
      std::cout<<"input z (ex. 0.1 or 0.0):"; std::cin>>z;
      target_pose1.position.x = x;//0.4;
      target_pose1.position.y = y;//0.0;
      target_pose1.position.z = z;//0.2;
      std::cout<<"select pickup direction.. (0: from top(rotate 0); 1: from top(rotate 45); 2: from top(rotate 90); 3: from top(rotate 135 degree); 4: from side): ";std::cin>>direction; 

      switch(direction){
          case 0:
            std::cout<<"from top(rotate 0 degree) selected"<<std::endl;
            //target_pose1.orientation.y = 0.70; // yellow gribber
            //target_pose1.orientation.z = sqrt(1-0.70*0.70); // yellow gribber

            target_pose1.orientation.x = 0.49;//0.577;//0.49; // two-sided gribber
            target_pose1.orientation.y = 0.49;//0.577;//0.49; // two-sided gribber
            target_pose1.orientation.z = 0.49;//0.577;//0.49;
            target_pose1.orientation.w = sqrt(1-target_pose1.orientation.x*target_pose1.orientation.x-target_pose1.orientation.y*target_pose1.orientation.y-target_pose1.orientation.z*target_pose1.orientation.z);//2.027;//sqrt(1-0.49*0.49-0.49*0.49-0.49*0.49); // two-sided gribber
            break;
          case 1:  
            std::cout<<"from top(rotate 45 degree) selected"<<std::endl;
            //target_pose1.orientation.y = 0.70; // yellow gribber
            //target_pose1.orientation.z = sqrt(1-0.70*0.70); // yellow gribber

            target_pose1.orientation.x = 0.66; // two-sided gribber
            target_pose1.orientation.y = 0.24; // two-sided gribber
            target_pose1.orientation.w = 0.66;
            target_pose1.orientation.z = sqrt(1-0.66*0.66-0.24*0.24-0.66*0.66); // two-sided gribber 
            break;
          case 2:
            std::cout<<"from top(rotate 90 degree) selected"<<std::endl;
            //target_pose1.orientation.y = 0.70; // yellow gribber
            //target_pose1.orientation.z = sqrt(1-0.70*0.70); // yellow gribber

            target_pose1.orientation.x = 0.70; // two-sided gribber
            target_pose1.orientation.y = 0; // two-sided gribber
            target_pose1.orientation.w = 0.70;
            target_pose1.orientation.z = sqrt(1-0.70*0.70-0.70*0.70); // two-sided gribber 
            break;
          case 3:
            std::cout<<"from top(rotate 135 degree) selected"<<std::endl;
            //target_pose1.orientation.y = 0.70; // yellow gribber
            //target_pose1.orientation.z = sqrt(1-0.70*0.70); // yellow gribber

            target_pose1.orientation.x = 0.24; // two-sided gribber
            target_pose1.orientation.y = 0.66; // two-sided gribber
            target_pose1.orientation.z = 0.67;
            target_pose1.orientation.w = sqrt(1-0.24*0.24-0.66*0.66-0.67*0.67); // two-sided gribber 
            break;
          case 4:
            std::cout<<"from side selected"<<std::endl; 
            //target_pose1.orientation.z = 1; // yellow gribber
            
            target_pose1.orientation.x = 0.70; // two-sided gribber
            target_pose1.orientation.z = sqrt(1-0.70*0.70); // two-sided gribber
            break;  
          default:
            target_pose1.orientation.y = 0.70;
            target_pose1.orientation.z = sqrt(1-0.70*0.70);
            break;
      }

      //while(true){
      //    getCommandFromMeta_commandOnly(command);
      //    if(command != "") break;
      //}


      /*****************************************************************
      *                       Attempt to pick up                       *
      *****************************************************************/
      // specify the target_pose
      // the robot will attempt to:
      // 1. go above the object
      // 2. open grabber
      // 3. go down
      // 4. close grabber 
      attemptPickup(group, target_pose1, direction, pub_8);
      

      /*****************************************************************
      *                             Back home                          *
      *****************************************************************/
      ROS_INFO("back home");
      //target = "home";
      //target = "home";
      gotoNamedTarget(group, target, 1);

      std::cout<<"Do you want to execute another pickup? If yes, 1. If no, 0"<<std::endl;
      std::cin>>flag;
    }

    /*****************************************************************
    *                        Attempt to place                        *
    *****************************************************************/
    geometry_msgs::Pose target_pose2;
    std::cout<<"Input position to place item"<<std::endl;
    std::cout<<"input x (ex. 0.4):"; std::cin>>x;
    std::cout<<"input y (ex. 0.2):"; std::cin>>y;
    std::cout<<"input z (ex. 0.1 or 0.0):"; std::cin>>z;
    target_pose2.position.x = x;//0.4;
    target_pose2.position.y = y;//0.0;
    target_pose2.position.z = z;//0.2;

    std::cout<<"select pickup direction.. (0: from top; 1: from side): ";std::cin>>direction; 

    switch(direction){
        case 0:
          std::cout<<"from top selected"<<std::endl;
          //target_pose1.orientation.y = 0.70; // yellow gribber
          //target_pose1.orientation.z = sqrt(1-0.70*0.70); // yellow gribber

          target_pose2.orientation.x = 0.49; // two-sided gribber
          target_pose2.orientation.y = 0.49; // two-sided gribber
          target_pose2.orientation.z = 0.49;
          target_pose2.orientation.w = sqrt(1-0.49*0.49-0.49*0.49-0.49*0.49); // two-sided gribber
          break;
        case 1:  
          std::cout<<"from side selected"<<std::endl; 
          //target_pose1.orientation.z = 1; // yellow gribber
 
          target_pose2.orientation.x = 0.70; // two-sided gribber
          target_pose2.orientation.z = sqrt(1-0.70*0.70); // two-sided gribber
          break;
        default:
          target_pose2.orientation.y = 0.70;
          target_pose2.orientation.z = sqrt(1-0.70*0.70);
          break;
    }
    attemptPlace(group, target_pose2, direction, pub_8);

     /*****************************************************************
    *                             Back home                          *
    *****************************************************************/
    ROS_INFO("back home");
    //target = "home";
    //target = "home";
    gotoNamedTarget(group, target, 1);
       
    /*****************************************************************
    *                              done                              *
    *****************************************************************/
    
    ros::spin();
    return 0;
}


void attemptPlace(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose &target_pose2, int direction, ros::Publisher &pub_8)
{
    ROS_INFO("Go to target_pose2");
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
/******************************************************************
*                 First move to close enough position             *
******************************************************************/
 /*   geometry_msgs::Pose temp_target_pose2;
    temp_target_pose2.position.x = target_pose2.position.x - 0.05;
    temp_target_pose2.position.y = target_pose2.position.y - 0.05;
    temp_target_pose2.position.z = target_pose2.position.z;
    temp_target_pose2.orientation.x = target_pose2.orientation.x;
    temp_target_pose2.orientation.y = target_pose2.orientation.y;
    temp_target_pose2.orientation.z = target_pose2.orientation.z;
    temp_target_pose2.orientation.w = target_pose2.orientation.w;

    // set target pose
    group.setPoseTarget(temp_target_pose2);

    // plan
    
    my_plan.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    compensatePlan(my_plan, current_joint_values[7]);

    // visualization
    ROS_INFO("Visualizing plan 2 (pose goal) %s",success?"":"FAILED");    
   // sleep(1.5);

    // execute
    if(success) group.execute(my_plan);
    sleep(2.5);
*/
/******************************************************************
*                      Then move to target                        *
******************************************************************/
    // set target pose
    group.setPoseTarget(target_pose2);

    // plan
    //moveit::planning_interface::MoveGroup::Plan my_plan;
    success = group.plan(my_plan);
    my_plan.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    //compensatePlan(my_plan, current_joint_values[7]);

    // visualization
    ROS_INFO("Visualizing plan 2 (pose goal) %s",success?"":"FAILED");    
   // sleep(1.5);

    // execute
    if(success) group.execute(my_plan);
    sleep(5);

    /*****************************************************************
    *                          Open grabber                          *
    *****************************************************************/
    ROS_INFO("open grabber");
    openGrabber(pub_8, -1.5);

    /*****************************************************************
    *                   Move grabber a little bit                    *
    *****************************************************************/

    //target_pose2.position.y -= 0.15;
    target_pose2.position.z += 0.07;
   
    // set target pose
    group.setPoseTarget(target_pose2);

    // plan
    success = group.plan(my_plan);

    my_plan.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    //compensatePlan(my_plan, current_joint_values[7]);

    // visualization
    ROS_INFO("Visualizing plan 2 (pose goal) %s",success?"":"FAILED");    
    //sleep(1.5);

    // execute
    if(success) group.execute(my_plan);
    sleep(5);

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


void compensatePlan(moveit::planning_interface::MoveGroup::Plan &plan, double joint_8_value){
    double joint_0_compensate = -0.3186;
    double joint_1_compensate = -0.25;
    for(size_t idx = 0; idx < plan.trajectory_.joint_trajectory.points.size(); idx++){
        plan.trajectory_.joint_trajectory.points[idx].positions[0] += joint_0_compensate;
        plan.trajectory_.joint_trajectory.points[idx].positions[1] += joint_1_compensate;
        plan.trajectory_.joint_trajectory.points[idx].positions.push_back(joint_8_value);
    }
    //std::cout<<my_plan.planning_time_<<std::endl;
    //std::cout<<my_plan.start_state_<<std::endl;
    //std::cout<<my_plan.trajectory_<<std::endl;
    //std::cout<<my_plan.trajectory_.joint_trajectory.points[3]<<std::endl;
    //std::cout<<my_plan.trajectory_.joint_trajectory.joint_names[0]<<std::endl;
    //std::cout<<my_plan.trajectory_.joint_trajectory.joint_names.size()<<std::endl;
    for(size_t idx = 0; idx < plan.trajectory_.joint_trajectory.points.back().positions.size(); idx++){
        std::cout<<"point = "<<plan.trajectory_.joint_trajectory.points.back().positions[idx]<<std::endl;
    }
}

void openGrabber(ros::Publisher &pub_8, double angle){
    std_msgs::Float64 val_8 ;
    val_8.data = angle;
    pub_8.publish(val_8);
    sleep(3);
}

void rotateGripper(ros::Publisher &pub_7, double angle) {
    std_msgs::Float64 val_7 ;
    val_7.data = angle;
    pub_7.publish(val_7);
    sleep(3);  
}

void closeGrabber(ros::Publisher &pub_8, double angle){
    std_msgs::Float64 val_8 ;
    val_8.data = angle;
    pub_8.publish(val_8);
    sleep(3);
}

void pickup_angle(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose &target_pose1, ros::Publisher &pub_7,ros::Publisher &pub_8, double angle){
      /*****************************************************************
    *                       Specify target pose                      *
    *****************************************************************/
    ROS_INFO("Go to target_pose1");
    // set target pose
    group.setPoseTarget(target_pose1);
    // plan
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    my_plan.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
    // sleep(1.5);
    // execute
    if(success) group.execute(my_plan);
    sleep(5);
    
    /****************************************************************
    *                          Rotate wrist                         *
    ****************************************************************/
    rotateGripper(pub_7, angle / 90.00 * 1.8);

    /*****************************************************************
    *                          Open grabber                          *
    *****************************************************************/
    ROS_INFO("open grabber");
    openGrabber(pub_8, -1.0);

    /****************************************************************
    *                Move down to pick up by jacobian               *
    *****************************************************************/
    robot_state::RobotStatePtr kinematic_state = group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName());
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
    const Eigen::Affine3d &temp = kinematic_state->getGlobalLinkTransform("link_8");

    Eigen::Vector3d Trans;
    Eigen::Matrix3d Rot;
    Trans = temp.translation();
    Rot =  temp.rotation();
    Eigen::Quaterniond q = r2q(Rot);
    target_pose1.orientation.x = q.x();
    target_pose1.orientation.y = q.y();
    target_pose1.orientation.z = q.z();
    target_pose1.orientation.w = q.w();

    target_pose1.position.z = target_pose1.position.z - 0.7;

    Jacobian_hybrid(group, target_pose1, 0.03);

    /*****************************************************************
    *                          Close grabber                          *
    *****************************************************************/
    ROS_INFO("open grabber");
    closeGrabber(pub_8, 0.0);
}

void attemptPickup(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose &target_pose1, int direction, ros::Publisher &pub_8){

    /*****************************************************************
    *                       Specify target pose                      *
    *****************************************************************/
    ROS_INFO("Go to target_pose1");

    // set target pose
    group.setPoseTarget(target_pose1);

    // plan
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    my_plan.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    //compensatePlan(my_plan, current_joint_values[7]);

    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
   // sleep(1.5);

    // execute
    if(success) group.execute(my_plan);
    sleep(5);

    /*****************************************************************
    *                     Specify path constraint                    *
    *****************************************************************/
    // fix link_8 orientation
    /*
    moveit_msgs::OrientationConstraint ocm;  
    ocm.link_name = "link_8";  
    ocm.header.frame_id = "world";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;
  
    // Now, set it as the path constraint for the group.
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);  
    group.setPathConstraints(test_constraints);
    */


    /*****************************************************************
    *                          Open grabber                          *
    *****************************************************************/
    ROS_INFO("open grabber");
    openGrabber(pub_8, -1.0);

    /*****************************************************************
    *                       vision closed loop                       *
    *****************************************************************/
    //adjust(geometry_msgs::Pose &target_pose1);
    /*
    double x_diff = 1.0;
    double y_diff = 1.0;
    double diff_margin = 0.03;
    
    // check loop
    while(true){ 
      // parameters  
      std::string command;
      std::string x_diff_s;
      std::string y_diff_s;

      // read current diffs based on vision sensor
      std::ifstream closedLoopFileIn ("/home/fujenchu/Projects/ARM_project/shareFromWin/closedLoopInfo.txt");
      getline (closedLoopFileIn,command);
      if(command != ""){ // got sth
        getline (closedLoopFileIn,x_diff_s);
        getline (closedLoopFileIn,y_diff_s);

        std::cout << "receive a command = " << command << std::endl; 
        std::cout<<"x_diff = "<<x_diff_s<<std::endl;
        std::cout<<"y_diff = "<<y_diff_s<<std::endl;
        x_diff = std::atof(x_diff_s.c_str());
        y_diff = std::atof(y_diff_s.c_str());
      }
      closedLoopFileIn.close();

      // if safisfied, break
      if( x_diff < diff_margin && y_diff < diff_margin) break;

      // if not satisfied, modify position based on readings
      target_pose1.position.x += x_diff;
      target_pose1.position.y += y_diff;
      ROS_INFO("Tweak positions");
      ROS_INFO("new target position x: %f", target_pose1.position.x);
      ROS_INFO("new target position y: %f", target_pose1.position.y);

      // set target pose
      group.setPoseTarget(target_pose1);

      // plan
      success = group.plan(my_plan);

      my_plan.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
      compensatePlan(my_plan, current_joint_values[7]);

      // visualization
      ROS_INFO("Visualizing plan 2 (pose goal) %s",success?"":"FAILED");    
      sleep(5.0);

      // execute
      if(success) group.execute(my_plan);
      sleep(5.0);


    }
    */
    ROS_INFO("closed loop");
    //adjust_Jacobian(group, target_pose1, 0.03);

    std::cout<<"select pickup direction.. (0: from top(rotate 0); 1: from top(rotate 45); 2: from top(rotate 90); 3: from top(rotate 135 degree); 4: from side): ";std::cin>>direction; 

    switch(direction){
        case 0:
          std::cout<<"from top(rotate 0 degree) selected"<<std::endl;
          //target_pose1.orientation.y = 0.70; // yellow gribber
          //target_pose1.orientation.z = sqrt(1-0.70*0.70); // yellow gribber

          target_pose1.orientation.x = 0.491; // two-sided gribber
          target_pose1.orientation.y = 0.491; // two-sided gribber
          target_pose1.orientation.z = 0.508; //0.508
          target_pose1.orientation.w = sqrt(1-0.491*0.491-0.491*0.491-0.508*0.508); // two-sided gribber
          break;
        case 1:  
          std::cout<<"from top(rotate 45 degree) selected"<<std::endl;
          //target_pose1.orientation.y = 0.70; // yellow gribber
          //target_pose1.orientation.z = sqrt(1-0.70*0.70); // yellow gribber

          target_pose1.orientation.x = 0.66; // two-sided gribber
          target_pose1.orientation.y = 0.24; // two-sided gribber
          target_pose1.orientation.w = 0.66;
          target_pose1.orientation.z = sqrt(1-0.66*0.66-0.24*0.24-0.66*0.66); // two-sided gribber 
          break;
        case 2:
          std::cout<<"from top(rotate 90 degree) selected"<<std::endl;
          //target_pose1.orientation.y = 0.70; // yellow gribber
          //target_pose1.orientation.z = sqrt(1-0.70*0.70); // yellow gribber

          target_pose1.orientation.x = 0.70; // two-sided gribber
          target_pose1.orientation.y = 0; // two-sided gribber
          target_pose1.orientation.w = 0.70;
          target_pose1.orientation.z = sqrt(1-0.70*0.70-0.70*0.70); // two-sided gribber 
          break;
        case 3:
          std::cout<<"from top(rotate 135 degree) selected"<<std::endl;
          //target_pose1.orientation.y = 0.70; // yellow gribber
          //target_pose1.orientation.z = sqrt(1-0.70*0.70); // yellow gribber

          target_pose1.orientation.x = 0.24; // two-sided gribber
          target_pose1.orientation.y = 0.66; // two-sided gribber
          target_pose1.orientation.z = 0.67;
          target_pose1.orientation.w = sqrt(1-0.24*0.24-0.66*0.66-0.67*0.67); // two-sided gribber 
          break;
        case 4:
          std::cout<<"from side selected"<<std::endl; 
          //target_pose1.orientation.z = 1; // yellow gribber
 
          target_pose1.orientation.x = 0.70; // two-sided gribber
          target_pose1.orientation.z = sqrt(1-0.70*0.70); // two-sided gribber
          break;  
        default:
          target_pose1.orientation.y = 0.70;
          target_pose1.orientation.z = sqrt(1-0.70*0.70);
          break;
    }

    /*****************************************************************
    *                             Pick                               *
    *****************************************************************/
    ROS_INFO("Go to pick");
    switch(direction){
        case 0: // goes down to pick
          target_pose1.position.z -= 0.07;
          break;
        case 1: // goes down to pick
          target_pose1.position.z -= 0.07;
          break;
        case 2: // goes down to pick
          target_pose1.position.z -= 0.07;
          break;
        case 3: // goes down to pick
          target_pose1.position.z -= 0.07;
          break;
        case 4: // goes to side to pick 
          target_pose1.position.y += 0.1;
          break;
        default:
          target_pose1.position.z -= 0.1;
          break;
    }
     
    // set target pose
    group.setPoseTarget(target_pose1);

    // plan
    success = group.plan(my_plan);

    my_plan.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    //compensatePlan(my_plan, current_joint_values[7]);

    // visualization
    ROS_INFO("Visualizing plan 2 (pose goal) %s",success?"":"FAILED");    
    //sleep(1.5);

    // execute
    if(success) group.execute(my_plan);
    sleep(5);


    /*****************************************************************
    *                          Close grabber                         *
    *****************************************************************/
    ROS_INFO("close grabberrrr");
    closeGrabber(pub_8, 0.0);//0.3

    /*****************************************************************
    *                        lift a liite bit                        *
    *****************************************************************/
    ROS_INFO("lift grabberrrr");
    target_pose1.position.z += 0.03;
    // set target pose
    group.setPoseTarget(target_pose1);

    // plan
    success = group.plan(my_plan);

    my_plan.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    //compensatePlan(my_plan, current_joint_values[7]);

    // visualization
    ROS_INFO("Visualizing plan 2 (pose goal) %s",success?"":"FAILED");    
    //sleep(1.5);

    // execute
    if(success) group.execute(my_plan);
    sleep(5);
  
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
    my_plan.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    //compensatePlan(my_plan, current_joint_values[7]);

    // set planning time to default
    group.setPlanningTime(20.0);
    group.clearPathConstraints();

    // visualization
    ROS_INFO("Visualizing plan home (joint space goal) %s",success?"":"FAILED");
    //sleep(5.0);    
 
    // execution
    ROS_INFO("Execution if plan successfully");
    if(success) group.execute(my_plan);
    sleep(2);
    
    // show current Joints (the real robot joints, not the planned joints)
    std::vector<double> currentJoints = group.getCurrentJointValues();
    std::cout<< "current joint values:"<<std::endl;
    for(size_t i = 0; i<currentJoints.size(); i++) std::cout<< currentJoints[i]<<" ";
    std::cout<<std::endl;


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
    pose.position.z = -0.15;//-0.1;//-0.07

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
          group.setPlanningTime(20.0);
          break;

        default: // no constraints
          break;
    }


}
void adjust(geometry_msgs::Pose &target_pose){

    // get target and grabber from vision
    std::string target_x, target_y, grabber_x, grabber_y;
    double target_x_val, target_y_val, grabber_x_val, grabber_y_val;
    std::ifstream myfileIn ("/home/fujenchu/Projects/ARM_project/shareFromWin/detection_results.txt");
    if(true){ // got sth
        getline (myfileIn,target_x);
        getline (myfileIn,target_y);
        getline (myfileIn,grabber_x);
        getline (myfileIn,grabber_y);
        
        std::ofstream myfileOut ("/home/fujenchu/Projects/ARM_project/shareFromWin/detection_results.txt", std::ios::trunc);
        myfileOut << "";
        myfileOut.close();
     
        std::cout<<"x = "<<target_x<<std::endl;
        std::cout<<"y = "<<target_y<<std::endl;
        std::cout<<"x = "<<grabber_x<<std::endl;
        std::cout<<"y = "<<grabber_y<<std::endl;
    }
    myfileIn.close();

    target_x_val = std::atof(target_x.c_str());
    target_y_val = std::atof(target_y.c_str());
    grabber_x_val = std::atof(grabber_x.c_str());
    grabber_y_val = std::atof(grabber_y.c_str());

    // adject based on current difference
    target_pose.position.x = target_pose.position.x + (target_x_val - grabber_x_val);
    target_pose.position.y = target_pose.position.y + (target_y_val - grabber_y_val);

}

void Jacobian_hybrid(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose &target_pose, double diff_threshold = 0.03)
{
    Eigen::VectorXd initialJoints(7); // 7 by 1 vector //7J
    Eigen::Matrix4d initialPoseG = Eigen::MatrixXd::Identity(4,4); // 4 by 4 matrix for transformation
    Eigen::VectorXd initialLocation(3); 
    Eigen::Matrix3d InitialRot;
    
    Eigen::VectorXd finalLocation(3); 
    Eigen::Matrix3d FinalRot;
    Eigen::MatrixXd finalPoseG = Eigen::MatrixXd::Identity(4,4);
    Eigen::Quaterniond Fianl_orientation;

    Eigen::Vector3d w(0.0,0.0,0.0);
    Eigen::Vector3d v(0.0,0.0,0.0);
    Eigen::MatrixXd twist_spatial = Eigen::MatrixXd::Zero(4,4);

    Eigen::VectorXd currentLocation(3);
    Eigen::VectorXd jointsVelocity;
    Eigen::VectorXd newJoints;
    Eigen::Matrix4d relativePoseG;
    Eigen::Matrix3d relativeR;
    Eigen::Vector3d relativeP(0.0,0.0,0.0);

    Eigen::MatrixXd jacobian_all = Eigen::MatrixXd::Zero(6,8); // 6 by 8 to catch Jacobian matrix from ROS
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,7); // 6 by 7 Jacobian matrix (7 joints) //7J
    Eigen::MatrixXd jacPseudoInv; // 7 by 6 inverse Jacobian matrix (7 by 6 invJ* 6 by 1 twist = 7 by 1 new joint values )//7J

    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);

    // publish data
    ros::NodeHandle node_handle;
    ros::Publisher pub_1 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_1/command", 1, true);
    ros::Publisher pub_2 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_2/command", 1, true);
    ros::Publisher pub_3 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_3/command", 1, true);
    ros::Publisher pub_4 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_4/command", 1, true);
    ros::Publisher pub_5 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_5/command", 1, true);
    ros::Publisher pub_6 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_6/command", 1, true);
    ros::Publisher pub_7 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_7/command", 1, true);

    //Setup for target pose configuration
    finalLocation << target_pose.position.x, target_pose.position.y, target_pose.position.z;
    double q11 = target_pose.orientation.x;
    double q12 = target_pose.orientation.y;
    double q13 = target_pose.orientation.z;
    double q10 = target_pose.orientation.w;
    Fianl_orientation.x() = target_pose.orientation.x;
    Fianl_orientation.y() = target_pose.orientation.y;
    Fianl_orientation.z() = target_pose.orientation.z;
    Fianl_orientation.w() = target_pose.orientation.w;

    FinalRot(0,0) = 1.0 - 2.0 * (q12 * q12 + q13 * q13);
    FinalRot(0,1) = 2.0 * (q11 * q12 - q10 * q13);
    FinalRot(0,2) = 2.0 * (q10 * q12 + q11 * q13);
    FinalRot(1,0) = 2.0 * (q11 * q12 + q10 * q13);
    FinalRot(1,1) = 1.0 - 2.0 * (q11 * q11 + q13 * q13);
    FinalRot(1,2) = 2.0 * (q12 * q13 - q10 * q11);
    FinalRot(2,0) = 2.0 * (q11 * q13 - q10 * q12);
    FinalRot(2,1) = 2.0 * (q10 * q11 + q12 * q13);
    FinalRot(2,2) = 1.0 - 2.0 * (q11 * q11 + q12 * q12);
    
    finalPoseG.block(0,0,3,3) << FinalRot;
    finalPoseG.block(0,3,3,1) = finalLocation.block(0,0,3,1);

    //Setup for initial pose configuration
    robot_state::RobotStatePtr kinematic_state = group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName());
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
    const Eigen::Affine3d &temp = kinematic_state->getGlobalLinkTransform("link_8");

    initialLocation = temp.translation();
    InitialRot =  temp.rotation();
    std::cout<<"Initial translation and rotation are: \n"<<initialLocation<<"\n"<<InitialRot<<std::endl;

    double dist;
    dist = distance_se3(initialLocation, r2q(InitialRot), finalLocation, Fianl_orientation, diff_threshold);

    double step_size = 0.02;

    while(dist > diff_threshold) {
      kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
      initialJoints << current_joint_values[0], current_joint_values[1], current_joint_values[2], current_joint_values[3], current_joint_values[4], current_joint_values[5], current_joint_values[6];//7J
      //get current eef_state  
      const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("link_8");//7J

      /*****************************************************************
      *                 Joint values to inverse Jacobian               *
      *****************************************************************/
      // Get the Jacobian
      kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames()[6]), reference_point_position, jacobian_all);//7J
      
      // Get inverse Jacobian of 6 links
      jacobian = jacobian_all.block(0,0,6,7);//7J
      //ROS_INFO_STREAM("jacobian_all: " << jacobian_all);
      //ROS_INFO_STREAM("jacobian: " << jacobian);    
      //ROS_INFO_STREAM("joint_model_group->getLinkModelNames()[6]: " << joint_model_group->getLinkModelNames()[6]);  

      Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(6,6);//7J 
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
      for(size_t idx = 0; idx < 6; idx++)
      {// only 6 no matter link_7 or link_8
          double singularvalue = svd.singularValues()[idx];
          if(singularvalue < 0.03) 
          {
              S_inv(idx,idx) = 0.0;
              std::cout<<"singularity occurred!"<<std::endl;
          }
          else 
              S_inv(idx,idx) = 1 / singularvalue;
      }

      Eigen::MatrixXd tempV = Eigen::MatrixXd::Ones(7,6);//7J
      tempV.block(0,0,7,6) = svd.matrixV();
      jacPseudoInv = tempV * S_inv * svd.matrixU().inverse();

      /***********************************************************
      *                  Compute angle velocity                  *
      ************************************************************/
      Eigen::MatrixXd relativeRot = InitialRot.inverse() * FinalRot;
      Eigen::MatrixXd Rot_body_hat = relativeRot.log();
      Eigen::VectorXd Rot_body(3);
      Rot_body << Rot_body_hat(2,1), Rot_body_hat(0,2), Rot_body_hat(1,0);
      Eigen::VectorXd Rot_spatial(3);
      Rot_spatial = InitialRot * Rot_body;
      //std::cout<<"Rot_spatial: \n"<<Rot_spatial<<std::endl;

      Eigen::VectorXd Trans_body = finalLocation - initialLocation;
      Trans_body = InitialRot.inverse() * Trans_body;
      Eigen::VectorXd temp1 = InitialRot * Rot_body;//Ri * wb
      Eigen::VectorXd temp2(3);
      temp2 << (initialLocation(1) * temp1(2) - temp1(1) * initialLocation(2)), (initialLocation(2) * temp1(0) - temp1(2) * initialLocation(0)), (initialLocation(0) * temp1(1) - temp1(0) * initialLocation(1));
      //Eigen::VectorXd Trans_spatial =  (InitialRot * Trans_body + temp2).normalized() * t;
      Eigen::VectorXd Trans_spatial =  InitialRot * (Trans_body.normalized() * step_size) + temp2;
      Trans_spatial = Trans_spatial;

      Eigen::VectorXd spatial_velocity(6);
      spatial_velocity.block(0,0,3,1) = Trans_spatial;
      spatial_velocity.block(3,0,3,1) = Rot_spatial * 2 * step_size;

      //Update Joint values                       
      jointsVelocity = jacPseudoInv * spatial_velocity;
      newJoints = initialJoints + jacPseudoInv * spatial_velocity;

      /**********************************
      *          Update twist           *
      ***********************************/
      //update joint values
      std_msgs::Float64 val_1;
      std_msgs::Float64 val_2;
      std_msgs::Float64 val_3;
      std_msgs::Float64 val_4;
      std_msgs::Float64 val_5;
      std_msgs::Float64 val_6;
      std_msgs::Float64 val_7;
      val_1.data = newJoints[0];
      val_2.data = newJoints[1];
      val_3.data = newJoints[2];
      val_4.data = newJoints[3];
      val_5.data = newJoints[4];
      val_6.data = newJoints[5];
      val_7.data = newJoints[6];//7J

      pub_1.publish(val_1);
      pub_2.publish(val_2);
      pub_3.publish(val_3);
      pub_4.publish(val_4);
      pub_5.publish(val_5);
      pub_6.publish(val_6);
      pub_7.publish(val_7); //7J 

      double maxLength = 0.0;
      double waitTime  = 0.0;
      for(size_t idx = 0; idx < 7; idx++){//7J
          if(std::abs(jointsVelocity[idx]) > maxLength) maxLength = std::abs(jointsVelocity[idx]);
      }
      waitTime = maxLength/0.5;
      waitTime = waitTime + 0.05; // second
      unsigned int microseconds = waitTime * 1000000; // microsecond

      usleep(microseconds);

      /********************************************
      *           Stopping Criteria               *
      *********************************************/
      kinematic_state->setJointGroupPositions(joint_model_group, newJoints);
      const Eigen::Affine3d &temp_eef = kinematic_state->getGlobalLinkTransform("link_8");
      initialLocation = temp_eef.translation();
      InitialRot =  temp_eef.rotation();

      //update distance
      dist = distance_se3(initialLocation, r2q(InitialRot), finalLocation, Fianl_orientation, diff_threshold);
    }
}

void adjust_Jacobian(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose &target_pose, double diff_threshold = 0.03)
{
    /**
    adjust the end-effector with Jacobian     
    **/
    
    /*****************************************************************
    *                   Define and Initialize Objects                *
    *****************************************************************/
    Eigen::VectorXd initialJoints(7); // 7 by 1 vector //7J
    Eigen::VectorXd initialPose(6); // 6 by 1 vector for twist
    Eigen::Matrix4d initialPoseG; // 4 by 4 matrix for transformation
    
    Eigen::VectorXd finalLocation(3); 
    Eigen::MatrixXd finalPoseG = Eigen::MatrixXd::Identity(4,4);
    
    Eigen::MatrixXd jacobian_all = Eigen::MatrixXd::Zero(6,8); // 6 by 8 to catch Jacobian matrix from ROS
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,7); // 6 by 6 Jacobian matrix (6 joints) //7J
    Eigen::MatrixXd jacPseudoInv; // 7 by 6 inverse Jacobian matrix (7 by 6 invJ* 6 by 1 twist = 7 by 1 new joint values )//7J

    Eigen::Vector3d w(0.0,0.0,0.0);
    Eigen::Vector3d v(0.0,0.0,0.0);
    Eigen::VectorXd twist(6);

    Eigen::VectorXd currentLocation(3);
    Eigen::VectorXd jointsVelocity;
    Eigen::VectorXd newJoints;
    Eigen::Matrix4d relativePoseG;
    Eigen::Matrix3d relativeR;
    Eigen::Vector3d relativeP(0.0,0.0,0.0);

    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::Matrix3d eyes = Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix3d W_head;

    //double diff = 1000.0;  
    double tau = 0.0;
    double difftau = 1000.0;
    double diffheight = 1000.0;
    double traceR = 0.0;
    double diff_angle = 1000.00;
    double diff_pos = 1000.00;

    int vnormScale = 0;
    int wnormScale = 0;

    // publish data
    ros::NodeHandle node_handle;
    ros::Publisher pub_1 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_1/command", 1, true);
    ros::Publisher pub_2 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_2/command", 1, true);
    ros::Publisher pub_3 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_3/command", 1, true);
    ros::Publisher pub_4 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_4/command", 1, true);
    ros::Publisher pub_5 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_5/command", 1, true);
    ros::Publisher pub_6 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_6/command", 1, true);
    ros::Publisher pub_7 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_7/command", 1, true);

    ROS_INFO_STREAM("initializing.. \n");
    sleep(5);
    // set up (1)kinematic_state, (2)joint_model_group
    ROS_INFO_STREAM("initializing1.. \n");
    robot_state::RobotStatePtr kinematic_state = group.getCurrentState();
    ROS_INFO_STREAM("initializing2.. \n");

    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName());

    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    for(size_t idx = 0; idx < joint_names.size(); idx++ ){
       std::cout<< joint_names[idx]<<std::endl;
    }

    ROS_INFO_STREAM("initializing3.. \n");

    ROS_INFO_STREAM("initializing4.. \n");

    // get ((1)initialRot, (2)current_joint_values
    kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
    ROS_INFO_STREAM("initializing5.. \n");
    //get end-effector pose configuration for temp
    const Eigen::Affine3d &temp = kinematic_state->getGlobalLinkTransform("link_8");//7J
    ROS_INFO_STREAM("initializing6.. \n");
    Eigen::Matrix3d initialRot = temp.rotation();

    double q1 = target_pose.orientation.x;
    double q2 = target_pose.orientation.y;
    double q3 = target_pose.orientation.z;
    double q0 = target_pose.orientation.w;
    /*compute the euler angle from quarternion*/
    double ax = std::atan2(2.0*(q0*q1+q2*q3),1.0-2.0*(q1*q1+q2*q2));
    double ay = std::asin(2.0*(q0*q2-q1*q3));
    double az = std::atan2(2.0*(q0*q3+q2*q1),1.0-2.0*(q3*q3+q2*q2));
    /*construct x,y,z axis rotation matrix*/
    Eigen::Matrix3d RotX = Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix3d RotY = Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix3d RotZ = Eigen::MatrixXd::Identity(3,3);
    RotX(1,1) = std::cos(ax);
    RotX(1,2) = -std::sin(ax);
    RotX(2,2) = std::cos(ax);
    RotX(2,1) = std::sin(ax);

    RotY(0,0) = std::cos(ay);
    RotY(2,2) = std::cos(ay);
    RotY(0,2) = std::sin(ay);
    RotY(2,0) = -std::sin(ay);

    RotZ(0,0) = std::cos(az);
    RotZ(1,1) = std::cos(az);
    RotZ(1,0) = std::sin(az);
    RotZ(0,1) = -std::sin(az);

    Eigen::Matrix3d FinalRot = RotX * RotY * RotZ;
    /*
    FinalRot(0,0) = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
    FinalRot(0,1) = 2.0 * (q1 * q2 - q0 * q3);
    FinalRot(0,2) = 2.0 * (q0 * q2 + q1 * q3);
    FinalRot(1,0) = 2.0 * (q1 * q2 + q0 * q3);
    FinalRot(1,1) = 1.0 - 2.0 * (q1 * q1 + q3 * q3);
    FinalRot(1,2) = 2.0 * (q2 * q3 - q0 * q1);
    FinalRot(2,0) = 2.0 * (q1 * q3 - q0 * q2);
    FinalRot(2,1) = 2.0 * (q0 * q1 + q2 * q3);
    FinalRot(1,1) = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
    */

    //std::cout<<"quarternion for initialRot"<<std::endl;
    //std::cout<<temp.rotation().x<<" "<<<temp.rotation().y<<" "<<<temp.rotation().z<<std::endl;

    ROS_INFO_STREAM("initializing7.. \n");
    ROS_INFO_STREAM("initial position: \n" << temp.translation());
    ROS_INFO_STREAM("initialRot: \n" << initialRot);
    ROS_INFO_STREAM("eular angle of initialRot: \n" << temp.rotation().eulerAngles(0,1,2));
    /*****************************************************************
    *                       Reset META Command                       *
    *****************************************************************/
    //bool checkFlag = false;
    /*****************************************************************
    *                       Termination parameters                   *
    *****************************************************************/
    //double difference = 1000.0;
    //double angDiff = 1000.0;
    /*****************************************************************
    *                    Start Inverse Jacobian Loop                 *
    *****************************************************************/

 
    //finalPoseG.block(0,0,3,3) << 1,0,0,0,1,0,0,0,1;
    finalPoseG.block(0,0,3,3) << FinalRot;
    //finalPoseG.block(0,0,3,3) << initialRot;
    //finalLocation << 0.45, 0.20, 0.15;
    finalLocation << target_pose.position.x, target_pose.position.y, target_pose.position.z;
    finalPoseG.block(0,3,3,1) = finalLocation.block(0,0,3,1);

    while(diff_pos > diff_threshold || diff_angle > 0.02)
    { // || difftau > 0.5 or || diffheight > 0.05

            ROS_INFO_STREAM("looping.. \n");
            sleep(1);

            //get current joint values
            kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
            initialJoints << current_joint_values[0], current_joint_values[1], current_joint_values[2], current_joint_values[3], current_joint_values[4], current_joint_values[5], current_joint_values[6];//7J
            //get current eef_state  
            const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("link_8");//7J
            //initialPose << end_effector_state.translation(), end_effector_state.rotation().eulerAngles(0,1,2);
            initialPoseG = end_effector_state.matrix();
            ROS_INFO_STREAM("initRot: \n" << end_effector_state.rotation());
            ROS_INFO_STREAM("initialPoseG: \n" << end_effector_state.matrix());

            /*****************************************************************
            *                 Joint values to inverse Jacobian               *
            *****************************************************************/
            // Get the Jacobian
            kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames()[6]), reference_point_position, jacobian_all);//7J
            
            // Get inverse Jacobian of 6 links
            jacobian = jacobian_all.block(0,0,6,7);//7J
            //ROS_INFO_STREAM("jacobian_all: " << jacobian_all);
            //ROS_INFO_STREAM("jacobian: " << jacobian);    
            //ROS_INFO_STREAM("joint_model_group->getLinkModelNames()[6]: " << joint_model_group->getLinkModelNames()[6]);  

            Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(6,6);//7J 
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
            for(size_t idx = 0; idx < 6; idx++)
            {// only 6 no matter link_7 or link_8
                double singularvalue = svd.singularValues()[idx];
                if(singularvalue < 0.03) 
                {
                    S_inv(idx,idx) = 0.0;
                    std::cout<<"singularity occurred!"<<std::endl;
                }
                else 
                    S_inv(idx,idx) = 1 / singularvalue;
            }

            //U is 6 by 6
            //S is 6 by 6
            //V is 7 by 6
            //ROS_INFO_STREAM("svd.matrixV(): " << svd.matrixV());
            //ROS_INFO_STREAM("svd.matrixU(): " << svd.matrixU());
            Eigen::MatrixXd tempV = Eigen::MatrixXd::Ones(7,6);//7J
            tempV.block(0,0,7,6) = svd.matrixV();

            jacPseudoInv = tempV * S_inv * svd.matrixU().inverse();
            std::cout<<"jacPseudoInv"<<std::endl;std::cout<<jacPseudoInv<<std::endl;
                
            /*****************************************************************
            *                    Relative Pose to Twist                      *
            *****************************************************************/
            // get relativePoseG as spatial frame
            relativePoseG = finalPoseG*initialPoseG.inverse();
            std::cout<<"relativePoseG"<<std::endl;std::cout<<relativePoseG<<std::endl;
            relativeR = relativePoseG.block(0,0,3,3);
            relativeP = relativePoseG.block(0,3,3,1);
            traceR = relativeR(0,0) + relativeR(1,1) + relativeR(2,2);
            std::cout<<"traceR"<<std::endl;std::cout<<traceR<<std::endl;

            // calculate v, w depending on tau
            tau = acos(0.5*(traceR - 1.0));
            std::cout<<"tau"<<std::endl;std::cout<<tau<<std::endl;
            if (tau < 0.1)
            {
                w << 0.0,0.0,0.0;
                v << relativeP;
            }
            else
            {
                W_head = (relativeR - relativeR.transpose()) / (2.0*sin(tau));
                w << W_head(2,1), W_head(0,2), W_head(1,0);
                //ROS_INFO_STREAM("w: " << w.transpose());
                v = ((eyes - relativeR)*W_head + w*w.transpose()*tau).inverse()*relativeP;
                //ROS_INFO_STREAM("v: " << v.transpose());
            }

            // get twist
            //std::cout<<"vnormScale:";std::cin>>vnormScale;
            //std::cout<<"wnormScale:";std::cin>>wnormScale;
            vnormScale = 40;//40
            wnormScale = 10;//10


            v = v/(vnormScale*v.norm());
            if(w.norm() != 0) 
              w = w/(wnormScale*w.norm());
            else 
              w << 0.0, 0.0, 0.0;
            twist << v, w;

            /*****************************************************************
            *                           Update Twist                         *
            *****************************************************************/
            ROS_INFO_STREAM("twist: " << twist.transpose());

            jointsVelocity = jacPseudoInv * twist;
            newJoints = initialJoints + jacPseudoInv * twist;

            //update joint values
            std_msgs::Float64 val_1;
            std_msgs::Float64 val_2;
            std_msgs::Float64 val_3;
            std_msgs::Float64 val_4;
            std_msgs::Float64 val_5;
            std_msgs::Float64 val_6;
            std_msgs::Float64 val_7;
            val_1.data = newJoints[0];
            val_2.data = newJoints[1];
            val_3.data = newJoints[2];
            val_4.data = newJoints[3];
            val_5.data = newJoints[4];
            val_6.data = newJoints[5];
            val_7.data = newJoints[6];//7J
            //val_8.data = newJoints[7];

            pub_1.publish(val_1);
            pub_2.publish(val_2);
            pub_3.publish(val_3);
            pub_4.publish(val_4);
            pub_5.publish(val_5);
            pub_6.publish(val_6);
            pub_7.publish(val_7); //7J 
            //pub_8.publish(val_8); 

            double maxLength = 0.0;
            double waitTime  = 0.0;
            for(size_t idx = 0; idx < 7; idx++){//7J
                if(std::abs(jointsVelocity[idx]) > maxLength) maxLength = std::abs(jointsVelocity[idx]);
            }
            waitTime = maxLength/0.5;
            waitTime = waitTime + 0.05; // second
            unsigned int microseconds = waitTime * 1000000; // microsecond

            usleep(microseconds);
            

                        
            /*****************************************************************
            *                       Stopping Criteria                        *
            *****************************************************************/
            //get current eef
            kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
            const Eigen::Affine3d &end_effector_state1 = kinematic_state->getGlobalLinkTransform("link_8");//7J
            Eigen::Quaterniond quaternion1(end_effector_state1.rotation());
            currentLocation << end_effector_state1.translation();
            //currentPoseG = end_effector_state1.matrix();
                        //, end_effector_state1.rotation().eulerAngles(0,1,2);

            //compute difftau 
            //relativePoseG = finalPoseG*currentPoseG.inverse();
            //relativeR = relativePoseG.block(0,0,3,3);
            //traceR = relativeR(0,0) + relativeR(1,1) + relativeR(2,2);
            //difftau = acos(0.5*(traceR - 1));
            //ROS_INFO_STREAM("Rotation difference: " << difftau);

            //compute diffs and norms
            diff_angle = fabs(quaternion1.x() * q1 + quaternion1.y() * q2 + quaternion1.z() * q3 + quaternion1.w() * q0);
            if (diff_angle > 1.0 - 1e-9)
              diff_angle = 0.0;
            else
              diff_angle = acos(diff_angle);

            diff_pos = ((finalLocation - currentLocation).block(0,0,3,1)).norm();
            //diff = diff_pos * 0.95 + diff_angle * 0.05;

            //diffheight = ((finalLocation - currentLocation).block(2,0,1,1)).norm();
            ROS_INFO_STREAM("Location difference for position: " << diff_pos);
            ROS_INFO_STREAM("Location difference for orientation: " << diff_angle);
            //ROS_INFO_STREAM("Location height difference: " << diffheight);
            ROS_INFO_STREAM("Location final  : " << finalLocation.transpose());
            ROS_INFO_STREAM("Location current: " << currentLocation.transpose());


    }
    ROS_INFO_STREAM("done" << '\n');
    sleep(1);
}

double distance_se3(Eigen::Vector3d initialLocation, Eigen::Quaterniond Ini_orientation, Eigen::Vector3d finalLocation, Eigen::Quaterniond Final_orientation, double threshold){
    double dist1, dist2;
    dist1 = (finalLocation-initialLocation).norm();
        
    /*          Method 1  (from OMPL)       */
    dist2 = fabs(Ini_orientation.x() * Final_orientation.x() + Ini_orientation.y() * Final_orientation.y() + Ini_orientation.z() * Final_orientation.z() + Ini_orientation.w() * Final_orientation.w());
    if (dist2 > 1.0 - 1e-9)
        dist2 = 0.0;
    else
        dist2 = acos(dist2);

    if(dist1 > threshold)
        return dist1;
    else
        return dist2;
}

Eigen::Quaterniond r2q(Eigen::Matrix3d rotation){
  Eigen::Quaterniond q;
  q.w() = sqrt(1.0 + rotation(0,0) + rotation(1,1) + rotation(2,2)) / 2.0;
  q.x() = (rotation(2,1) - rotation(1,2)) / (4.0 * q.w());
  q.y() = (rotation(0,2) - rotation(2,0)) / (4.0 * q.w());
  q.z() = (rotation(1,0) - rotation(0,1)) / (4.0 * q.w());
  return q;
}

    /*****************************************************************
    *                     Specify Cartesian Paths                    *
    *****************************************************************/
    /*
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(target_pose1);  // starting from target_pose1
    target_pose1.position.z -= 0.1;
    waypoints.push_back(target_pose1);  // down
    target_pose1.position.z -= 0.1;
    waypoints.push_back(target_pose1);  // down
    target_pose1.position.z -= 0.1;
    waypoints.push_back(target_pose1);  // down

    // We want the cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively
    // disabling it.
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);// eef_step // jump_threshold

    std::cout<<"flag1"<<std::endl;
    moveit::planning_interface::MoveGroup::Plan my_plan_Cartesian;
    my_plan_Cartesian.start_state_.joint_state.name.push_back("joint_1");
    my_plan_Cartesian.start_state_.joint_state.name.push_back("joint_2");
    my_plan_Cartesian.start_state_.joint_state.name.push_back("joint_3");
    my_plan_Cartesian.start_state_.joint_state.name.push_back("joint_4");
    my_plan_Cartesian.start_state_.joint_state.name.push_back("joint_5");
    my_plan_Cartesian.start_state_.joint_state.name.push_back("joint_6");
    my_plan_Cartesian.start_state_.joint_state.name.push_back("joint_7");
    my_plan_Cartesian.start_state_.joint_state.name.push_back("joint_8");
    std::cout<<"flag2"<<std::endl;
    my_plan_Cartesian.start_state_.joint_state.position.push_back(group.getCurrentJointValues()[0]);
    my_plan_Cartesian.start_state_.joint_state.position.push_back(group.getCurrentJointValues()[1]);
    my_plan_Cartesian.start_state_.joint_state.position.push_back(group.getCurrentJointValues()[2]);
    my_plan_Cartesian.start_state_.joint_state.position.push_back(group.getCurrentJointValues()[3]);
    my_plan_Cartesian.start_state_.joint_state.position.push_back(group.getCurrentJointValues()[4]);
    my_plan_Cartesian.start_state_.joint_state.position.push_back(group.getCurrentJointValues()[5]);
    my_plan_Cartesian.start_state_.joint_state.position.push_back(group.getCurrentJointValues()[6]);
    my_plan_Cartesian.start_state_.joint_state.position.push_back(-1.0);
    std::cout<<"flag3"<<std::endl;
    std::cout<<my_plan_Cartesian.start_state_<<std::endl;
    my_plan_Cartesian.trajectory_ = trajectory;
    my_plan_Cartesian.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    std::cout<<my_plan_Cartesian.trajectory_<<std::endl;
    compensatePlan(my_plan_Cartesian);

    // visualization
    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);    
    sleep(5.0);

    // execution
    if(fraction > 0.5) group.execute(my_plan_Cartesian);
    sleep(5.0);
    //group.clearPathConstraints();
*/