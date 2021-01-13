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

/* Author: Ruinian Xu */

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <dynamixel_msgs/JointState.h>
#include <dynamixel_msgs/PositionVelocity.h>

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

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

bool mutex = 0;
std::vector<double> current_joint_values;
std::vector<double> detection_result;

const double gripper_open_value = -0.7; //open value of 8th motor
const double gripper_close_value = 0.1; //close value of 8th motor
//const double gripper_open_value = -0.65;
const double gripper_giant_close_value = 0.0;
const double gripper_slight_close_value = 0.1;
const double gripper_mild_close_value = 0.2;
const double gripper_hard_close_value = 0.35;
const double gripper_extreme_close_value = 0.4;

void mutex_traj();
void traj_end_mutex(const std_msgs::Bool::ConstPtr& msg);
void openGrabber(ros::Publisher &pub_8, ros::Publisher &pub_9);
void closeGrabber(ros::Publisher &pub_8, ros::Publisher &pub_9);
double rotateGripper(ros::Publisher &pub_7, double angle);
void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg);
void detection_handler(const std_msgs::Float64MultiArray::ConstPtr& msg);
const std::vector<std::string> listNamedPoses(moveit::planning_interface::MoveGroup &group);
void gotoNamedTarget(moveit::planning_interface::MoveGroup &group, std::string target, bool constraint_on);
void addObject2Scene(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher);
void addBoxObjects(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher, std::string object_name, double x, double y, double z, double w, double h, double d);
void getConstraint(moveit::planning_interface::MoveGroup &group, std::string target);
void cut(moveit::planning_interface::MoveGroup &group, double x_g, double y_g, double z_g, double angle_g, double x_c, double y_c, double x_1, double y_1, double x_2, double y_2, ros::Publisher &pub_7, ros::Publisher &pub_8, ros::Publisher &pub_9);

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
    // add washing machine into the planning scene
    addBoxObjects(group_arm, planning_scene_interface ,collision_object_publisher, "box", 0.27, -0.23, 0.06, 0.18, 0.18, 0.23);
  
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


    //set 8th and 9th motor to zero to avoid collision
    std_msgs::Float64 val_8;
    std_msgs::Float64 val_9;
    val_8.data = 0;
    val_9.data = 0;
    pub_8.publish(val_8);
    pub_9.publish(val_9);


    std::string target = ""; 
    int targetNum = 0;
    std::cout<<"select target pose above: "; std::cin >> targetNum;
    if(targetNum == 0) target = "Home";
    else target = "JoyStick";
    gotoNamedTarget(group_arm, target, 0);

    while (1){
        if (detection_result.size() != 0){
            double x_g, y_g, z_g, angle_g, x_c, y_c, x_c_1, y_c_1, x_c_2, y_c_2;
            
            x_g = detection_result[0];
            y_g = detection_result[1];
            z_g = detection_result[2];
            angle_g = detection_result[3];
            x_c = detection_result[4];
            y_c = detection_result[5];
            x_c_1 = detection_result[6];
            x_c_1 = detection_result[7];
            x_c_2 = detection_result[8];
            x_c_2 = detection_result[9];

            cut(group_arm, x_g, y_g, z_g, angle_g, x_c, y_c, x_c_1, y_c_1, x_c_2, y_c_2, pub_7, pub_8, pub_9);

            gotoNamedTarget(group_arm, target, 0);

            break;
        }
        
    }

    ros::spin();
    return 0;
}	

void cut(moveit::planning_interface::MoveGroup &group, double x_g, double y_g, double z_g, double angle_g, double x_c, double y_c, double x_1, double y_1, double x_2, double y_2, ros::Publisher &pub_7, ros::Publisher &pub_8, ros::Publisher &pub_9) {
    // predefined the position of the string
    double x_string, y_string, z_string;
    x_string = 0.35;
    y_string = -0.15;
    z_string = 0.065 + 0.03;

	/**************************************************
	*               Go to grasping position           *
	***************************************************/
	ROS_INFO("Go to Grasping Position");
	geometry_msgs::Pose target_pose_pickup;

    target_pose_pickup.position.x = x_g;//0.4;
    target_pose_pickup.position.y = y_g;//0.0;
    target_pose_pickup.position.z = z_g + 0.15;//0.2;, input actual Z of object instead of approach
    tf::Quaternion qat_pick = tf::createQuaternionFromRPY(0, -M_PI, -angle_g);
    qat_pick.normalize();
    target_pose_pickup.orientation.x = qat_pick.x();//0.577;//0.49; // two-sided gribber
    target_pose_pickup.orientation.y = qat_pick.y();//0.577;//0.49; // two-sided gribber
    target_pose_pickup.orientation.z = qat_pick.z();//0.577;//0.49;
    target_pose_pickup.orientation.w = qat_pick.w(); 

    // set target pose
    group.setPoseTarget(target_pose_pickup);
    // plan
    moveit::planning_interface::MoveGroup::Plan my_plan;
    // bool success = group.plan(my_plan); MOVEIT_EDIT
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    //compensate_slark(my_plan); //compensate the slark for 2nd motor
    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    // execute
    std::cout<<my_plan.trajectory_.joint_trajectory<<std::endl;
    if(success) group.execute(my_plan);
    mutex_traj();

    /****************************************************************
    *                          Rotate wrist                         *
    ****************************************************************/
    // ROS_INFO("Rotate wrist");
    // rotateGripper(pub_7, angle_g);

    /*****************************************************************
    *                          Open grabber                          *
    *****************************************************************/
    ROS_INFO("Open gripper");
    openGrabber(pub_8, pub_9);

    
    /****************************************************************
    *                Move down to pick up by jacobian               *
    *****************************************************************/
    ROS_INFO("Come down");
    openGrabber(pub_8, pub_9);
    std::vector<geometry_msgs::Pose> waypoints;
    robot_state::RobotStatePtr kinematic_state = group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName());
    kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
    const Eigen::Affine3d &temp_down = kinematic_state->getGlobalLinkTransform("link_eef");

    geometry_msgs::Pose pose;
    pose.position.x = temp_down.translation()(0);
    pose.position.y = temp_down.translation()(1);
    pose.position.z = temp_down.translation()(2) - 0.15; //0.14
    tf::Quaternion qat_down = tf::createQuaternionFromRPY(0, -M_PI, -angle_g);
    qat_down.normalize();
    pose.orientation.x = qat_down.x();
    pose.orientation.y = qat_down.y();
    pose.orientation.z = qat_down.z();
    pose.orientation.w = qat_down.w();
    waypoints.push_back(pose);

    //group.setMaxVelocityScalingFactor(0.1);
    moveit_msgs::RobotTrajectory trajectory_down;
    const double jump_threshold = 0.0;
    double eef_step = 0.01;
    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_down);
    for (int i=0; i < trajectory_down.joint_trajectory.points.size(); i++)
        trajectory_down.joint_trajectory.points[i].positions[6] = trajectory_down.joint_trajectory.points[0].positions[6];

    moveit::planning_interface::MoveGroup::Plan plan_down;
    plan_down.trajectory_ = trajectory_down;
    group.execute(plan_down);

    /*****************************************************************
    *                          Close grabber                         *
    *****************************************************************/
    ROS_INFO("Close gripper");
    closeGrabber(pub_8, pub_9);

    /*****************************************************************
    *        Determine if it is neccessary to rotate                 *
    *****************************************************************/
    bool flag_flip = false;
    if (x_1 > x_2)
        flag_flip = true;

    /*****************************************************************
    *                   Move to the cup location                     *
    *****************************************************************/
    // 1. location 2. orientation
    double final_angle;
    tf::Quaternion qat;
    geometry_msgs::Pose pose_cut;
    pose_cut.position.z = z_string;
    // case 1: blade is at the right to the handle
    if (y_c < y_g){
        final_angle = 0;
        qat = tf::createQuaternionFromRPY(0, -M_PI, 0);
        // case 1: right top
        if (x_c > x_g){
            if (y_1 > y_2){
                pose_cut.position.x = x_string + 0.07;
                pose_cut.position.y = y_string + 0.05 + sqrt(pow(x_c-x_g,2)+pow(y_c-y_g,2));
            }
            else{
                pose_cut.position.x = x_string - 0.03;
                pose_cut.position.y = y_string + 0.05 + sqrt(pow(x_c-x_g,2)+pow(y_c-y_g,2));
            }
        }
        // case 2: right bottom
        else{
            if (y_1 < y_2){
                pose_cut.position.x = x_string + 0.07;
                pose_cut.position.y = y_string + 0.05 + sqrt(pow(x_c-x_g,2)+pow(y_c-y_g,2));
            }
            else{
                pose_cut.position.x = x_string - 0.03;
                pose_cut.position.y = y_string + 0.05 + sqrt(pow(x_c-x_g,2)+pow(y_c-y_g,2));
            }
        }
        
    }
    // case 2: blade is at the left to the handle
    else{
        final_angle = M_PI;
        qat = tf::createQuaternionFromRPY(0, -M_PI, M_PI);
        // case 1: left top
        if (x_c > x_g){
            if (y_1 > y_2){
                pose_cut.position.x = x_string + 0.07;
                pose_cut.position.y = y_string + 0.05 + sqrt(pow(x_c-x_g,2)+pow(y_c-y_g,2));
            }
            else{
                pose_cut.position.x = x_string - 0.03;
                pose_cut.position.y = y_string + 0.05 + sqrt(pow(x_c-x_g,2)+pow(y_c-y_g,2));
            }
        }
        // case 2: left bottom
        else{
            if (y_1 < y_2){
                pose_cut.position.x = x_string + 0.07;
                pose_cut.position.y = y_string + 0.05 + sqrt(pow(x_c-x_g,2)+pow(y_c-y_g,2));
            }
            else{
                pose_cut.position.x = x_string - 0.03;
                pose_cut.position.y = y_string + 0.05 + sqrt(pow(x_c-x_g,2)+pow(y_c-y_g,2));
            }
        }
    }
    qat.normalize();
    pose_cut.orientation.x = qat.x();
    pose_cut.orientation.y = qat.y();
    pose_cut.orientation.z = qat.z();
    pose_cut.orientation.w = qat.w();


    std::cout<<pose_cut<<std::endl;
    std::cout<<final_angle<<std::endl;

    // set target pose
    ROS_INFO("Move to cut place");
    group.setPoseTarget(pose_cut);
    // plan
    moveit::planning_interface::MoveGroup::Plan plan_c;
    // bool success = group.plan(my_plan); MOVEIT_EDIT
    moveit::planning_interface::MoveItErrorCode success_c = group.plan(plan_c);
    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success_c?"":"FAILED");
    // execute
    if(success_c) group.execute(plan_c);
    mutex_traj();

    bool pause;
    std::cout<<"pause"<<std::endl;
    std::cin>>pause;
    

    /*****************************************************************
    *                          Cut action                            *
    *****************************************************************/
    ROS_INFO("Cut");
    moveit_msgs::RobotTrajectory trajectory_cut;
    std::vector<geometry_msgs::Pose> waypoints_cut;
    
    if (pose_cut.position.x > x_string){
        pose_cut.position.y -= 0.05;
        waypoints_cut.push_back(pose_cut);
        pose_cut.position.x -= 0.05;
        waypoints_cut.push_back(pose_cut);
        pose_cut.position.x -= 0.08;
        waypoints_cut.push_back(pose_cut);
        
    }
    else{
        pose_cut.position.x += 0.05;
        waypoints_cut.push_back(pose_cut);
        pose_cut.position.y -= 0.05;
        waypoints_cut.push_back(pose_cut);
        pose_cut.position.x += 0.08;
        waypoints_cut.push_back(pose_cut);
    }

    
    eef_step = 0.025;
    fraction = group.computeCartesianPath(waypoints_cut, eef_step, jump_threshold, trajectory_cut);
    for (int i=0; i < trajectory_cut.joint_trajectory.points.size(); i++)
        trajectory_cut.joint_trajectory.points[i].positions[6] = trajectory_cut.joint_trajectory.points[0].positions[6];

    moveit::planning_interface::MoveGroup::Plan plan_cut;
    plan_cut.trajectory_ = trajectory_cut;
    group.execute(plan_cut);
    mutex_traj();

    ROS_INFO("Open gripper");
    openGrabber(pub_8, pub_9);
}

void traj_end_mutex(const std_msgs::Bool::ConstPtr& msg){
    if (msg->data){
        mutex = 1;
    }
}

void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg)
{
    current_joint_values.clear();
    for(std::size_t i = 0; i < msg->position.size(); ++i) {
        current_joint_values.push_back(msg->position[i]);
    }
}

void grasping_detection_handler(const std_msgs::Float64MultiArray::ConstPtr& msg){
    detection_result.clear();
    for (std::size_t i = 0; i < msg->data.size(); ++i) {
        detection_result.push_back(msg->data[i]);
    }
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
    // bool success = group.plan(my_plan); MOVEIT_EDIT
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

    // set planning time to default
    group.setPlanningTime(20.0);
    group.clearPathConstraints();

    // visualization
    ROS_INFO("Visualizing plan home (joint space goal) %s",success?"":"FAILED");
    //sleep(5.0);    
 
    // execution
    ROS_INFO("Execution if plan successfully");
    if(success) group.execute(my_plan);
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

void addBoxObjects(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher, std::string object_name, double x, double y, double z, double w, double h, double d){
    /* Define the object message */
    moveit_msgs::CollisionObject object;

    /* The header must contain a valid TF frame */
    object.header.frame_id = group.getPlanningFrame();
    /* The id of the object */
    object.id = object_name;

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x =  x;
    pose.position.y =  y;
    pose.position.z = z;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = w;
    primitive.dimensions[1] = h;
    primitive.dimensions[2] = d;

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

void addObject2Scene(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher){
    /* Define the object message */
    moveit_msgs::CollisionObject object;

    /* The header must contain a valid TF frame */
    object.header.frame_id = group.getPlanningFrame();
    /* The id of the object */
    object.id = "table";

    /* A default pose */
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x =  0.5;
    pose.position.y =  0.0;
    pose.position.z = -0.05;//-0.13;//-0.1;//-0.07

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.9;
    primitive.dimensions[1] = 1.8;
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

void openGrabber(ros::Publisher &pub_8, ros::Publisher &pub_9){
    std_msgs::Float64 val_8;
    std_msgs::Float64 val_9;
    val_8.data = gripper_open_value;
    val_9.data = -gripper_open_value;
    pub_8.publish(val_8);
    pub_9.publish(val_9);
    sleep(1.5);
}

double rotateGripper(ros::Publisher &pub_7, double angle) {
	ROS_INFO("Rotate gripper");
    std_msgs::Float64 val;

    val.data = current_joint_values[6] + angle;
    
    pub_7.publish(val);
    sleep(2.0);
}

void closeGrabber(ros::Publisher &pub_8, ros::Publisher &pub_9) {
	ROS_INFO("Close gripper");
    std_msgs::Float64 val_8 ;
    std_msgs::Float64 val_9 ;
    val_8.data = gripper_hard_close_value; 
    val_9.data = -gripper_open_value;
    pub_8.publish(val_8);
    pub_9.publish(val_9);

    sleep(2.5);
}

