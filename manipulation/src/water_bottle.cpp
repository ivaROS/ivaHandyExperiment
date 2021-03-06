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
#include <dynamixel_msgs/JointState.h>

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

std::vector<double> current_joint_values;
bool mutex = 0;
double load_8 = 0;
const double pi = std::acos(-1);
//const double pi = 3.1415926535897;
const double joint_8_home_value_parallel = 0.0;
const double gripper_close_value_parallel = -2.6;
const double joint_8_home_value_paw= -0.5;
const double gripper_close_value_paw = 1.0;
const double parallel_torque_compensate = 0.1;
const double paw_torque_compensate = 0.1;
geometry_msgs::Pose target_pickup;

void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg);
void traj_end_mutex(const std_msgs::Bool::ConstPtr& msg);
void update_state(const dynamixel_msgs::JointState::ConstPtr& msg);
void mutex_traj();
void mutex_rotate(double target_angle);

void openGrabber(ros::Publisher &pub_8, double angle);
void closeGrabber(ros::Publisher &pub_8, double angle);
void rotateGripper(ros::Publisher &pub_7, double angle);

void read_para_bottle(double &x, double &y, double &z);
void read_para_mug(double &x, double &y, double &z);
std::string read_command(std::string &command);

void addObject2Scene(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher);
const std::vector<std::string> listNamedPoses(moveit::planning_interface::MoveGroup &group);
void PourWater(moveit::planning_interface::MoveGroup &group, ros::Publisher &pub_7, ros::Publisher &pub_8);
void gotoNamedTarget(moveit::planning_interface::MoveGroup &group, std::string target, bool constraint_on, double joint_8_value);
void getConstraint(moveit::planning_interface::MoveGroup &group, std::string target);
void compensatePlan(moveit::planning_interface::MoveGroup::Plan &plan, double joint_8_value);
Eigen::Quaterniond toQuaternion(double roll, double pitch, double yaw);
void Jacobian_hybrid(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose &target_pose, std::vector<double> &target_joint_values, double diff_threshold);
double distance_se3_position(Eigen::Vector3d initialLocation, Eigen::Vector3d finalLocation);
double distance_se3_orientation(Eigen::Quaterniond Ini_orientation, Eigen::Quaterniond Final_orientation);

int main(int argc, char **argv){
	/*****************************************************************
    *                         Arm initialization                     *
    *****************************************************************/
    // ros initialization
    ros::init(argc, argv, "arm_kinematics");
    ros::NodeHandle node_handle;
    ros::Publisher pub_7 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_7/command", 1, true);
    ros::Publisher pub_8 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_8/command", 1, true);
    ros::Subscriber sub = node_handle.subscribe<sensor_msgs::JointState>("/joint_states", 1, joint_state_handler);
    ros::Subscriber sub_endtime = node_handle.subscribe<std_msgs::Bool>("/finalasm_joint_trajectory_action_controller/mutex", 5, traj_end_mutex);
    ros::Subscriber sub_joint_8_state = node_handle.subscribe<dynamixel_msgs::JointState>("/finalasm_position_controller_8/state", 1, update_state);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // kinematic_state & kinematic_model loading & planner
    moveit::planning_interface::MoveGroup group("arm4");  //"arm4_full"
    group.setPlannerId("LBKPIECEkConfigDefault");//ForageRRTkConfigDefault//LBKPIECEkConfigDefault//RRTstarkConfigDefault

    // We will use the planning_scene_interface class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 

    /*    Joint and link info    */
    const std::vector<std::string> &joint_names = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName())->getJointModelNames();
    const std::vector<std::string> &link_names = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName())->getLinkModelNames();

    /*****************************************************************
    *                       Visualization setup                      *
    *****************************************************************/
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
    //addObject2Scene(group, planning_scene_interface ,collision_object_publisher);

    /*****************************************************************
    *                        List stored poses                       *
    *****************************************************************/
    const std::vector<std::string> namedTargets = listNamedPoses(group);

    /*****************************************************************
    *                      Specify initial pose                      *
    *****************************************************************/
    // namedTargets stores the names of pre-defined poses(joint values)
    // select the name (ex. "home"), gotoNamedTarget function will find plan to the specific pose
    int targetNum = 0;
    std::cout<<"select target pose above: (1:Home 2:Holding 3:Give_side)"; std::cin >> targetNum;
    gotoNamedTarget(group, namedTargets[targetNum], 0, joint_8_home_value_paw);
    
    /*********************************************************************************
    *     Pipeline: 1.Go to grab the water bottle 2.Move to the task position        *
    *     3.Rotate the wrist to pour the water 4.Rotate the wrist back               *
    *     5.Place the bottle 6.Back to home position                                 *
    **********************************************************************************/
    int enable = 1;
    while(enable){
        //Pickup the bottle where the motion is consisted of go to grasping position, grasp object and move to task position
        PourWater(group, pub_7, pub_8);

        //Back to home position
        gotoNamedTarget(group, namedTargets[targetNum], 0, joint_8_home_value_paw);

        std::cout<<"Would you like to repeat task? 1(Yes) 2(No)"<<std::endl;
        std::cin >> enable;
    }

    ros::spin();
    return 0;
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

void openGrabber(ros::Publisher &pub_8, double angle){
    std_msgs::Float64 val_8 ;
    val_8.data = angle;
    pub_8.publish(val_8);
    double sleep_time = std::abs(angle - current_joint_values.back()) / 0.85 * 2.5;
    sleep(sleep_time);
}

void rotateGripper(ros::Publisher &pub_7, double angle) {
    std_msgs::Float64 val_7;

    //std::cout<<" current value of seventh joint: "<<current_joint_values[6]<<std::endl;
    //std::cout<<" compensate value: "<<std::abs(angle - 180.00) / 180.00 * pi<<std::endl;

    if ((current_joint_values[6] + (angle / 180.00 * pi)) > 3.14){
        val_7.data = current_joint_values[6] - (std::abs(angle - 360.00) / 360.00 * 2 * pi);
        pub_7.publish(val_7);
        mutex_rotate(val_7.data);
    }
    else if ((current_joint_values[6] + (angle / 180.00 * pi)) < -3.14){
        val_7.data = current_joint_values[6] + (std::abs(angle - 360.00) / 360.00 * 2 * pi);
        pub_7.publish(val_7);
        mutex_rotate(val_7.data);
    }
    else{
        val_7.data = current_joint_values[6] + (angle / 180 * pi);
        pub_7.publish(val_7);
        mutex_rotate(val_7.data);
    }
}

void closeGrabber(ros::Publisher &pub_8, double angle){
    std_msgs::Float64 val_8 ;
    val_8.data = angle;
    pub_8.publish(val_8);

    while(1){
        if(std::abs(load_8) > 0.6){
            val_8.data = current_joint_values.back() + paw_torque_compensate;
            pub_8.publish(val_8);
            break;
        }
    }
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
    pose.position.z = -0.28;//-0.1;//-0.07

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

void PourWater(moveit::planning_interface::MoveGroup &group, ros::Publisher &pub_7, ros::Publisher &pub_8){
	/**************************************************
    *               Go to grasping position           *
    ***************************************************/
    ROS_INFO("Go to Grasping Position");
    //Get grasping position from vision part
    double x, y, z;
    x = 0.7; y = 0.15; z = -0.12;
    //read_para_bottle(x, y, z);

    //Read the position of mug
    double x_task = 0.65, y_task = -0.05, z_task = 0.03; //z_task height of mug
    //read_para_mug(x_task, y_task, z_task);

    double angle; //angle between the bottle and gripper negative left-sdie and positive right-side
    angle = std::atan(y / x) * 180.0 / pi;
    //0.165m distance between 7th motor and the top of gripper
    double len_gripper = 0.165; 

    /************Approach the bottle **************/
    ////////////////////////////////////////////////
    ////////////////////////////////////////////////
    geometry_msgs::Pose target_pose_approach;
    target_pose_approach.position.x = x - len_gripper - 0.1; // - std::cos(angle / 180.0 * pi) * len_gripper;
    target_pose_approach.position.y = y;// - std::sin(angle / 180.0 * pi) * len_gripper;
    target_pose_approach.position.z = z;//(z / 2.0) - 0.20;
    
    //Eigen::Quaterniond ori = toQuaternion((-90.0 - angle) / 180.0 * pi, 90.0 / 180.0 * pi, 0.0);
    target_pose_approach.orientation.x = -0.5;//ori.x();//0.577;//0.49; // two-sided gribber
    target_pose_approach.orientation.y = 0.5;//ori.y();//0.577;//0.49; // two-sided gribber
    target_pose_approach.orientation.z = 0.5;//ori.z();//0.577;//0.49;
    target_pose_approach.orientation.w = 0.5;//ori.w();

    group.setPoseTarget(target_pose_approach);
    // plan
    moveit::planning_interface::MoveGroup::Plan my_plan_approach;
    bool success_approach = group.plan(my_plan_approach);
    my_plan_approach.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    compensatePlan(my_plan_approach, joint_8_home_value_paw);
    
    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success_approach?"":"FAILED");  

    //execution  
    if(success_approach) group.execute(my_plan_approach);
    mutex_traj();

    /**********Pick the bottle************/
    ////////////////////////////////////////
    ////////////////////////////////////////
    geometry_msgs::Pose target_pose_pick;
    target_pose_pick.position.x = x - len_gripper; // - std::cos(angle / 180.0 * pi) * len_gripper;
    target_pose_pick.position.y = y;// - std::sin(angle / 180.0 * pi) * len_gripper;
    target_pose_pick.position.z = z;//(z / 2.0) - 0.20;
    
    //Eigen::Quaterniond ori = toQuaternion((-90.0 - angle) / 180.0 * pi, 90.0 / 180.0 * pi, 0.0);
    target_pose_pick.orientation.x = -0.5;//ori.x();//0.577;//0.49; // two-sided gribber
    target_pose_pick.orientation.y = 0.5;//ori.y();//0.577;//0.49; // two-sided gribber
    target_pose_pick.orientation.z = 0.5;//ori.z();//0.577;//0.49;
    target_pose_pick.orientation.w = 0.5;//ori.w();

    group.setPoseTarget(target_pose_pick);
    // plan
    moveit::planning_interface::MoveGroup::Plan my_plan_pick;
    bool success_pick = group.plan(my_plan_pick);
    my_plan_pick.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    compensatePlan(my_plan_pick, joint_8_home_value_paw);

    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success_pick?"":"FAILED");  

    //execution  
    if(success_pick) group.execute(my_plan_pick);
    mutex_traj();

    //setup for global target pickup
    target_pickup.position.x = target_pose_pick.position.x;
    target_pickup.position.y = target_pose_pick.position.y;
    target_pickup.position.z = target_pose_pick.position.z;
    target_pickup.orientation.x = target_pose_pick.orientation.x;
    target_pickup.orientation.y = target_pose_pick.orientation.y;
    target_pickup.orientation.z = target_pose_pick.orientation.z;
    target_pickup.orientation.w = target_pose_pick.orientation.w;

    //Debug
    /*
    std::vector<double> goal_joint;
    for (int i = 0; i < my_plan.trajectory_.joint_trajectory.points.back().positions.size(); ++i)
        goal_joint.push_back(my_plan.trajectory_.joint_trajectory.points.back().positions[i]);

    robot_state::RobotStatePtr kinematic_state = group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName());
    kinematic_state->setJointGroupPositions(joint_model_group, goal_joint);
    const Eigen::Affine3d &pose = kinematic_state->getGlobalLinkTransform("link_8");

    Eigen::VectorXd GoalLocation = pose.translation();
    Eigen::Quaterniond GoalOrientation(pose.rotation());
    std::cout<<"Goal position: \n"<<GoalLocation<<"\n"<<GoalOrientation.x()<<"\n"<<GoalOrientation.y()<<"\n"<<GoalOrientation.z()<<"\n"<<GoalOrientation.w()<<std::endl;

    std::cout<<"Goal joint configuration"<<std::endl;
    for (int i = 0; i < my_plan.trajectory_.joint_trajectory.points.back().positions.size(); ++i)
        std::cout<<my_plan.trajectory_.joint_trajectory.points.back().positions[i]<<std::endl;
    */


    /**************************************************
	*                  Close gripper                  *
	***************************************************/
    closeGrabber(pub_8, gripper_close_value_paw);

    /**************************************************
	*              Move to task position              *
	***************************************************/
	geometry_msgs::Pose target_pose_task;
    double compensate_cup = 0.115;

	target_pose_task.position.x = x_task - len_gripper + 0.01;
    if (y_task > 0)
        target_pose_task.position.y = y_task + compensate_cup;
    else
        target_pose_task.position.y = y_task + compensate_cup;
    target_pose_task.position.z = z_task;//(z_task / 2) - 0.26 + 0.1;
    target_pose_task.orientation.x = -0.50;//0.577;//0.49; // two-sided gribber
    target_pose_task.orientation.y = 0.50;//0.577;//0.49; // two-sided gribber
    target_pose_task.orientation.z = 0.50;//0.577;//0.49;
    target_pose_task.orientation.w = 0.50;

    group.setPoseTarget(target_pose_task);
    // plan
    moveit::planning_interface::MoveGroup::Plan my_plan_task;
    bool success_task = group.plan(my_plan_task);
    my_plan_task.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    compensatePlan(my_plan_task, current_joint_values[7] + paw_torque_compensate);

    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success_task?"":"FAILED");  

    //execution  
    if(success_task) group.execute(my_plan_task);
    mutex_traj();

    /*************Rotate the gripper and sleep for while and rotate it back**************/
    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////
    if (y > y_task){
        if ((current_joint_values[6] + (120.0 / 90.00 * pi / 2)) > 3.14){
            double motor_value_back = current_joint_values[6];
            std_msgs::Float64 val_7;
            val_7.data = 3.14;
            pub_7.publish(val_7);

            //sleep for a while
            sleep(2);

            val_7.data = motor_value_back;
            pub_7.publish(val_7);
        }
        else{
            rotateGripper(pub_7, 120.0);

            //sleep for a while
            sleep(2);

            rotateGripper(pub_7, -120.0);
        }
    }  
    else
        if ((current_joint_values[6] + (120.0 / 90.00 * pi / 2)) < -3.14){
            double motor_value_back = current_joint_values[6];
            std_msgs::Float64 val_7;
            val_7.data = -3.14;
            pub_7.publish(val_7);

            //sleep for a while
            sleep(2);

            val_7.data = motor_value_back;
            pub_7.publish(val_7);
        }
        else{
            rotateGripper(pub_7, -120.0);

            //sleep for a while
            sleep(2);

            rotateGripper(pub_7, 120.0);
        }

    /***********   Place the bottle   ************/
    ///////////////////////////////////////////////
    ///////////////////////////////////////////////
    group.setPoseTarget(target_pickup);
    // plan
    moveit::planning_interface::MoveGroup::Plan my_plan_place;
    bool success_place = group.plan(my_plan_place);
    my_plan_place.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    compensatePlan(my_plan_place, current_joint_values[7] + paw_torque_compensate);

    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success_place?"":"FAILED");  

    //execution  
    if(success_place) group.execute(my_plan_place);
    mutex_traj();

    openGrabber(pub_8, joint_8_home_value_paw);

}

void Place(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose target_pickup, ros::Publisher &pub_8){
	group.setPoseTarget(target_pickup);
    // plan
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    my_plan.trajectory_.joint_trajectory.joint_names.push_back("joint_8");
    compensatePlan(my_plan, current_joint_values[7] + paw_torque_compensate);

    // visualization
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");  

    //execution  
    if(success) group.execute(my_plan);
    mutex_traj();

    openGrabber(pub_8, joint_8_home_value_paw);
}


void compensatePlan(moveit::planning_interface::MoveGroup::Plan &plan, double joint_8_value){
    for(size_t idx = 0; idx < plan.trajectory_.joint_trajectory.points.size(); idx++){
        plan.trajectory_.joint_trajectory.points[idx].positions.push_back(joint_8_value);
    }
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

void gotoNamedTarget(moveit::planning_interface::MoveGroup &group, std::string target, bool constraint_on, double joint_8_value){
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
    compensatePlan(my_plan, joint_8_value);

    // set planning time to default
    group.setPlanningTime(20.0);
    group.clearPathConstraints();

    // visualization
    ROS_INFO("Visualizing plan home (joint space goal) %s",success?"":"FAILED");   
 
    // execution
    ROS_INFO("Execution if plan successfully");
    if(success) group.execute(my_plan);
    mutex_traj();
    

}

Eigen::Quaterniond toQuaternion(double roll, double pitch, double yaw) //sequence ZYX
{
	Eigen::Quaterniond q;
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w() = cy * cr * cp + sy * sr * sp;
	q.x() = cy * sr * cp - sy * cr * sp;
	q.y() = cy * cr * sp + sy * sr * cp;
	q.z() = sy * cr * cp - cy * sr * sp;
	return q;
}

void Jacobian_hybrid(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose &target_pose, std::vector<double> &target_joint_values, double diff_threshold = 0.03)
{
    // pose configuration
    Eigen::VectorXd initialJoints(7); // 7 by 1 vector //7J
    Eigen::VectorXd initialLocation(3); 
    Eigen::Matrix3d InitialRot;
        
    Eigen::VectorXd finalLocation(3); 
    Eigen::Matrix3d FinalRot;
    Eigen::Quaterniond Final_orientation;

    Eigen::Vector3d w(0.0,0.0,0.0);
    Eigen::Vector3d v(0.0,0.0,0.0);
    Eigen::MatrixXd twist_spatial = Eigen::MatrixXd::Zero(4,4);

    Eigen::VectorXd jointsVelocity;
    Eigen::VectorXd newJoints;

    //Setup for target pose configuration
    finalLocation << target_pose.position.x, target_pose.position.y, target_pose.position.z;
    double q11 = target_pose.orientation.x;
    double q12 = target_pose.orientation.y;
    double q13 = target_pose.orientation.z;
    double q10 = target_pose.orientation.w;
    Final_orientation.x() = target_pose.orientation.x;
    Final_orientation.y() = target_pose.orientation.y;
    Final_orientation.z() = target_pose.orientation.z;
    Final_orientation.w() = target_pose.orientation.w;

    FinalRot(0,0) = 1.0 - 2.0 * (q12 * q12 + q13 * q13);
    FinalRot(0,1) = 2.0 * (q11 * q12 - q10 * q13);
    FinalRot(0,2) = 2.0 * (q10 * q12 + q11 * q13);
    FinalRot(1,0) = 2.0 * (q11 * q12 + q10 * q13);
    FinalRot(1,1) = 1.0 - 2.0 * (q11 * q11 + q13 * q13);
    FinalRot(1,2) = 2.0 * (q12 * q13 - q10 * q11);
    FinalRot(2,0) = 2.0 * (q11 * q13 - q10 * q12);
    FinalRot(2,1) = 2.0 * (q10 * q11 + q12 * q13);
    FinalRot(2,2) = 1.0 - 2.0 * (q11 * q11 + q12 * q12);

    //Setup for initial pose configuration
    robot_state::RobotStatePtr kinematic_state = group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName());
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
    const Eigen::Affine3d &temp = kinematic_state->getGlobalLinkTransform("link_8");
    //kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
    initialJoints << current_joint_values[0], current_joint_values[1], current_joint_values[2], current_joint_values[3], current_joint_values[4], current_joint_values[5], current_joint_values[6];
    
    initialLocation = temp.translation();
    Eigen::Quaterniond Ini_orientation(temp.rotation());
    std::vector<double> ini_joint_values;
    double q21 = Ini_orientation.x();
    double q22 = Ini_orientation.y();
    double q23 = Ini_orientation.z();
    double q20 = Ini_orientation.w();
    InitialRot(0,0) = 1.0 - 2.0 * (q22 * q22 + q23 * q23);
    InitialRot(0,1) = 2.0 * (q21 * q22 - q20 * q23);
    InitialRot(0,2) = 2.0 * (q20 * q22 + q21 * q23);
    InitialRot(1,0) = 2.0 * (q21 * q22 + q20 * q23);
    InitialRot(1,1) = 1.0 - 2.0 * (q21 * q21 + q23 * q23);
    InitialRot(1,2) = 2.0 * (q22 * q23 - q20 * q21);
    InitialRot(2,0) = 2.0 * (q21 * q23 - q20 * q22);
    InitialRot(2,1) = 2.0 * (q20 * q21 + q22 * q23);
    InitialRot(2,2) = 1.0 - 2.0 * (q21 * q21 + q22 * q22);

    double dist_pos,dist_ori;
    dist_pos = distance_se3_position(initialLocation, finalLocation);
    dist_ori = distance_se3_orientation(Ini_orientation, Final_orientation);
  
    double step_size = 0.005;
    int count = 1;

    while(dist_pos > diff_threshold || dist_ori > diff_threshold) {
      /*****************************************************************
      *                 Joint values to inverse Jacobian               *
      *****************************************************************/
      Eigen::MatrixXd jacobian_all = Eigen::MatrixXd::Zero(6,8); // 6 by 8 to catch Jacobian matrix from ROS
      Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,7); // 6 by 7 Jacobian matrix (7 joints) //7J
      Eigen::MatrixXd jacPseudoInv; // 7 by 6 inverse Jacobian matrix (7 by 6 invJ* 6 by 1 twist = 7 by 1 new joint values )//7J
      Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
      // Get the Jacobian      
      kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames()[6]), reference_point_position, jacobian_all);//7J
      //std::string base_link = "base_link";
      //kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(base_link), reference_point_position, jacobian_all);

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
              //std::cout<<"singularity occurred!"<<std::endl;
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

      Eigen::VectorXd Trans_world = finalLocation - initialLocation;
        
      Eigen::Matrix3d ini_position_hat = Eigen::MatrixXd::Zero(3,3);
      ini_position_hat(0,1) = -1.0 * initialLocation(2);
      ini_position_hat(1,0) = initialLocation(2);
      ini_position_hat(0,2) = initialLocation(1);
      ini_position_hat(2,0) = -1.0 * initialLocation(1);
      ini_position_hat(1,2) = -1.0 * initialLocation(0);
      ini_position_hat(2,1) = initialLocation(0);
      Eigen::MatrixXd adj_g = Eigen::MatrixXd::Zero(6,6);
      adj_g.block(0,0,3,3) = InitialRot;
      adj_g.block(0,3,3,3) = ini_position_hat * InitialRot;
      adj_g.block(3,3,3,3) = InitialRot;
      //std::cout<<"initial position: \n"<<initialLocation<<std::endl;
      //std::cout<<"ini_position_hat: \n"<<ini_position_hat<<std::endl;
      //std::cout<<"transform adj_g: \n"<<adj_g<<std::endl;

      Eigen::MatrixXd T = Eigen::MatrixXd::Zero(6,6);
      T.block(0,0,3,3) = InitialRot.inverse();
      T.block(3,3,3,3) = Eigen::MatrixXd::Identity(3,3);
      //std::cout<<"InitialRot inverse: \n"<<InitialRot.inverse()<<std::endl;;
      //std::cout<<"transform T: \n"<<T<<std::endl;

      Eigen::VectorXd body_velocity(6), Trans_body(3);
      Trans_body = InitialRot.inverse() * Trans_world;
      body_velocity.block(0,0,3,1) = Trans_body;
      body_velocity.block(3,0,3,1) = Rot_body;


      Eigen::VectorXd spatial_velocity(6);
      
      body_velocity.block(0,0,3,1) = Trans_body.normalized() * step_size;
      spatial_velocity = adj_g * body_velocity;
      spatial_velocity.block(3,0,3,1) = spatial_velocity.block(3,0,3,1).normalized() * 2.0 * step_size;
      
      /*
      spatial_velocity.block(0,0,3,1) = Trans_world.normalized() * step_size;
      //Eigen::Vector3d spatial_trans;
      //spatial_trans << 
      //Rot_body << 0.0, 0.0, 0.0;
      //spatial_velocity.block(3,0,3,1) = Rot_body;
      spatial_velocity.block(3,0,3,1) = InitialRot * (Rot_body.normalized()) * step_size;
      */
      
      //Update Joint values                       
      jointsVelocity = jacPseudoInv * spatial_velocity;
      //std::cout<<"jointsVelocity"<<jointsVelocity<<std::endl;
      //jointsVelocity = jacPseudoInv * spatial_velocity;
      newJoints = initialJoints + jointsVelocity;

      /********************************************
      *           Stopping Criteria               *
      *********************************************/
      //Get next joint step by reading actual motor value
      /*
      initialJoints << current_joint_values[0], current_joint_values[1], current_joint_values[2], current_joint_values[3], current_joint_values[4], current_joint_values[5], current_joint_values[6];
      std::cout<<"Desired joint value: \n"<<newJoints<<std::endl;
      std::cout<<"Actual joint value: \n"<<initialJoints<<std::endl;
      kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);
      const Eigen::Affine3d &temp_eef = kinematic_state->getGlobalLinkTransform("link_8");
      initialLocation = temp_eef.translation();
      Eigen::Quaterniond current_orientation(temp_eef.rotation());
      q21 = current_orientation.x();
      q22 = current_orientation.y();
      q23 = current_orientation.z();
      q20 = current_orientation.w();
      */
      //Get the next joint step by simulation
      ini_joint_values.clear();
      for (int i = 0; i < 7; i++)
        ini_joint_values.push_back(newJoints(i));
      initialJoints << ini_joint_values[0], ini_joint_values[1], ini_joint_values[2], ini_joint_values[3], ini_joint_values[4], ini_joint_values[5], ini_joint_values[6];

      kinematic_state->setJointGroupPositions(joint_model_group, ini_joint_values);
      const Eigen::Affine3d &cur_eef = kinematic_state->getGlobalLinkTransform("link_8");
      initialLocation = cur_eef.translation();
      Eigen::Quaterniond current_orientation(cur_eef.rotation());
      q21 = current_orientation.x();
      q22 = current_orientation.y();
      q23 = current_orientation.z();
      q20 = current_orientation.w();
      InitialRot(0,0) = 1.0 - 2.0 * (q22 * q22 + q23 * q23);
      InitialRot(0,1) = 2.0 * (q21 * q22 - q20 * q23);
      InitialRot(0,2) = 2.0 * (q20 * q22 + q21 * q23);
      InitialRot(1,0) = 2.0 * (q21 * q22 + q20 * q23);
      InitialRot(1,1) = 1.0 - 2.0 * (q21 * q21 + q23 * q23);
      InitialRot(1,2) = 2.0 * (q22 * q23 - q20 * q21);
      InitialRot(2,0) = 2.0 * (q21 * q23 - q20 * q22);
      InitialRot(2,1) = 2.0 * (q20 * q21 + q22 * q23);
      InitialRot(2,2) = 1.0 - 2.0 * (q21 * q21 + q22 * q22);
      
      //update distance
      dist_pos = distance_se3_position(initialLocation, finalLocation);
      dist_ori = distance_se3_orientation(current_orientation, Final_orientation);

      count++;
      //int step_stop;
      //std::cin>>step_stop;
    }
    //std::cout<<"Jacobian found solution"<<std::endl;
    //std::cout<<"Total count: "<<count<<std::endl;
    //std::cout<<"Distance for orientation: "<<dist_ori<<std::endl;
    //std::cout<<"Distance for position: "<<dist_pos<<std::endl;
    //std::cout<<"Goal joint values: \n"<<initialJoints<<std::endl;

    //target_joint_values.clear();
    for (int i = 0; i < ini_joint_values.size(); ++i)
        target_joint_values[i] = ini_joint_values[i];
}

double distance_se3_position(Eigen::Vector3d initialLocation, Eigen::Vector3d finalLocation){
    double dist;
    dist = (finalLocation-initialLocation).norm();
        
    return dist;
}

double distance_se3_orientation(Eigen::Quaterniond Ini_orientation, Eigen::Quaterniond Final_orientation){
    double dist;
    dist = fabs(Ini_orientation.x() * Final_orientation.x() + Ini_orientation.y() * Final_orientation.y() + Ini_orientation.z() * Final_orientation.z() + Ini_orientation.w() * Final_orientation.w());
    if (dist > 1.0 - 1e-9)
        dist = 0.0;
    else
        dist = acos(dist);

    return dist;
}

void read_para_bottle(double &x, double &y, double &z){
    std::cout<<"Get the 3D positions of bottle"<<std::endl;
    bool debug = false;
    if (debug){
        // option A: default input
        std::cout<< "[debug mode].. "<<std::endl;
        x = 0.61; y = 0.0; z = 0.02;
        std::cout<<"[debug mode] x = "<<x<<std::endl;
        std::cout<<"[debug mode] y = "<<y<<std::endl;
        std::cout<<"[debug mode] z = "<<z<<std::endl;
    }
    else{
        // option B: YOLO input
        std::string cmd = "";
        read_command(cmd);
        while(read_command(cmd) != "grab"){
          std::string bbs_loadPath = "/home/ruinianxu/vision_position/3Dlocations_sodaBottle.txt";
          std::ifstream myfile(bbs_loadPath.c_str());
          std::string x_s, y_s, z_s;

          // find 3D bbs 
          if (myfile.is_open()){
            if(!getline (myfile, x_s))
               x_s = "0.5";
            if(!getline (myfile, y_s))
               y_s = "0";
            if(!getline (myfile,z_s))
               z_s = "0";

            myfile.close();
          }
          else 
            std::cout<<"file not open"<<std::endl;

          x = atof (x_s.c_str());
          y = atof (y_s.c_str());
          z = atof (z_s.c_str());
        }

        std::cout<<"reading.. x approach = "<<x<<std::endl;
        std::cout<<"reading.. y approach = "<<y<<std::endl;
        std::cout<<"reading.. z approach = "<<z<<std::endl;
    }
}

void read_para_mug(double &x, double &y, double &z){
    std::cout<<"Get the 3D positions of mug"<<std::endl;
    bool debug = false;
    if (debug){
        // option A: default input
        std::cout<< "[debug mode].. "<<std::endl;
        x = 0.61; y = 0.0; z = 0.02;
        std::cout<<"[debug mode] x = "<<x<<std::endl;
        std::cout<<"[debug mode] y = "<<y<<std::endl;
        std::cout<<"[debug mode] z = "<<z<<std::endl;
    }
    else{
        // option B: YOLO input
        std::string bbs_loadPath = "/home/ruinianxu/vision_position/3Dlocations_mug.txt";
        std::ifstream myfile(bbs_loadPath.c_str());
        std::string x_s, y_s, z_s;

        // find 3D bbs 
        if (myfile.is_open()){
          if(!getline (myfile, x_s))
             x_s = "0.5";
          if(!getline (myfile, y_s))
             y_s = "0";
          if(!getline (myfile,z_s))
             z_s = "0";

          myfile.close();
        }
        else 
          std::cout<<"file not open"<<std::endl;

        x = atof (x_s.c_str());
        y = atof (y_s.c_str());
        z = atof (z_s.c_str());
        

        std::cout<<"reading.. x approach = "<<x<<std::endl;
        std::cout<<"reading.. y approach = "<<y<<std::endl;
        std::cout<<"reading.. z approach = "<<z<<std::endl;
    }
}

std::string read_command(std::string &command){
    std::string output="";
    std::ifstream myfileIn ("/home/ruinianxu/meta_command/commands.txt");

    getline (myfileIn,command);
    if(command != ""){ // got sth
        std::cout << "receive a command = " << command << std::endl;
        std::ofstream myfileOut ("/home/ruinianxu/meta_command/commands.txt", std::ios::trunc);
        myfileOut << "";
        myfileOut.close();
        return command;
    }
    myfileIn.close();
    return output;
}