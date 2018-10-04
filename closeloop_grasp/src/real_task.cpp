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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions> 
#include <tf/tf.h>
#include <random>

// include headers for moveit group
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

std::vector<double> current_joint_values;
bool mutex = 0;

const double pi = std::acos(-1);
const double gripper_open_value = -0.7; //open value of 8th motor
const double gripper_close_value = 0.7; //close value of 8th motor
const double graspable_threshold = 0.8;

void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg);
void traj_end_mutex(const std_msgs::Bool::ConstPtr& msg);
void mutex_traj();
const std::vector<std::string> listNamedPoses(moveit::planning_interface::MoveGroup &group);
void addObject2Scene(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher);
void gotoNamedTarget(moveit::planning_interface::MoveGroup &group, std::string target, bool constraint_on);
void getConstraint(moveit::planning_interface::MoveGroup &group, std::string target);
void closeGrabber(ros::Publisher &pub_8);
void approach(moveit::planning_interface::MoveGroup &group);
void closegrasp(moveit::planning_interface::MoveGroup &group, ros::Publisher &pub_8);

int main(int argc, char** argv) {
  // ros initialization
  ros::init(argc, argv, "handy_real_image_collector");
  ros::NodeHandle node_handle;  
  ros::Publisher pub_7 = node_handle.advertise<std_msgs::Float64>("/finalarm_position_controller_7/command", 1, true);
  ros::Publisher pub_8 = node_handle.advertise<std_msgs::Float64>("/finalarm_position_controller_8/command", 1, true);
  ros::Publisher pub_9 = node_handle.advertise<std_msgs::Float64>("/finalarm_position_controller_9/command", 1, true);
  ros::Subscriber sub_multi_joint_states = node_handle.subscribe<sensor_msgs::JointState>("/joint_states", 1, joint_state_handler);
  ros::Subscriber sub_endtime = node_handle.subscribe<std_msgs::Bool>("/finalarm_joint_trajectory_action_controller/mutex", 5, traj_end_mutex);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // kinematic_state & kinematic_model loading & planner
  moveit::planning_interface::MoveGroup group_arm("arm");
  group_arm.setPlannerId("BKPIECEkConfigDefault");

  // We will use the planning_scene_interface class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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
  *                           Pipeline                             *
  *     1.Go to the home position                                  *
  *     2.Approach the object position given by grasp detector     *
  *     3.Close the loop by keeping running closegrasp             *
  *****************************************************************/
  int quit = 0
  while (!quit){
    /*********** Step1 **********/
    std_msgs::Float64 val_8;
    std_msgs::Float64 val_9;
    val_8.data = gripper_open_value;
    val_9.data = -gripper_open_value;
    pub_8.publish(val_8);
    pub_9.publish(val_9);
    gotoNamedTarget(group_arm, "Home", 0);

    /*********** Step2 **********/
    approach(group);
    
    /*********** Step3 **********/
    cloesgrasp(group, pub_8);
    
    std::cout<<"Would you want to quit performing closegrasp? 0. No 1. Yes"<<std::endl;
    std::cin>>quit;
  }
  //Open the gripper and go back to home position after quit
  std_msgs::Float64 val_8;
  std_msgs::Float64 val_9;
  val_8.data = gripper_open_value;
  val_9.data = -gripper_open_value;
  pub_8.publish(val_8);
  pub_9.publish(val_9);
  gotoNamedTarget(group_arm, "Home", 0);
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

void gotoNamedTarget(moveit::planning_interface::MoveGroup &group, std::string target, bool constraint_on){
    //set 8th and 9th motor value to zero to avoid collision
    // std_msgs::Float64 val_8;
    // std_msgs::Float64 val_9;
    // val_8.data = 0;
    // val_9.data = 0;
    // pub_8.publish(val_8);
    // pub_9.publish(val_9);

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
    bool success_exe = group.execute(my_plan);

    //redo if execution failed

    while(!success_exe){
      group.plan(my_plan);
      success_exe = group.execute(my_plan);
    }

    mutex_traj();
    
    // show current Joints (the real robot joints, not the planned joints)
    std::vector<double> currentJoints = group.getCurrentJointValues();
    /*
    std::cout<< "current joint values:"<<std::endl;
    for(size_t i = 0; i<currentJoints.size(); i++) std::cout<< currentJoints[i]<<" ";
    std::cout<<std::endl;
    */
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
    //pose.position.z = -0.12;//-0.123;//-0.13;//-0.1;//-0.07
    pose.position.z = -0.13; //[FC]

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

void closeGrabber(ros::Publisher &pub_8){
    std_msgs::Float64 val_8 ;
    val_8.data = gripper_close_value;
    pub_8.publish(val_8);

    // std_msgs::Float64 val_9 ;
    // val_9.data = gripper_close_value;
    // pub_9.publish(val_9);
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
    while(1){
        if(std::abs(load_8) > 0.4){
            val_8.data = current_joint_values[7] + paw_torque_compensate;
            // val_9.data = -val_8.data;
            pub_8.publish(val_8);
            // pub_9.publish(val_9);
            break;
        }
    }
}

void approach(moveit::planning_interface::MoveGroup &group){
  //read the position given by grasp detector, x,y,z,rotate_angle(radius)
  std::string line_1, line_2, line_3, line_4;
  std::ifstream file_approach ("path to file recording approach pose");
  getline(file_approach, line_1);
  getline(file_approach, line_2);
  getline(file_approach, line_3);
  getline(file_approach, line_4);
  double x, y, z, angle;
  x = atof(line_1.c_str());
  y = atof(line_2.c_str());
  z = atof(line_3.c_str());
  angle = atof(line_4.c_str());
  //Setup approach pose conifg
  geometry_msgs::Pose pose_approach;
  pose_approach.position.x = x;
  pose_approach.position.y = y;
  pose_approach.position.z = z;
  tf::Quaternion qat1 = tf::createQuaternionFromRPY(0, -M_PI, angle);
  qat1.normalize();
  pose_approach.orientation.x = qat1.x();
  pose_approach.orientation.y = qat1.y();
  pose_approach.orientation.z = qat1.z();
  pose_approach.orientation.w = qat1.w(); 
  //plan and execute traj
  group.setPoseTarget(pose_approach);
  moveit::planning_interface::MoveGroup::Plan plan_approach;
  bool success_plan_approach = false;
  while (!success_plan_approach){
    success_plan_approach = group.plan(plan_approach);
  }
  bool success_exe_approach = group.execute(plan_approach);
  while(!success_exe_approach){
    success_plan_approach = false;
    while (!success_plan_approach){
      success_plan_approach = group.plan(plan_approach);
    }
    success_exe_approach = group.execute(plan_approach);
  }
  mutex_traj();
}

void closegrasp(moveit::planning_interface::MoveGroup &group, ros::Publisher &pub_8){
  //read ouput from closegrasp, x,y,z,angle,confidence
  std::string line_1, line_2, line_3, line_4, line_5;
  std::ifstream file_closegrasp ("path to file recording closegrasp output");
  getline(file_approach, line_1);
  getline(file_approach, line_2);
  getline(file_approach, line_3);
  getline(file_approach, line_4);
  getline(file_approach, line_5);
  double x, y, z, angle, confidence;
  x = atof(line_1.c_str());
  y = atof(line_2.c_str());
  z = atof(line_3.c_str());
  angle = atof(line_4.c_str());
  confidence = atof(line_5.c_str());

  geometry_msgs::Pose pose_closegrasp;
  while(confidence < graspable_threshold){
    //control strategy: factor z direction by  2 
    //Setup closegrasp pose conifg
    pose_closegrasp.position.x = x;
    pose_closegrasp.position.y = y;
    //factor z by 2 when x and y distance is larger than 0.02
    if (sqrt(x*x + y*y) > 0.02){
      pose_closegrasp.position.z = z / 2.0;
    }
    else
      pose_closegrasp.position.z = z;
    
    tf::Quaternion qat1 = tf::createQuaternionFromRPY(0, -M_PI, angle);
    qat1.normalize();
    pose_closegrasp.orientation.x = qat1.x();
    pose_closegrasp.orientation.y = qat1.y();
    pose_closegrasp.orientation.z = qat1.z();
    pose_closegrasp.orientation.w = qat1.w(); 
    //plan and execute traj
    group.setPoseTarget(pose_closegrasp);
    moveit::planning_interface::MoveGroup::Plan plan_closegrasp;
    bool success_plan_closegrasp = false;
    while (!success_plan_closegrasp){
      success_plan_closegrasp = group.plan(plan_closegrasp);
    }
    bool success_exe_closegrasp = group.execute(plan_closegrasp);
    while(!success_exe_closegrasp){
      success_plan_closegrasp = false;
      while (!success_plan_closegrasp){
        success_plan_closegrasp = group.plan(plan_closegrasp);
      }
      success_exe_closegrasp = group.execute(plan_closegrasp);
    }
    mutex_traj();

    getline(file_approach, line_1);
    getline(file_approach, line_2);
    getline(file_approach, line_3);
    getline(file_approach, line_4);
    getline(file_approach, line_5);
    double x, y, z, angle, confidence;
    x = atof(line_1.c_str());
    y = atof(line_2.c_str());
    z = atof(line_3.c_str());
    angle = atof(line_4.c_str());
    confidence = atof(line_5.c_str());
  }
  //close gripper
  closeGrabber(pub_8);
  //lift the gripper
  //Setup lift pose
  geometry_msgs::Pose pose_lift;
  pose_lift.position.x = pose_closegrasp.position.x;
  pose_lift.position.y = pose_closegrasp.position.y;
  pose_lift.position.z = pose_closegrasp.position.z + 0.2;
  pose_lift.orientation.x = pose_closegrasp.orientation.x;
  pose_lift.orientation.y = pose_closegrasp.orientation.y;
  pose_lift.orientation.z = pose_closegrasp.orientation.z;
  pose_lift.orientation.w = pose_closegrasp.orientation.w; 
  //plan and execute traj
  group.setPoseTarget(pose_lift);
  moveit::planning_interface::MoveGroup::Plan plan_lift;
  bool success_plan_lift = false;
  while (!success_plan_lift){
    success_plan_lift = group.plan(plan_lift);
  }
  bool success_exe_lift = group.execute(plan_lift);
  while(!success_exe_lift){
    success_plan_lift = false;
    while (!success_plan_lift){
      success_plan_lift = group.plan(plan_closeplan_liftgrasp);
    }
    success_exe_lift = group.execute(plan_lift);
  }
  mutex_traj();
}