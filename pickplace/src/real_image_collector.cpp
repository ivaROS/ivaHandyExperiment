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

std::random_device                  rand_dev;
std::mt19937                        generator(rand_dev());
std::uniform_real_distribution<>    distr(-1, 1);
// std::uniform_real_distribution<>    distr_far(0.01, 0.05);
// std::uniform_real_distribution<>    distr_close(0.00, 0.01);
std::uniform_real_distribution<>    distr_far(0.005, 0.05);
std::uniform_real_distribution<>    distr_close(0.00, 0.005);

std::vector<double> current_joint_values;
bool mutex = 0;

const double pi = std::acos(-1);
const double gripper_open_value = -0.7; //open value of 8th motor
const double gripper_close_value = 0.7; //close value of 8th motor

void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg);
void traj_end_mutex(const std_msgs::Bool::ConstPtr& msg);
void mutex_traj();
const std::vector<std::string> listNamedPoses(moveit::planning_interface::MoveGroup &group);
void addObject2Scene(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher);
void gotoNamedTarget(moveit::planning_interface::MoveGroup &group, std::string target, bool constraint_on);
void getConstraint(moveit::planning_interface::MoveGroup &group, std::string target);
geometry_msgs::Pose random_pose_ungraspable(geometry_msgs::Pose pose_grasp, double angle_grasp);
geometry_msgs::Pose random_pose_graspable(geometry_msgs::Pose pose_grasp, double angle_grasp);
geometry_msgs::Pose approach(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose pose_grasp, double angle_grasp, std::string mode);
void down(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose pose);
void lift(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose pose);
void save_groundtruth(geometry_msgs::Pose pose_grasp1, double angle1, geometry_msgs::Pose pose_grasp2, double angle2, 
  geometry_msgs::Pose pose_grasp3, double angle3, std::string label, std::string obj_name, std::string idx_campose, std::string idx_objpose, std::string idx_inipose);

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

  //Before go to home position, take a pic of reference image
  //write 1 to text file to notice it is time to save image
  // std::ofstream myfile;
  // myfile.open ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt",  std::ios::trunc);
  // myfile << "1\n";
  // myfile.close();
  // //wait for saving image ends
  // int flag = 1;
  // while(flag){
  //   std::string line;
  //   std::ifstream myfile ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt");
  //   getline (myfile,line);
  //   flag = atoi(line.c_str());
  //   myfile.close();
  // }

  /*****************************************************************
  *                      Specify initial pose                      *
  *****************************************************************/
  // namedTargets stores the names of pre-defined poses(joint values)
  // select the name (ex. "home"), gotoNamedTarget function will find plan to the specific pose
  //argument that must be changed at the begining
  int pause;
  std::string name = "scoop";
  std::string idx_campose = "2";
  std::string idx_objpose = "4";
  double z_compen = 0;
  if (name == "scoop")
    z_compen = 0.010;
  else if (name == "ball")
    z_compen = 0.039;
  else if (name == "screwdriver")
    z_compen = 0.004;
    //z_compen = 0.007;
  else if (name == "sodabottle")
    z_compen = 0.07;
  else if (name == "mouse")
    z_compen = 0.012;


  std::string target = ""; 
  int targetNum = 0;
  std::cout<<"select target pose above: "; std::cin >> targetNum;
  if(targetNum == 0) target = "Home";
  else if (targetNum == 1) target = "Hold_Water";
  else target = "Home";

  gotoNamedTarget(group_arm, target, 0);

  // std::cin>>pause;

  //set 8th and 9th motor to zero to avoid collision
  std::cout<<"open gripper"<<std::endl;
  std_msgs::Float64 val_8;
  std_msgs::Float64 val_9;
  val_8.data = gripper_open_value;
  val_9.data = -gripper_open_value;
  pub_8.publish(val_8);
  pub_9.publish(val_9);
  std::cout<<"opened gripper"<<std::endl;

  geometry_msgs::Pose pose_grasp1;
  geometry_msgs::Pose pose_grasp2;
  geometry_msgs::Pose pose_grasp3;

  std::string line_x1, line_x2, line_x3, line_y1, line_y2, line_y3, line_z1, line_z2, line_z3, line_angle1, line_angle2, line_angle3;
  std::ifstream myfileIn ("/home/ruinianxu/software/aruco_tag/graspPositions.txt");
  getline (myfileIn,line_x1);
  getline (myfileIn,line_y1);
  getline (myfileIn,line_z1);
  getline (myfileIn,line_angle1);
  getline (myfileIn,line_x2);
  getline (myfileIn,line_y2);
  getline (myfileIn,line_z2);
  getline (myfileIn,line_angle2);
  getline (myfileIn,line_x3);
  getline (myfileIn,line_y3);
  getline (myfileIn,line_z3);
  getline (myfileIn,line_angle3);

  std::cout<<line_x1<<std::endl;
  std::cout<<line_y1<<std::endl;
  std::cout<<line_z1<<std::endl;
  std::cout<<line_angle1<<std::endl;

  //convert string to double
  double x1,x2,x3,y1,y2,y3,z1,z2,z3,angle1,angle2,angle3;
  x1 = atof(line_x1.c_str());
  x2 = atof(line_x2.c_str());
  x3 = atof(line_x3.c_str());
  y1 = atof(line_y1.c_str());
  y2 = atof(line_y2.c_str());
  y3 = atof(line_y3.c_str());
  z1 = atof(line_z1.c_str());
  z2 = atof(line_z2.c_str());
  z3 = atof(line_z3.c_str());
  angle1 = atof(line_angle1.c_str());
  angle2 = atof(line_angle2.c_str());
  angle3 = atof(line_angle3.c_str());

  std::cout<<x1<<std::endl;
  std::cout<<y1<<std::endl;
  std::cout<<z1<<std::endl;
  std::cout<<angle1<<std::endl;

  //setup for three grasps
  pose_grasp1.position.x = x1;
  pose_grasp1.position.y = y1;
  pose_grasp1.position.z = z1 + z_compen;
  tf::Quaternion qat1 = tf::createQuaternionFromRPY(0, -M_PI, angle1);
  qat1.normalize();
  pose_grasp1.orientation.x = qat1.x();//0.577;//0.49; // two-sided gribber
  pose_grasp1.orientation.y = qat1.y();//0.577;//0.49; // two-sided gribber
  pose_grasp1.orientation.z = qat1.z();//0.577;//0.49;
  pose_grasp1.orientation.w = qat1.w(); 

  pose_grasp2.position.x = x2;
  pose_grasp2.position.y = y2;
  pose_grasp2.position.z = z2 + z_compen;
  tf::Quaternion qat2 = tf::createQuaternionFromRPY(0, -M_PI, angle2);
  qat2.normalize();
  pose_grasp2.orientation.x = qat2.x();//0.577;//0.49; // two-sided gribber
  pose_grasp2.orientation.y = qat2.y();//0.577;//0.49; // two-sided gribber
  pose_grasp2.orientation.z = qat2.z();//0.577;//0.49;
  pose_grasp2.orientation.w = qat2.w(); 

  pose_grasp3.position.x = x3;
  pose_grasp3.position.y = y3;
  pose_grasp3.position.z = z3 + z_compen;
  tf::Quaternion qat3 = tf::createQuaternionFromRPY(0, -M_PI, angle3);
  qat3.normalize();
  pose_grasp3.orientation.x = qat3.x();//0.577;//0.49; // two-sided gribber
  pose_grasp3.orientation.y = qat3.y();//0.577;//0.49; // two-sided gribber
  pose_grasp3.orientation.z = qat3.z();//0.577;//0.49;
  pose_grasp3.orientation.w = qat3.w(); 

  for (int i = 0; i < 45; i++){
    geometry_msgs::Pose pose_ee;

    if (i < 5){
      //make sure ar tag on the gripper has been seen
      int flag_ar = 0;
      while (!flag_ar){
        //plan and execution
        std::string mode = "graspable";
        pose_ee = approach(group_arm, pose_grasp1, angle1, mode);

        //read the file 
        std::string line_ar;
        std::ifstream file_ar ("/home/ruinianxu/software/aruco_tag/arucoPositions.txt");
        
        getline (file_ar,line_ar);
        flag_ar = atoi(line_ar.c_str());
        for (int j = 0; j < 2; j++){
          if (flag_ar)
            break;
          else{
            for (int k = 0; k < 9; k++)
              getline (file_ar,line_ar);
            flag_ar = atoi(line_ar.c_str());
          }
        }
        
        file_ar.close();
      }
      // save groundtruth
      std::string label = "1";
      std::string idx_inipose = std::to_string(i);
      save_groundtruth(pose_grasp1, angle1, pose_grasp2, angle2, pose_grasp3, angle3, label, name, idx_campose, idx_objpose, idx_inipose);

      // down(group_arm, pose_ee);
      //write 1 to text file to notice it is time to save image
      std::ofstream myfile;
      myfile.open ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt", std::ios::trunc);
      myfile << "1\n";
      myfile.close();
      //wait for saving image ends
      int flag = 1;
      while(flag){
        std::string line;
        std::ifstream myfile ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt");
        getline (myfile,line);
        flag = atoi(line.c_str());
        myfile.close();
      }
      

      //lift the arm
      // lift(group_arm, pose_ee);
      //go back to home
      gotoNamedTarget(group_arm, target, 0);
    }
    else if (i < 10){
      //make sure ar tag on the gripper has been seen
      int flag_ar = 0;
      while (!flag_ar){
        //plan and execution
        std::string mode = "graspable";
        pose_ee = approach(group_arm, pose_grasp2, angle2, mode);

        //read the file 
        std::string line_ar;
        std::ifstream file_ar ("/home/ruinianxu/software/aruco_tag/arucoPositions.txt");
        
        getline (file_ar,line_ar);
        flag_ar = atoi(line_ar.c_str());
        for (int j = 0; j < 2; j++){
          if (flag_ar)
            break;
          else{
            for (int k = 0; k < 9; k++)
              getline (file_ar,line_ar);
            flag_ar = atoi(line_ar.c_str());
          }
        }
        
        file_ar.close();
      }
      // save groundtruth
      std::string label = "1";
      std::string idx_inipose = std::to_string(i);
      save_groundtruth(pose_grasp1, angle1, pose_grasp2, angle2, pose_grasp3, angle3, label, name, idx_campose, idx_objpose, idx_inipose);

      // down(group_arm, pose_ee);
      //write 1 to text file to notice it is time to save image
      std::ofstream myfile;
      myfile.open ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt", std::ios::trunc);
      myfile << "1\n";
      myfile.close();
      //wait for saving image ends
      int flag = 1;
      while(flag){
        std::string line;
        std::ifstream myfile ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt");
        getline (myfile,line);
        flag = atoi(line.c_str());
        myfile.close();
      }

      //lift the arm
      // lift(group_arm, pose_ee);
      //go back to home
      gotoNamedTarget(group_arm, target, 0);
    }
    else if (i < 15){
      //make sure ar tag on the gripper has been seen
      int flag_ar = 0;
      while (!flag_ar){
        //plan and execution
        std::string mode = "graspable";
        pose_ee = approach(group_arm, pose_grasp3, angle3, mode);

        //read the file 
        std::string line_ar;
        std::ifstream file_ar ("/home/ruinianxu/software/aruco_tag/arucoPositions.txt");
        
        getline (file_ar,line_ar);
        flag_ar = atoi(line_ar.c_str());
        for (int j = 0; j < 2; j++){
          if (flag_ar)
            break;
          else{
            for (int k = 0; k < 9; k++)
              getline (file_ar,line_ar);
            flag_ar = atoi(line_ar.c_str());
          }
        }
        
        file_ar.close();
      }
      // save groundtruth
      std::string label = "1";
      std::string idx_inipose = std::to_string(i);
      save_groundtruth(pose_grasp1, angle1, pose_grasp2, angle2, pose_grasp3, angle3, label, name, idx_campose, idx_objpose, idx_inipose);

      // down(group_arm, pose_ee);
      //write 1 to text file to notice it is time to save image
      std::ofstream myfile;
      myfile.open ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt", std::ios::trunc);
      myfile << "1\n";
      myfile.close();
      //wait for saving image ends
      int flag = 1;
      while(flag){
        std::string line;
        std::ifstream myfile ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt");
        getline (myfile,line);
        flag = atoi(line.c_str());
        myfile.close();
      }

      //lift the arm
      // lift(group_arm, pose_ee);
      //go back to home
      gotoNamedTarget(group_arm, target, 0);
    }

    else if (i < 25){
      //make sure ar tag on the gripper has been seen
      std::cout<<"Loop:"<<i<<std::endl;
      int flag_ar = 0;
      while (!flag_ar){
        //plan and execution
        // std::cin>>pause;
        std::string mode = "ungraspable";
        pose_ee = approach(group_arm, pose_grasp1, angle1, mode);
        // std::cin>>pause;

        //read the file 
        std::string line_ar;
        std::ifstream file_ar ("/home/ruinianxu/software/aruco_tag/arucoPositions.txt");

        getline (file_ar,line_ar);
        flag_ar = atoi(line_ar.c_str());
        for (int j = 0; j < 2; j++){
          if (flag_ar)
            break;
          else{
            for (int k = 0; k < 9; k++)
              getline (file_ar,line_ar);
            flag_ar = atoi(line_ar.c_str());
          }
        }
        
        file_ar.close();
      }
      // save groundtruth
      std::string label = "0";
      std::string idx_inipose = std::to_string(i);
      save_groundtruth(pose_grasp1, angle1, pose_grasp2, angle2, pose_grasp3, angle3, label, name, idx_campose, idx_objpose, idx_inipose);

      // down(group_arm, pose_ee);
      // std::cin>>pause;
      //write 1 to text file to notice it is time to save image
      std::ofstream myfile;
      myfile.open ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt", std::ios::trunc);
      myfile << "1\n";
      myfile.close();
      //wait for saving image ends
      int flag = 1;
      while(flag){
        std::string line;
        std::ifstream myfile ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt");
        getline (myfile,line);
        flag = atoi(line.c_str());
        myfile.close();
      }


      //lift the arm
      // std::cin>>pause;
      // lift(group_arm, pose_ee);
      //go back to home
      gotoNamedTarget(group_arm, target, 0);
    }
    else if (i < 35){
      //make sure ar tag on the gripper has been seen
      int flag_ar = 0;
      while (!flag_ar){
        //plan and execution
        std::string mode = "ungraspable";
        pose_ee = approach(group_arm, pose_grasp2, angle2, mode);

        //read the file 
        std::string line_ar;
        std::ifstream file_ar ("/home/ruinianxu/software/aruco_tag/arucoPositions.txt");

        getline (file_ar,line_ar);
        flag_ar = atoi(line_ar.c_str());
        for (int j = 0; j < 2; j++){
          if (flag_ar)
            break;
          else{
            for (int k = 0; k < 9; k++)
              getline (file_ar,line_ar);
            flag_ar = atoi(line_ar.c_str());
          }
        }
        
        file_ar.close();
      }
      // save groundtruth
      std::string label = "0";
      std::string idx_inipose = std::to_string(i);
      save_groundtruth(pose_grasp1, angle1, pose_grasp2, angle2, pose_grasp3, angle3, label, name, idx_campose, idx_objpose, idx_inipose);

      // down(group_arm, pose_ee);
      //write 1 to text file to notice it is time to save image
      std::ofstream myfile;
      myfile.open ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt", std::ios::trunc);
      myfile << "1\n";
      myfile.close();
      //wait for saving image ends
      int flag = 1;
      while(flag){
        std::string line;
        std::ifstream myfile ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt");
        getline (myfile,line);
        flag = atoi(line.c_str());
        myfile.close();
      }


      //lift the arm
      // lift(group_arm, pose_ee);
      //go back to home
      gotoNamedTarget(group_arm, target, 0);
    }
    else if (i < 45){
      //make sure ar tag on the gripper has been seen
      int flag_ar = 0;
      while (!flag_ar){
        //plan and execution
        std::string mode = "ungraspable";
        pose_ee = approach(group_arm, pose_grasp3, angle3, mode);

        //read the file 
        std::string line_ar;
        std::ifstream file_ar ("/home/ruinianxu/software/aruco_tag/arucoPositions.txt");
        
        getline (file_ar,line_ar);
        flag_ar = atoi(line_ar.c_str());
        for (int j = 0; j < 2; j++){
          if (flag_ar)
            break;
          else{
            for (int k = 0; k < 9; k++)
              getline (file_ar,line_ar);
            flag_ar = atoi(line_ar.c_str());
          }
        }
        
        file_ar.close();
      }
      // save groundtruth
      std::string label = "0";
      std::string idx_inipose = std::to_string(i);
      save_groundtruth(pose_grasp1, angle1, pose_grasp2, angle2, pose_grasp3, angle3, label, name, idx_campose, idx_objpose, idx_inipose);

      // down(group_arm, pose_ee);
      //write 1 to text file to notice it is time to save image
      std::ofstream myfile;
      myfile.open ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt", std::ios::trunc);
      myfile << "1\n";
      myfile.close();
      //wait for saving image ends
      int flag = 1;
      while(flag){
        std::string line;
        std::ifstream myfile ("/home/ruinianxu/software/toy-opencv-mat-socket-server-master/command/command.txt");
        getline (myfile,line);
        flag = atoi(line.c_str());
        myfile.close();
      }


      //lift the arm
      // lift(group_arm, pose_ee);
      //go back to home
      gotoNamedTarget(group_arm, target, 0);
    }

  }

  return 1;
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

geometry_msgs::Pose random_pose_ungraspable(geometry_msgs::Pose pose_grasp, double angle_grasp){
  geometry_msgs::Pose pose;
  double x, y, z, angle;
  if (distr(generator) > 0){
    x = distr_far(generator);
  }
  else
    x = -distr_far(generator);
  if (distr(generator) > 0){
    y = distr_far(generator);
  }
  else
    y = -distr_far(generator);
  
  z = distr_far(generator);
  
  angle = distr(generator) * pi;

  std::cout<<"ungraspable pose, angle:"<<angle / pi * 180<<std::endl;
  std::cout<<"ungraspable pose, total angle:"<<angle_grasp + (angle) / pi * 180<<std::endl;

  std::cout<<"incre x:"<<x<<std::endl;
  std::cout<<"incre y:"<<y<<std::endl;
  std::cout<<"incre z:"<<z<<std::endl;
  std::cout<<"incre angle:"<<angle<<std::endl;

  pose.position.x = pose_grasp.position.x + x;//0.4;
  pose.position.y = pose_grasp.position.y + y;//0.0;
  pose.position.z = pose_grasp.position.z + z;//0.2;
  tf::Quaternion qat = tf::createQuaternionFromRPY(0, -M_PI, angle_grasp / 180 * pi + angle);
  qat.normalize();
  pose.orientation.x = qat.x();//0.577;//0.49; // two-sided gribber
  pose.orientation.y = qat.y();//0.577;//0.49; // two-sided gribber
  pose.orientation.z = qat.z();//0.577;//0.49;
  pose.orientation.w = qat.w(); 

  std::cout<<"Position of initial x:"<<pose.position.x<<std::endl;
  std::cout<<"Position of initial y:"<<pose.position.y<<std::endl;
  std::cout<<"Position of initial z:"<<pose.position.z<<std::endl;

  return pose;
}

geometry_msgs::Pose random_pose_graspable(geometry_msgs::Pose pose_grasp, double angle_grasp){
  geometry_msgs::Pose pose;
  double x, y, z, angle;
  if (distr(generator) > 0){
    x = distr_close(generator);
  }
  else
    x = -distr_close(generator);
  if (distr(generator) > 0){
    y = distr_close(generator);
  }
  else
    y = -distr_close(generator);

    z = distr_close(generator);

  angle = distr(generator) * pi / 12;

  std::cout<<"graspable pose, angle:"<<angle / pi * 180<<std::endl;

  std::cout<<"graspable pose, total angle:"<<angle_grasp + (angle) / pi * 180<<std::endl;

  pose.position.x = pose_grasp.position.x + x;//0.4;
  pose.position.y = pose_grasp.position.y + y;//0.0;
  pose.position.z = pose_grasp.position.z + z;//0.2;
  tf::Quaternion qat = tf::createQuaternionFromRPY(0, -M_PI, angle_grasp / 180 * pi + angle);
  qat.normalize();
  pose.orientation.x = qat.x();//0.577;//0.49; // two-sided gribber
  pose.orientation.y = qat.y();//0.577;//0.49; // two-sided gribber
  pose.orientation.z = qat.z();//0.577;//0.49;
  pose.orientation.w = qat.w(); 

  return pose;
}

geometry_msgs::Pose approach(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose pose_grasp, double angle_grasp, std::string mode){
  geometry_msgs::Pose pose_ee;
  moveit::planning_interface::MoveGroup::Plan plan_approach;
  bool success_approach;
  if (mode == "ungraspable"){
    pose_ee = random_pose_ungraspable(pose_grasp, angle_grasp);
    // Firstly, go to the position above the target position
    // pose_ee.position.z = pose_ee.position.z + 0.03;

    std::cout<<"Approach position x:"<<pose_ee.position.x<<std::endl;
    std::cout<<"Approach position y:"<<pose_ee.position.y<<std::endl;
    std::cout<<"Approach position z:"<<pose_ee.position.z<<std::endl;

    // set target pose
    group.setPoseTarget(pose_ee);

    // plan
    success_approach = group.plan(plan_approach);
    while(!success_approach){
      pose_ee = random_pose_ungraspable(pose_grasp, angle_grasp);
      // Firstly, go to the position above the target position
      // pose_ee.position.z = pose_ee.position.z + 0.03;

      // set target pose
      group.setPoseTarget(pose_ee);

      success_approach = group.plan(plan_approach);
    }   
  }
  else if (mode == "graspable"){
    pose_ee = random_pose_graspable(pose_grasp, angle_grasp);
    // Firstly, go to the position above the target position
    // pose_ee.position.z = pose_ee.position.z + 0.03;

    std::cout<<"Approach position x:"<<pose_ee.position.x<<std::endl;
    std::cout<<"Approach position y:"<<pose_ee.position.y<<std::endl;
    std::cout<<"Approach position z:"<<pose_ee.position.z<<std::endl;

    // set target pose
    group.setPoseTarget(pose_ee);

    // plan
    success_approach = group.plan(plan_approach);
    while(!success_approach){
      pose_ee = random_pose_graspable(pose_grasp, angle_grasp);
      // Firstly, go to the position above the target position
      // pose_ee.position.z = pose_ee.position.z + 0.03;

      // set target pose
      group.setPoseTarget(pose_ee);

      success_approach = group.plan(plan_approach);
    }  
  }
  // execute need to replan and re execute if execution fails
  bool success_exe = group.execute(plan_approach);

  while(!success_exe){
    success_approach = false;
    while(!success_approach){
      success_approach = group.plan(plan_approach);
    }  
    success_exe = group.execute(plan_approach);
  }

  mutex_traj();

  // pose_ee.position.z = pose_ee.position.z - 0.03;

  return pose_ee;
}

void down(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose pose){
  //Then go down
  std::cout<<"Down position x:"<<pose.position.x<<std::endl;
  std::cout<<"Down position y:"<<pose.position.y<<std::endl;
  std::cout<<"Down position z:"<<pose.position.z<<std::endl;

  // set target pose
  group.setPoseTarget(pose);
  // plan
  moveit::planning_interface::MoveGroup::Plan plan;
  bool success = false;
  while(!success){
    success = group.plan(plan);
  }
  // execute
  bool success_exe = group.execute(plan);
  while(!success_exe){
    success = false;
    while(!success){
      success = group.plan(plan);
    }  
    success_exe = group.execute(plan);
  }

  mutex_traj();
}

void lift(moveit::planning_interface::MoveGroup &group, geometry_msgs::Pose pose){
  pose.position.z = pose.position.z + 0.05;

  // std::cout<<"Lift position x:"<<pose.position.x<<std::endl;
  // std::cout<<"Lift position y:"<<pose.position.y<<std::endl;
  // std::cout<<"Lift position z:"<<pose.position.z<<std::endl;

  // set target pose
  group.setPoseTarget(pose);
  // plan
  moveit::planning_interface::MoveGroup::Plan plan;
  bool success = false;
  while(!success)
    success = group.plan(plan);
  // execute
  bool success_exe = group.execute(plan);
  while(!success_exe){
    success = false;
    while(!success){
      success = group.plan(plan);
    }  
    success_exe = group.execute(plan);
  }
  mutex_traj();

  pose.position.z = pose.position.z - 0.05;
}

void save_groundtruth(geometry_msgs::Pose pose_grasp1, double angle1, geometry_msgs::Pose pose_grasp2, double angle2, 
  geometry_msgs::Pose pose_grasp3, double angle3, std::string label, std::string obj_name, std::string idx_campose, std::string idx_objpose, std::string idx_inipose)
{
  std::string path_grasp = "/home/ruinianxu/software/toy-opencv-mat-socket-server-master/data/groundtruth/" + obj_name + "/" + idx_campose + "/" + idx_objpose + "/" + idx_inipose + "/pose_grasp.txt";
  std::string path_artag = "/home/ruinianxu/software/toy-opencv-mat-socket-server-master/data/groundtruth/" + obj_name + "/" + idx_campose + "/" + idx_objpose + "/" + idx_inipose + "/pose_ar_tag.txt";
  std::string path_label = "/home/ruinianxu/software/toy-opencv-mat-socket-server-master/data/groundtruth/" + obj_name + "/" + idx_campose + "/" + idx_objpose + "/" + idx_inipose + "/label.txt";
  std::ofstream myfile_grasp(path_grasp, std::ios::trunc);
  if (myfile_grasp.is_open()){
    myfile_grasp << std::to_string(pose_grasp1.position.x) + " " + std::to_string(pose_grasp1.position.y) + " " + std::to_string(pose_grasp1.position.z) 
      + " " + std::to_string(pose_grasp1.orientation.x) + " " + std::to_string(pose_grasp1.orientation.y) + " " + std::to_string(pose_grasp1.orientation.z) 
      + " " + std::to_string(pose_grasp1.orientation.w) + " " + std::to_string(angle1) + "\n";
    myfile_grasp << std::to_string(pose_grasp2.position.x) + " " + std::to_string(pose_grasp2.position.y) + " " + std::to_string(pose_grasp2.position.z) 
        + " " + std::to_string(pose_grasp2.orientation.x) + " " + std::to_string(pose_grasp2.orientation.y) + " " + std::to_string(pose_grasp2.orientation.z) 
        + " " + std::to_string(pose_grasp2.orientation.w) + " " + std::to_string(angle2) + "\n";
    myfile_grasp << std::to_string(pose_grasp3.position.x) + " " + std::to_string(pose_grasp3.position.y) + " " + std::to_string(pose_grasp3.position.z) 
        + " " + std::to_string(pose_grasp3.orientation.x) + " " + std::to_string(pose_grasp3.orientation.y) + " " + std::to_string(pose_grasp3.orientation.z) 
        + " " + std::to_string(pose_grasp3.orientation.w) + " " + std::to_string(angle3) + "\n";
  }
  myfile_grasp.close();

  std::ifstream myfileIn ("/home/ruinianxu/software/aruco_tag/arucoPositions.txt");
  std::ofstream myfile_artag(path_artag, std::ios::trunc);
  std::string line;
  if (myfileIn.is_open()){
    while(getline(myfileIn, line)){
      myfile_artag << line + "\n";
    }
    myfileIn.close();
    myfile_artag.close();
  }

  std::ofstream myfile_label(path_label, std::ios::trunc);
  myfile_label << label + "\n";
  myfile_label.close();
}