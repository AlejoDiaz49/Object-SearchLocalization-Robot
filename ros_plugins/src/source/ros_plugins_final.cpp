#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include "gazebo_msgs/ContactsState.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ApplyBodyWrench.h"
#include "gazebo_msgs/SetModelState.h"
#include <iostream>
#include <algorithm>
#include <string>
#include <fstream>
#include "tf/tf.h"



/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */


int coin_count;
int bomb_count;
int ArucoId;
int InfoId;
double global_coin_count;
double global_bomb_count;
ros::Publisher score_pub;
std::string ModelDirectory;


void statesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  geometry_msgs::Point counter_score;
  std::string coin_id_pref = "coin_";
  std::string bomb_id_pref = "bomb_";

  for (int j=0; j<=9; j++){
    std::stringstream s;
    s << coin_id_pref << j;
    std::string coin_id = s.str();
    int mycount = std::count (msg->name.begin(), msg->name.end(), coin_id);
    coin_count += mycount;
  }

  for (int j=0; j<=4; j++){
    std::stringstream s;
    s << bomb_id_pref << j;
    std::string bomb_id = s.str();
    int mycount = std::count (msg->name.begin(), msg->name.end(), bomb_id);
    bomb_count += mycount;
  }
  //std::cout << "Coins Remaining " << coin_count  << " \n";
  //std::cout << "Bombs Remaining " << bomb_count  << " \n";
  global_coin_count = (10 - coin_count)*0.1;
  global_bomb_count = -(5-bomb_count)*0.2;
  counter_score.x = global_coin_count;
  counter_score.y = global_bomb_count;
  score_pub.publish(counter_score);

  coin_count = 0;
  bomb_count = 0;

}

void coinCallback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
  if (msg->states.size() > 0){
    std::string verify;
    verify = "coin";
    std::string toErase;
    toErase = "::chassis::box_collision";
    if (msg->states.size() == 2){
      std::vector<std::string> collision;

      std::string collision_2 = msg->states[0].collision2_name;
      std::string collision_1 = msg->states[0].collision1_name;
      collision.push_back(collision_1);
      collision.push_back(collision_2);

      for (int i=0; i<=1; i++){
        std::size_t found = collision[i].find(verify);
        if (found!=std::string::npos){

          // Search for the substring in string
          size_t pos = collision[i].find(toErase);

          if (pos != std::string::npos)
          {
            // If found then erase it from string
            collision[i].erase(pos, toErase.length());
          }

          gazebo_msgs::DeleteModel srv;
          srv.request.model_name = collision[i];
          //ROS_INFO("CONTACT COIN!: %s", collision[i].c_str());

          //ROS_INFO("CURRENT SCORE!: %f", global_coin_count+global_bomb_count);

          ros::service::call("/gazebo/delete_model", srv);
        }
      }
    }
  }
}

void bombCallback(const gazebo_msgs::ContactsState::ConstPtr& msg)
{
  if (msg->states.size() > 0){
    std::string verify;
    verify = "bomb";
    std::string toErase;
    toErase = "::chassis::box_collision";
    if (msg->states.size() == 2){
      std::vector<std::string> collision;

      std::string collision_2 = msg->states[0].collision2_name;
      std::string collision_1 = msg->states[0].collision1_name;
      collision.push_back(collision_1);
      collision.push_back(collision_2);

      for (int i=0; i<=1; i++){
        std::size_t found = collision[i].find(verify);
        if (found!=std::string::npos){

          // Search for the substring in string
          size_t pos = collision[i].find(toErase);

          if (pos != std::string::npos)
          {
            // If found then erase it from string
            collision[i].erase(pos, toErase.length());
          }

          gazebo_msgs::DeleteModel srv;
          srv.request.model_name = collision[i];
          //ROS_INFO("CONTACT BOMB: %s", collision[i].c_str());

          //ROS_INFO("CURRENT SCORE!: %f", global_coin_count+global_bomb_count);
          ros::service::call("/gazebo/delete_model", srv);
        }
      }
    }
  }
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "ros_plugins");


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  //Configuration
  ros::param::get("~ArucoId", ArucoId);
  ros::param::get("~ModelDirectory", ModelDirectory);

  std::string coin_directory = "/coin/model.sdf";
  std::string bomb_directory = "/bomb/model.sdf";
  std::string barrier_1_directory = "/barrier_1/model.sdf";
  std::string barrier_2_directory = "/barrier_2/model.sdf";
  std::string barrier_3_directory = "/barrier_3/model.sdf";
  std::string airplane_blue_directory = "/airplane_blue/model.sdf";
  std::string airplane_green_directory = "/airplane_green/model.sdf";
  std::string airplane_red_directory = "/airplane_red/model.sdf";

  std::string bike_blue_directory = "/bike_blue/model.sdf";
  std::string bike_green_directory = "/bike_green/model.sdf";
  std::string bike_red_directory = "/bike_red/model.sdf";

  std::string boat_blue_directory = "/boat_blue/model.sdf";
  std::string boat_green_directory = "/boat_green/model.sdf";
  std::string boat_red_directory = "/boat_red/model.sdf";

  std::string bottle_blue_directory = "/bottle_blue/model.sdf";
  std::string bottle_green_directory = "/bottle_green/model.sdf";
  std::string bottle_red_directory = "/bottle_red/model.sdf";

  std::string aruco1_directory = "/aruco_cube/model_1.sdf";
  std::string aruco2_directory = "/aruco_cube/model_2.sdf";
  std::string aruco3_directory = "/aruco_cube/model_3.sdf";
  std::string aruco4_directory = "/aruco_cube/model_4.sdf";
  std::string aruco5_directory = "/aruco_cube/model_5.sdf";
  std::string aruco6_directory = "/aruco_cube/model_6.sdf";
  std::string aruco7_directory = "/aruco_cube/model_7.sdf";
  std::string aruco8_directory = "/aruco_cube/model_8.sdf";
  std::string aruco9_directory = "/aruco_cube/model_9.sdf";
  std::string aruco10_directory = "/aruco_cube/model_10.sdf";
  std::string aruco11_directory = "/aruco_cube/model_11.sdf";
  std::string aruco12_directory = "/aruco_cube/model_12.sdf";


  //Model SDF parameter Input
  std::stringstream ss_coin;
  ss_coin << ModelDirectory << coin_directory;
  std::string coin = ss_coin.str();
  std::ifstream in(coin.c_str());

  std::stringstream ss_bomb;
  ss_bomb << ModelDirectory << bomb_directory;
  std::string bomb = ss_bomb.str();
  std::ifstream in_bomb(bomb.c_str());


  std::ifstream in_walls_1("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/barrier_1/model.sdf");
  std::ifstream in_walls_2("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/barrier_2/model.sdf");
  std::ifstream in_walls_3("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/barrier_3/model.sdf");

  std::ifstream in_obj_1("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/airplane_blue/model.sdf");
  std::ifstream in_obj_2("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/airplane_green/model.sdf");
  std::ifstream in_obj_3("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/airplane_red/model.sdf");

  std::ifstream in_obj_4("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/bike_blue/model.sdf");
  std::ifstream in_obj_5("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/bike_green/model.sdf");
  std::ifstream in_obj_6("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/bike_red/model.sdf");

  std::ifstream in_obj_7("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/boat_blue/model.sdf");
  std::ifstream in_obj_8("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/boat_green/model.sdf");
  std::ifstream in_obj_9("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/boat_red/model.sdf");

  std::ifstream in_obj_10("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/bottle_blue/model.sdf");
  std::ifstream in_obj_11("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/bottle_green/model.sdf");
  std::ifstream in_obj_12("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/bottle_red/model.sdf");

  std::ifstream info_point_1("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/aruco_cube/model_1.sdf");
  std::ifstream info_point_2("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/aruco_cube/model_2.sdf");
  std::ifstream info_point_3("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/aruco_cube/model_3.sdf");
  std::ifstream info_point_4("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/aruco_cube/model_4.sdf");
  std::ifstream info_point_5("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/aruco_cube/model_5.sdf");
  std::ifstream info_point_6("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/aruco_cube/model_6.sdf");
  std::ifstream info_point_7("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/aruco_cube/model_7.sdf");
  std::ifstream info_point_8("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/aruco_cube/model_8.sdf");
  std::ifstream info_point_9("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/aruco_cube/model_9.sdf");
  std::ifstream info_point_10("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/aruco_cube/model_10.sdf");
  std::ifstream info_point_11("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/aruco_cube/model_11.sdf");
  std::ifstream info_point_12("/home/ramon/workspace/robotics_lab/src/ros-muar-upm/labrob_gazebo/models/aruco_cube/model_12.sdf");


  std::stringstream buffer;
  std::stringstream buffer_bomb;
  std::stringstream buffer_barrier_1;
  std::stringstream buffer_barrier_2;
  std::stringstream buffer_barrier_3;
  std::stringstream buffer_obj_1;
  std::stringstream buffer_obj_2;
  std::stringstream buffer_obj_3;
  std::stringstream buffer_obj_4;
  std::stringstream buffer_obj_5;
  std::stringstream buffer_obj_6;
  std::stringstream buffer_obj_7;
  std::stringstream buffer_obj_8;
  std::stringstream buffer_obj_9;
  std::stringstream buffer_obj_10;
  std::stringstream buffer_obj_11;
  std::stringstream buffer_obj_12;

  std::stringstream buffer_info_1;
  std::stringstream buffer_info_2;
  std::stringstream buffer_info_3;
  std::stringstream buffer_info_4;
  std::stringstream buffer_info_5;
  std::stringstream buffer_info_6;
  std::stringstream buffer_info_7;
  std::stringstream buffer_info_8;
  std::stringstream buffer_info_9;
  std::stringstream buffer_info_10;
  std::stringstream buffer_info_11;
  std::stringstream buffer_info_12;


  buffer << in.rdbuf();
  buffer_bomb << in_bomb.rdbuf();
  buffer_barrier_1 << in_walls_1.rdbuf();
  buffer_barrier_2 << in_walls_2.rdbuf();
  buffer_barrier_3 << in_walls_3.rdbuf();
  buffer_obj_1 << in_obj_1.rdbuf();
  buffer_obj_2 << in_obj_2.rdbuf();
  buffer_obj_3 << in_obj_3.rdbuf();
  buffer_obj_4 << in_obj_4.rdbuf();
  buffer_obj_5 << in_obj_5.rdbuf();
  buffer_obj_6 << in_obj_6.rdbuf();
  buffer_obj_7 << in_obj_7.rdbuf();
  buffer_obj_8 << in_obj_8.rdbuf();
  buffer_obj_9 << in_obj_9.rdbuf();
  buffer_obj_10 << in_obj_10.rdbuf();
  buffer_obj_11 << in_obj_11.rdbuf();
  buffer_obj_12 << in_obj_12.rdbuf();

  buffer_info_1 << info_point_1.rdbuf();
  buffer_info_2 << info_point_2.rdbuf();
  buffer_info_3 << info_point_3.rdbuf();
  buffer_info_4 << info_point_4.rdbuf();
  buffer_info_5 << info_point_5.rdbuf();
  buffer_info_6 << info_point_6.rdbuf();
  buffer_info_7 << info_point_7.rdbuf();
  buffer_info_8 << info_point_8.rdbuf();
  buffer_info_9 << info_point_9.rdbuf();
  buffer_info_10 << info_point_10.rdbuf();
  buffer_info_11 << info_point_11.rdbuf();
  buffer_info_12 << info_point_12.rdbuf();


  gazebo_msgs::SpawnModel spawn;
  gazebo_msgs::SetModelState coin_state_set;
  gazebo_msgs::SetModelState state_set;
  std::string coin_id_pref = "coin_";
  std::string bomb_id_pref = "bomb_";
  std::string obj_id_pref = "obj_";
  std::string info_id_pref = "info_";

  std::string wall_1_id_pref = "wall_1_";
  std::string wall_2_id_pref = "wall_2_";
  std::string wall_3_id_pref = "wall_3_";
  std::string coin_body_suff = "::chassis";

  srand ( time(NULL) );

//  // COINS SPAWNING
//  //1st Quadrant

//  int min_x = 0;
//  int max_x = 9;
//  int min_y = -9;
//  int max_y = 0;
//  for (int j=0; j<=2; j++){
//    std::stringstream s;
//    s << coin_id_pref << j;
//    std::string coin_id = s.str();
//    ROS_INFO("I Spawned: %s", coin_id.c_str());

//    spawn.request.model_name = coin_id;
//    spawn.request.model_xml = buffer.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    coin_id.clear();
//  }

//  //2nd Quadrant

//  min_x = -9;
//  max_x = 0;
//  min_y = -9;
//  max_y = 0;
//  for (int j=3; j<=4; j++){
//    std::stringstream s;
//    s << coin_id_pref << j;
//    std::string coin_id = s.str();
//    ROS_INFO("I Spawned: %s", coin_id.c_str());

//    spawn.request.model_name = coin_id;
//    spawn.request.model_xml = buffer.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    coin_id.clear();
//  }

//  //3rd Quadrant

//  min_x = -9;
//  max_x = 0;
//  min_y = 0;
//  max_y = 9;
//  for (int j=5; j<=7; j++){
//    std::stringstream s;
//    s << coin_id_pref << j;
//    std::string coin_id = s.str();
//    ROS_INFO("I Spawned: %s", coin_id.c_str());

//    spawn.request.model_name = coin_id;
//    spawn.request.model_xml = buffer.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    coin_id.clear();
//  }

//  //4th Quadrant

//  min_x = 0;
//  max_x = 9;
//  min_y = 0;
//  max_y = 9;
//  for (int j=8; j<=9; j++){
//    std::stringstream s;
//    s << coin_id_pref << j;
//    std::string coin_id = s.str();
//    ROS_INFO("I Spawned: %s", coin_id.c_str());

//    spawn.request.model_name = coin_id;
//    spawn.request.model_xml = buffer.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    coin_id.clear();
//  }



//  // BOMBS SPAWNING
//  //1st Quadrant

//  min_x = 0;
//  max_x = 9;
//  min_y = -9;
//  max_y = 0;
//  for (int j=0; j<=0; j++){
//    std::stringstream s;
//    s << bomb_id_pref << j;
//    std::string bomb_id = s.str();
//    ROS_INFO("I Spawned: %s", bomb_id.c_str());

//    spawn.request.model_name = bomb_id;
//    spawn.request.model_xml = buffer_bomb.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    bomb_id.clear();
//  }

//  //2nd Quadrant

//  min_x = -9;
//  max_x = 0;
//  min_y = -9;
//  max_y = 0;
//  for (int j=1; j<=1; j++){
//    std::stringstream s;
//    s << bomb_id_pref << j;
//    std::string bomb_id = s.str();
//    ROS_INFO("I Spawned: %s", bomb_id.c_str());

//    spawn.request.model_name = bomb_id;
//    spawn.request.model_xml = buffer_bomb.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    bomb_id.clear();
//  }

//  //3rd Quadrant

//  min_x = -9;
//  max_x = 0;
//  min_y = 0;
//  max_y = 9;
//  for (int j=2; j<=2; j++){
//    std::stringstream s;
//    s << bomb_id_pref << j;
//    std::string bomb_id = s.str();
//    ROS_INFO("I Spawned: %s", bomb_id.c_str());

//    spawn.request.model_name = bomb_id;
//    spawn.request.model_xml = buffer_bomb.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    bomb_id.clear();
//  }

//  //4th mid Quadrant

//  min_x = 0;
//  max_x = 4.5;
//  min_y = 0;
//  max_y = 9;
//  for (int j=3; j<=3; j++){
//    std::stringstream s;
//    s << bomb_id_pref << j;
//    std::string bomb_id = s.str();
//    ROS_INFO("I Spawned: %s", bomb_id.c_str());

//    spawn.request.model_name = bomb_id;
//    spawn.request.model_xml = buffer_bomb.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    bomb_id.clear();
//  }

//  //4th 2nd mid Quadrant

//  min_x = 4.5;
//  max_x = 9;
//  min_y = 0;
//  max_y = 9;
//  for (int j=4; j<=4; j++){
//    std::stringstream s;
//    s << bomb_id_pref << j;
//    std::string bomb_id = s.str();
//    ROS_INFO("I Spawned: %s", bomb_id.c_str());

//    spawn.request.model_name = bomb_id;
//    spawn.request.model_xml = buffer_bomb.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    bomb_id.clear();
//  }

//  // WALL SPAWNING

//  tf::Matrix3x3 wall_rot;
//  int Roll = 0;
//  int Pitch = 0;
//  int Yaw;
//  int max_angle = 359;
//  int min_angle = 0;

//  //1st Quadrant

//  min_x = 0;
//  max_x = 8;
//  min_y = -8;
//  max_y = 0;

//  //Barrier 1
//  for (int j=0; j<=1; j++){
//    std::stringstream s;
//    s << wall_1_id_pref << j;
//    std::string wall_id = s.str();
//    ROS_INFO("I Spawned: %s", wall_id.c_str());

//    spawn.request.model_name = wall_id;
//    spawn.request.model_xml = buffer_barrier_1.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;

//    Yaw = ((float(rand()) / float(RAND_MAX)) * (max_angle - min_angle)) + min_angle;

//    wall_rot.setEulerYPR(Yaw, Pitch, Roll);
//    tf::Quaternion q_tf;
//    wall_rot.getRotation(q_tf);
//    spawn.request.initial_pose.orientation.w = q_tf.getW();
//    spawn.request.initial_pose.orientation.x = q_tf.getX();
//    spawn.request.initial_pose.orientation.y = q_tf.getY();
//    spawn.request.initial_pose.orientation.z = q_tf.getZ();
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    wall_id.clear();
//  }

//  //2nd Quadrant

//  min_x = -9;
//  max_x = 0;
//  min_y = -9;
//  max_y = 0;

//  //Barrier 1
//  for (int j=2; j<=2; j++){
//    std::stringstream s;
//    s << wall_1_id_pref << j;
//    std::string wall_id = s.str();
//    ROS_INFO("I Spawned: %s", wall_id.c_str());

//    spawn.request.model_name = wall_id;
//    spawn.request.model_xml = buffer_barrier_1.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;

//    Yaw = ((float(rand()) / float(RAND_MAX)) * (max_angle - min_angle)) + min_angle;

//    wall_rot.setEulerYPR(Yaw, Pitch, Roll);
//    tf::Quaternion q_tf;
//    wall_rot.getRotation(q_tf);
//    spawn.request.initial_pose.orientation.w = q_tf.getW();
//    spawn.request.initial_pose.orientation.x = q_tf.getX();
//    spawn.request.initial_pose.orientation.y = q_tf.getY();
//    spawn.request.initial_pose.orientation.z = q_tf.getZ();
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    wall_id.clear();
//  }

//  //Barrier 2
//  for (int j=0; j<=0; j++){
//    std::stringstream s;
//    s << wall_2_id_pref << j;
//    std::string wall_id_2 = s.str();
//    ROS_INFO("I Spawned: %s", wall_id_2.c_str());

//    spawn.request.model_name = wall_id_2;
//    spawn.request.model_xml = buffer_barrier_2.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;

//    Yaw = ((float(rand()) / float(RAND_MAX)) * (max_angle - min_angle)) + min_angle;

//    wall_rot.setEulerYPR(Yaw, Pitch, Roll);
//    tf::Quaternion q_tf;
//    wall_rot.getRotation(q_tf);
//    spawn.request.initial_pose.orientation.w = q_tf.getW();
//    spawn.request.initial_pose.orientation.x = q_tf.getX();
//    spawn.request.initial_pose.orientation.y = q_tf.getY();
//    spawn.request.initial_pose.orientation.z = q_tf.getZ();
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    wall_id_2.clear();
//  }

//  //Barrier 3
//  for (int j=0; j<=0; j++){
//    std::stringstream s;
//    s << wall_3_id_pref << j;
//    std::string wall_id_3 = s.str();
//    ROS_INFO("I Spawned: %s", wall_id_3.c_str());

//    spawn.request.model_name = wall_id_3;
//    spawn.request.model_xml = buffer_barrier_3.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;

//    Yaw = ((float(rand()) / float(RAND_MAX)) * (max_angle - min_angle)) + min_angle;

//    wall_rot.setEulerYPR(Yaw, Pitch, Roll);
//    tf::Quaternion q_tf;
//    wall_rot.getRotation(q_tf);
//    spawn.request.initial_pose.orientation.w = q_tf.getW();
//    spawn.request.initial_pose.orientation.x = q_tf.getX();
//    spawn.request.initial_pose.orientation.y = q_tf.getY();
//    spawn.request.initial_pose.orientation.z = q_tf.getZ();
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    wall_id_3.clear();
//  }



//  //3rd Quadrant

//  min_x = -9;
//  max_x = 0;
//  min_y = 0;
//  max_y = 9;

//  //Barrier 1
//  for (int j=3; j<=3; j++){
//    std::stringstream s;
//    s << wall_1_id_pref << j;
//    std::string wall_id = s.str();
//    ROS_INFO("I Spawned: %s", wall_id.c_str());

//    spawn.request.model_name = wall_id;
//    spawn.request.model_xml = buffer_barrier_1.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    Yaw = ((float(rand()) / float(RAND_MAX)) * (max_angle - min_angle)) + min_angle;

//    wall_rot.setEulerYPR(Yaw, Pitch, Roll);
//    tf::Quaternion q_tf;
//    wall_rot.getRotation(q_tf);
//    spawn.request.initial_pose.orientation.w = q_tf.getW();
//    spawn.request.initial_pose.orientation.x = q_tf.getX();
//    spawn.request.initial_pose.orientation.y = q_tf.getY();
//    spawn.request.initial_pose.orientation.z = q_tf.getZ();
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    wall_id.clear();
//  }

//  //Barrier 2
//  for (int j=1; j<=1; j++){
//    std::stringstream s;
//    s << wall_2_id_pref << j;
//    std::string wall_id_2 = s.str();
//    ROS_INFO("I Spawned: %s", wall_id_2.c_str());

//    spawn.request.model_name = wall_id_2;
//    spawn.request.model_xml = buffer_barrier_2.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    Yaw = ((float(rand()) / float(RAND_MAX)) * (max_angle - min_angle)) + min_angle;

//    wall_rot.setEulerYPR(Yaw, Pitch, Roll);
//    tf::Quaternion q_tf;
//    wall_rot.getRotation(q_tf);
//    spawn.request.initial_pose.orientation.w = q_tf.getW();
//    spawn.request.initial_pose.orientation.x = q_tf.getX();
//    spawn.request.initial_pose.orientation.y = q_tf.getY();
//    spawn.request.initial_pose.orientation.z = q_tf.getZ();
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    wall_id_2.clear();
//  }

//  //Barrier 3
//  for (int j=1; j<=1; j++){
//    std::stringstream s;
//    s << wall_3_id_pref << j;
//    std::string wall_id_3 = s.str();
//    ROS_INFO("I Spawned: %s", wall_id_3.c_str());

//    spawn.request.model_name = wall_id_3;
//    spawn.request.model_xml = buffer_barrier_3.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    Yaw = ((float(rand()) / float(RAND_MAX)) * (max_angle - min_angle)) + min_angle;

//    wall_rot.setEulerYPR(Yaw, Pitch, Roll);
//    tf::Quaternion q_tf;
//    wall_rot.getRotation(q_tf);
//    spawn.request.initial_pose.orientation.w = q_tf.getW();
//    spawn.request.initial_pose.orientation.x = q_tf.getX();
//    spawn.request.initial_pose.orientation.y = q_tf.getY();
//    spawn.request.initial_pose.orientation.z = q_tf.getZ();
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    wall_id_3.clear();
//  }

//  //4th Quadrant

//  min_x = 0;
//  max_x = 9;
//  min_y = 0;
//  max_y = 9;

//  //Barrier 1
//  for (int j=4; j<=4; j++){
//    std::stringstream s;
//    s << wall_1_id_pref << j;
//    std::string wall_id = s.str();
//    ROS_INFO("I Spawned: %s", wall_id.c_str());

//    spawn.request.model_name = wall_id;
//    spawn.request.model_xml = buffer_barrier_1.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    Yaw = ((float(rand()) / float(RAND_MAX)) * (max_angle - min_angle)) + min_angle;

//    wall_rot.setEulerYPR(Yaw, Pitch, Roll);
//    tf::Quaternion q_tf;
//    wall_rot.getRotation(q_tf);
//    spawn.request.initial_pose.orientation.w = q_tf.getW();
//    spawn.request.initial_pose.orientation.x = q_tf.getX();
//    spawn.request.initial_pose.orientation.y = q_tf.getY();
//    spawn.request.initial_pose.orientation.z = q_tf.getZ();
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    wall_id.clear();
//  }

//  //Barrier 2
//  for (int j=2; j<=2; j++){
//    std::stringstream s;
//    s << wall_2_id_pref << j;
//    std::string wall_id_2 = s.str();
//    ROS_INFO("I Spawned: %s", wall_id_2.c_str());

//    spawn.request.model_name = wall_id_2;
//    spawn.request.model_xml = buffer_barrier_2.str();
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    Yaw = ((float(rand()) / float(RAND_MAX)) * (max_angle - min_angle)) + min_angle;

//    wall_rot.setEulerYPR(Yaw, Pitch, Roll);
//    tf::Quaternion q_tf;
//    wall_rot.getRotation(q_tf);
//    spawn.request.initial_pose.orientation.w = q_tf.getW();
//    spawn.request.initial_pose.orientation.x = q_tf.getX();
//    spawn.request.initial_pose.orientation.y = q_tf.getY();
//    spawn.request.initial_pose.orientation.z = q_tf.getZ();
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    wall_id_2.clear();
//  }

//  // OBJECT SPAWNING
//  //1st Quadrant

//  min_x = 0;
//  max_x = 8;
//  min_y = -8;
//  max_y = 0;
//  for (int j=0; j<=2; j++){
//    std::stringstream s;
//    s << obj_id_pref << j;
//    std::string obj_id = s.str();
//    ROS_INFO("I Spawned: %s", obj_id.c_str());
//    spawn.request.model_name = obj_id;
//    switch (j+1) {
//    case 1:
//      spawn.request.model_xml = buffer_obj_1.str();
//      break;
//    case 2:
//      spawn.request.model_xml = buffer_obj_2.str();
//      break;
//    case 3:
//      spawn.request.model_xml = buffer_obj_3.str();
//      break;
//    case 4:
//      spawn.request.model_xml = buffer_obj_4.str();
//      break;
//    case 5:
//      spawn.request.model_xml = buffer_obj_5.str();
//      break;
//    case 6:
//      spawn.request.model_xml = buffer_obj_6.str();
//      break;
//    case 7:
//      spawn.request.model_xml = buffer_obj_7.str();
//      break;
//    case 8:
//      spawn.request.model_xml = buffer_obj_8.str();
//      break;
//    case 9:
//      spawn.request.model_xml = buffer_obj_9.str();
//      break;
//    case 10:
//      spawn.request.model_xml = buffer_obj_10.str();
//      break;
//    case 11:
//      spawn.request.model_xml = buffer_obj_11.str();
//      break;
//    case 12:
//      spawn.request.model_xml = buffer_obj_12.str();
//      break;
//    default:
//      break;
//    }
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    obj_id.clear();
//  }

//  //2nd Quadrant

//  min_x = -9;
//  max_x = 0;
//  min_y = -9;
//  max_y = 0;
//  for (int j=3; j<=5; j++){
//    std::stringstream s;
//    s << obj_id_pref << j;
//    std::string obj_id = s.str();
//    ROS_INFO("I Spawned: %s", obj_id.c_str());
//    spawn.request.model_name = obj_id;
//    switch (j+1) {
//    case 1:
//      spawn.request.model_xml = buffer_obj_1.str();
//      break;
//    case 2:
//      spawn.request.model_xml = buffer_obj_2.str();
//      break;
//    case 3:
//      spawn.request.model_xml = buffer_obj_3.str();
//      break;
//    case 4:
//      spawn.request.model_xml = buffer_obj_4.str();
//      break;
//    case 5:
//      spawn.request.model_xml = buffer_obj_5.str();
//      break;
//    case 6:
//      spawn.request.model_xml = buffer_obj_6.str();
//      break;
//    case 7:
//      spawn.request.model_xml = buffer_obj_7.str();
//      break;
//    case 8:
//      spawn.request.model_xml = buffer_obj_8.str();
//      break;
//    case 9:
//      spawn.request.model_xml = buffer_obj_9.str();
//      break;
//    case 10:
//      spawn.request.model_xml = buffer_obj_10.str();
//      break;
//    case 11:
//      spawn.request.model_xml = buffer_obj_11.str();
//      break;
//    case 12:
//      spawn.request.model_xml = buffer_obj_12.str();
//      break;
//    default:
//      break;
//    }
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    obj_id.clear();
//  }

//  //3rd Quadrant

//  min_x = -9;
//  max_x = 0;
//  min_y = 0;
//  max_y = 9;
//  for (int j=6; j<=8; j++){
//    std::stringstream s;
//    s << obj_id_pref << j;
//    std::string obj_id = s.str();
//    ROS_INFO("I Spawned: %s", obj_id.c_str());
//    spawn.request.model_name = obj_id;
//    switch (j+1) {
//    case 1:
//      spawn.request.model_xml = buffer_obj_1.str();
//      break;
//    case 2:
//      spawn.request.model_xml = buffer_obj_2.str();
//      break;
//    case 3:
//      spawn.request.model_xml = buffer_obj_3.str();
//      break;
//    case 4:
//      spawn.request.model_xml = buffer_obj_4.str();
//      break;
//    case 5:
//      spawn.request.model_xml = buffer_obj_5.str();
//      break;
//    case 6:
//      spawn.request.model_xml = buffer_obj_6.str();
//      break;
//    case 7:
//      spawn.request.model_xml = buffer_obj_7.str();
//      break;
//    case 8:
//      spawn.request.model_xml = buffer_obj_8.str();
//      break;
//    case 9:
//      spawn.request.model_xml = buffer_obj_9.str();
//      break;
//    case 10:
//      spawn.request.model_xml = buffer_obj_10.str();
//      break;
//    case 11:
//      spawn.request.model_xml = buffer_obj_11.str();
//      break;
//    case 12:
//      spawn.request.model_xml = buffer_obj_12.str();
//      break;
//    default:
//      break;
//    }
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    obj_id.clear();
//  }

//  //4th Quadrant

//  min_x = 0;
//  max_x = 9;
//  min_y = 0;
//  max_y = 9;
//  for (int j=9; j<=11; j++){
//    std::stringstream s;
//    s << obj_id_pref << j;
//    std::string obj_id = s.str();
//    ROS_INFO("I Spawned: %s", obj_id.c_str());
//    spawn.request.model_name = obj_id;
//    switch (j+1) {
//    case 1:
//      spawn.request.model_xml = buffer_obj_1.str();
//      break;
//    case 2:
//      spawn.request.model_xml = buffer_obj_2.str();
//      break;
//    case 3:
//      spawn.request.model_xml = buffer_obj_3.str();
//      break;
//    case 4:
//      spawn.request.model_xml = buffer_obj_4.str();
//      break;
//    case 5:
//      spawn.request.model_xml = buffer_obj_5.str();
//      break;
//    case 6:
//      spawn.request.model_xml = buffer_obj_6.str();
//      break;
//    case 7:
//      spawn.request.model_xml = buffer_obj_7.str();
//      break;
//    case 8:
//      spawn.request.model_xml = buffer_obj_8.str();
//      break;
//    case 9:
//      spawn.request.model_xml = buffer_obj_9.str();
//      break;
//    case 10:
//      spawn.request.model_xml = buffer_obj_10.str();
//      break;
//    case 11:
//      spawn.request.model_xml = buffer_obj_11.str();
//      break;
//    case 12:
//      spawn.request.model_xml = buffer_obj_12.str();
//      break;
//    default:
//      break;
//    }
//    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
//    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
//    ros::service::call("/gazebo/spawn_sdf_model", spawn);
//    obj_id.clear();
//  }

  //4th Quadrant Point of Information

  int min_x = -9;
  int max_x = -7;
  int min_y = 7;
  int max_y = 9;
  //for (int j=1; j<=1; j++){
    std::stringstream t;
    t << info_id_pref << ArucoId;
    std::string info_id = t.str();
    ROS_INFO("I Spawned: %s", info_id.c_str());
    spawn.request.model_name = info_id;
    switch (ArucoId) {
    case 1:
      spawn.request.model_xml = buffer_info_1.str();
      break;
    case 2:
      spawn.request.model_xml = buffer_info_2.str();
      break;
    case 3:
      spawn.request.model_xml = buffer_info_3.str();
      break;
    case 4:
      spawn.request.model_xml = buffer_info_4.str();
      break;
    case 5:
      spawn.request.model_xml = buffer_info_5.str();
      break;
    case 6:
      spawn.request.model_xml = buffer_info_6.str();
      break;
    case 7:
      spawn.request.model_xml = buffer_info_7.str();
      break;
    case 8:
      spawn.request.model_xml = buffer_info_8.str();
      break;
    case 9:
      spawn.request.model_xml = buffer_info_9.str();
      break;
    case 10:
      spawn.request.model_xml = buffer_info_10.str();
      break;
    case 11:
      spawn.request.model_xml = buffer_info_11.str();
      break;
    case 12:
      spawn.request.model_xml = buffer_info_12.str();
      break;
    default:
      break;
    }
    spawn.request.initial_pose.position.x = ((float(rand()) / float(RAND_MAX)) * (max_x - min_x)) + min_x;
    spawn.request.initial_pose.position.y = ((float(rand()) / float(RAND_MAX)) * (max_y - min_y)) + min_y;
    ros::service::call("/gazebo/spawn_sdf_model", spawn);
    info_id.clear();


//  int myints[]= {0,1,2,3,4,5,6,7,8,9,10,11};
//  std::vector<int> obj_vector;
//  std::list<int> mylist (myints,myints+12);
//  std::list<int>::iterator it1,it2;


//  mylist.remove(ArucoId-1);

//  for (std::list<int>::iterator it=mylist.begin(); it!=mylist.end(); ++it)
//    obj_vector.push_back(*it);
//    //std::cout << ' ' << *it;
//    //std::cout << '\n';

//  int delete_aruco_id_1, delete_aruco_id_2;

//  delete_aruco_id_1 = obj_vector[rand() % mylist.size()];
//  delete_aruco_id_2 = obj_vector[rand() % mylist.size()];

//  std::stringstream s;
//  s << obj_id_pref << delete_aruco_id_1;
//  std::string obj_id = s.str();
//  gazebo_msgs::DeleteModel srv;
//  srv.request.model_name = obj_id;


//  ros::service::call("/gazebo/delete_model", srv);

//  std::stringstream o;
//  o << obj_id_pref << delete_aruco_id_2;
//  std::string obj_id_2 = o.str();
//  //gazebo_msgs::DeleteModel srv;
//  srv.request.model_name = obj_id_2;


//  ros::service::call("/gazebo/delete_model", srv);




  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("coin_bumper", 1000, coinCallback);
  ros::Subscriber sub_2 = n.subscribe("bomb_bumper", 1000, bombCallback);
  ros::Subscriber sub_3 = n.subscribe("/gazebo/model_states", 1000, statesCallback);
  score_pub = n.advertise<geometry_msgs::Point>("robot_score", 1000);

//  for (int j=0; j<=9; j++){
//    std::stringstream s;
//    std::stringstream p;
//    s << coin_id_pref << j;
//    std::string coin_id = s.str();
//    p << coin_id << coin_body_suff;
//    std::string coin_body = p.str();
//    coin_state_set.request.model_state.model_name = coin_id;
//    coin_state_set.request.model_state.twist.angular.z = 5;
//    ros::service::call("/gazebo/set_model_state", coin_state_set);
//    coin_id.clear();
//    ros::Duration(0.4).sleep();
//  }


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
