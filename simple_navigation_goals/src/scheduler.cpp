#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Pose.h"
#include <string>
#include <sstream>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include <vector>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "simple_navigation_goals/getWorldPose.h"
#include <explore_lite/setExpRes.h>
#include "gazebo_msgs/ContactsState.h"
#include "gazebo_msgs/ModelStates.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalStatus.h"

#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_GREEN   "\x1b[32m"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Scheduler
{
	public:
		int Exploring;		
		int Stage;	//0 = Searching for Aruco
						//1 = Exploring
						//2 = Rescuing object
						//3 = Returning for Aruco

		int Aruco_PosCorrection;
		bool Aruco_Found;
		bool Aruco_NotFoundPath;
		bool Aruco_Close;
		bool Aruco_UpdatePose;
 		float Aruco_Pose[2];
		float Aruco_PoseSearch[2];

		int Coin_Issue;
		int Coin_Counter;
		bool Coin_Exception;
		bool Coin_DisableSearch;
		float Coin_ProbabilityPose[4];

		float DistanceUpdate;
		float DistanceSearch;

		int DesiredObject;
		int DesiredObject_Approuch;
		bool DesiredObject_NotFoundPath;
		bool DesiredObj_InFront;
		bool DesiredObj_Update;
		float DesiredObject_PoseSearch[2];
		float ObjectPose [12][6];	//	1. Object Already Detected (0=no, 1=yes)
											//	2. Full ID which represents the object (1-->12)
											//	3. Probability of detection (0-->1)
											//	4. X world coordinate
											//	5. Y world coordinate
											//	6. Z world coordinate

		ros::ServiceClient tf_map_cam_client;
		ros::ServiceClient exp_client;
		actionlib_msgs::GoalID stop_nav;
	   MoveBaseClient ac;

		Scheduler();

		int Get_ObjectID(std::string obj_name);
		void Explorer_Comunication(int signal_2_exp);
		void ArucoID_Callback(const std_msgs::UInt8::ConstPtr& msg);
		void ArucoPose_Callback(const geometry_msgs::Pose::ConstPtr& msg);
		void Coin_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg);
		void Object_RVIZSpawn(int object_ID, float x, float y, float z);
		void Recover_StatusMission();
		void Coin_IssueChecker();
	   void Send_Goal(float x, float y);
		void YOLO_Callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);

};


Scheduler::Scheduler() : ac("move_base", true)
{
	Exploring = 0;	
	Stage = 10;
   
	DistanceUpdate = 1.75;
	DistanceSearch = 1;

	Aruco_PosCorrection=0;
	Aruco_Found = false;
	Aruco_NotFoundPath = false;
	Aruco_Close = true;
	Aruco_UpdatePose = false;
	Aruco_Pose[0] = -16;
	Aruco_Pose[1] = 16.5;
	Aruco_PoseSearch[0] = Aruco_Pose[0] - DistanceSearch;
	Aruco_PoseSearch[1] = Aruco_Pose[1];

	Coin_Issue = 0;
	Coin_Counter = 0;
	Coin_Exception = false;
	Coin_DisableSearch = true;
	for (int i=0; i<4; i++)
		Coin_ProbabilityPose[i]=0.0;

	DesiredObject = 0;
	DesiredObject_Approuch = 0;
	DesiredObject_NotFoundPath = false;
	DesiredObj_InFront = false;
	DesiredObj_Update = false;
	for (int i=0; i<12; i++)
		for (int j=0; j<6; j++)
			ObjectPose[i][j]=0.0;

  	ac.waitForServer();
  	ROS_INFO("Move Base Client Started");
}


void Scheduler::Explorer_Comunication(int signal_2_exp)
{
  ac.cancelGoal();
  ROS_INFO("Comunicating with Explorer %i", signal_2_exp);
  explore_lite::setExpRes srv_exp;
  srv_exp.request.cmd = signal_2_exp;
  exp_client.call(srv_exp);
  ros::Duration(1).sleep();
}


int Scheduler::Get_ObjectID(std::string obj_name)
{
	if			(obj_name == "avion azul")			{return 1;}
	else if	(obj_name == "avion verde")		{return 2;}
	else if	(obj_name == "avion rojo")			{return 3;}
	else if	(obj_name == "bicicleta azul")	{return 4;}
	else if	(obj_name == "bicicleta verde")	{return 5;}
	else if	(obj_name == "bicicleta roja")	{return 6;}
	else if	(obj_name == "barco azul")			{return 7;}
	else if	(obj_name == "barco verde")		{return 8;}
	else if	(obj_name == "barco rojo")			{return 9;}
	else if	(obj_name == "botella azul")		{return 10;}
	else if	(obj_name == "botella verde")		{return 11;}
	else if	(obj_name == "botella roja")		{return 12;}
}


void Scheduler::ArucoID_Callback(const std_msgs::UInt8::ConstPtr& msg)	//Aruco id callback
{
	if (Aruco_Found == false)
	{ 
		ROS_INFO("Aruco marker %i found", msg->data);
		DesiredObject = msg->data;
		Aruco_Found = true;
		Aruco_UpdatePose = true;
	}
}


void Scheduler::ArucoPose_Callback(const geometry_msgs::Pose::ConstPtr& msg) //Aruco pose callback
{
	if (Aruco_UpdatePose == true && msg->position.z < 3.0) 
	{
		Aruco_UpdatePose = false;
		ROS_INFO("Aruco found at [%f, %f, %f]", msg->position.x, msg->position.y, msg->position.z); 
	} 

	if (msg->position.z < 2.0)
	{
		ROS_INFO("Aruco marker at %f meters.", msg->position.z);
		Aruco_Close = true;
	}
}


void Scheduler::Coin_Callback(const gazebo_msgs::ContactsState::ConstPtr& msg)	//Coin hit callback
{
	if (msg->states.size() > 0)
	{
		Coin_Exception = false;
		Coin_Issue = 0;
		Coin_Counter = Coin_Counter + 1;
		ROS_INFO("%i coin catched!",Coin_Counter);
		Recover_StatusMission();
	}
}


void Scheduler::Object_RVIZSpawn(int object_ID, float x, float y, float z)
{
	std::ostringstream arg1;
	std::ostringstream arg2;
	std::ostringstream arg3;
	std::ostringstream arg4;
	arg1 << object_ID;
	arg2 << x;
	arg3 << y;
	arg4 << z;
	std::string cmd = "rosrun simple_navigation_goals object_mapping_request " + arg1.str() + " " + arg2.str() + " " + arg3.str() + " " + arg4.str(); 
	system(cmd.c_str());
}


void Scheduler::Recover_StatusMission()	//Recover principal status
{
	if (Stage == 0)
	{
		ROS_INFO("Continuing with the Aruco search");
		Send_Goal(Aruco_PoseSearch[0], Aruco_PoseSearch[1]);
	}
	else if (Stage == 1)
	{
		ROS_INFO("Continuing with the exploration");
		Explorer_Comunication(1);
	}
	else if (Stage == 2)
	{
		ROS_INFO("Continuing with the rescuing of the desired object");
		Send_Goal(DesiredObject_PoseSearch[0], DesiredObject_PoseSearch[1]);
	}
	else if (Stage == 3)
	{
		ROS_INFO("Continuing with the going to the initial point");
		Send_Goal(0,0);
	}
}


void Scheduler::Coin_IssueChecker()	//Coin problems, forget about coin and keep up with the mission
{
	if (Coin_Exception == true && ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	if (Coin_Exception == true)
	{
		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED  || ac.getState() == actionlib::SimpleClientGoalState::REJECTED ||
			 ac.getState() == actionlib::SimpleClientGoalState::ABORTED    || ac.getState() == actionlib::SimpleClientGoalState::LOST) //|| (Coin_Exception == true && current_explore_status ==3 && coin_issues >= 5 && stage == 1))
		{
			Coin_Issue = Coin_Issue + 1;
			if (Coin_Issue >=5)
			{
				Coin_Exception = false;
				Coin_Issue = 0;
				ROS_INFO("Map drifts made impossible to get coin, continuing with mission");
				Recover_StatusMission();
			}
		}
	}
}


void Scheduler::Send_Goal(float x, float y)
{
	if (Stage == 0 || Stage == 2 || Stage == 3)
		ac.cancelGoal();
	if (Stage == 1 && Coin_Exception == false)
		Explorer_Comunication(0);

	ROS_INFO("Current goal canceled, sending new goal");

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	goal.target_pose.pose.orientation.w = 1.0;
  	
	ros::Duration(1).sleep();
	ac.sendGoal(goal);
}


void Scheduler::YOLO_Callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)	//Yolo object found callback
{
	int local_id = 0;
	int arr_ind = 0;
	for (int i = 0; i < msg->bounding_boxes.size(); i++) 
	{
		if (msg->bounding_boxes[i].Class != "moneda" && msg->bounding_boxes[i].Class != "bomba")
		{
			if (msg->bounding_boxes[i].Invalid == false && !isnan(msg->bounding_boxes[i].X) && !isnan(msg->bounding_boxes[i].Y) && !isnan(msg->bounding_boxes[i].Z) && msg->bounding_boxes[i].probability >=0.3)
			{
				local_id = Get_ObjectID(msg->bounding_boxes[i].Class);
				arr_ind = local_id - 1;

				if (Stage == 2 && Aruco_Found == true && DesiredObject == local_id)
				{
					DesiredObj_InFront = true;
					ROS_INFO("The desired object is in front");
				}

				switch((int)ObjectPose[arr_ind][0])
				{
					case 0: //First entry of the object
					{
						ROS_INFO(ANSI_COLOR_BLUE "First view of the object %i, saving coordinates" ANSI_COLOR_BLUE, local_id );
						ObjectPose[arr_ind][0]=1;
						ObjectPose[arr_ind][1]=local_id;
						ObjectPose[arr_ind][2]=msg->bounding_boxes[i].probability;
						simple_navigation_goals::getWorldPose srv;
						srv.request.cam_x = msg->bounding_boxes[i].X;
						srv.request.cam_y = msg->bounding_boxes[i].Y;
						srv.request.cam_z = msg->bounding_boxes[i].Z;
						tf_map_cam_client.call(srv);
						ObjectPose[arr_ind][3] = srv.response.world_x;
						ObjectPose[arr_ind][4] = srv.response.world_y;
						ObjectPose[arr_ind][5] = srv.response.world_z;
						Object_RVIZSpawn(ObjectPose[arr_ind][1], ObjectPose[arr_ind][3], ObjectPose[arr_ind][4], ObjectPose[arr_ind][5]);
						break;
					}
					case 1: //Object already detected
					{ 
						ROS_INFO(ANSI_COLOR_BLUE "Object %i already detected, checking previous estimation" ANSI_COLOR_BLUE, local_id);
						if (ObjectPose[arr_ind][2] > msg->bounding_boxes[i].probability)
						{
							ROS_INFO(ANSI_COLOR_BLUE "Worse estimation, skipping coordinates update" ANSI_COLOR_BLUE);
							break;
						}
						else
						{
							ROS_INFO(ANSI_COLOR_BLUE "Better estimation, updating coordinates..." ANSI_COLOR_BLUE);
							ObjectPose[arr_ind][2]=msg->bounding_boxes[i].probability;
							simple_navigation_goals::getWorldPose srv;
							srv.request.cam_x = msg->bounding_boxes[i].X;
							srv.request.cam_y = msg->bounding_boxes[i].Y;
							srv.request.cam_z = msg->bounding_boxes[i].Z;
							tf_map_cam_client.call(srv);
							ObjectPose[arr_ind][3] = srv.response.world_x;
							ObjectPose[arr_ind][4] = srv.response.world_y;
							ObjectPose[arr_ind][5] = srv.response.world_z;
							Object_RVIZSpawn(ObjectPose[arr_ind][1], ObjectPose[arr_ind][3], ObjectPose[arr_ind][4], ObjectPose[arr_ind][5]);
							
							if (Stage == 2 && DesiredObject == local_id)
								DesiredObj_Update = true;

							break;
						}
					}
					default:
					{
						break;
					}
				}
			}
		}

		else if (msg->bounding_boxes[i].Class == "moneda" && Coin_DisableSearch == false)
		{
			if (msg->bounding_boxes[i].Invalid == false)
			{
				//Position of the coin
				simple_navigation_goals::getWorldPose srv;
				srv.request.cam_x = msg->bounding_boxes[i].X;
				srv.request.cam_y = msg->bounding_boxes[i].Y;
				srv.request.cam_z = msg->bounding_boxes[i].Z;
				tf_map_cam_client.call(srv);
				float c_x = srv.response.world_x;
				float c_y = srv.response.world_y;	
				float c_z = srv.response.world_z;

				//Position of the robot
				srv.request.cam_x = 0;
				srv.request.cam_y = 0;
				srv.request.cam_z = 0;
				tf_map_cam_client.call(srv);
				float rob_x = srv.response.world_x;
				float rob_y = srv.response.world_y;
				float rob_z = srv.response.world_z;

				//Distance between the coin and the robot
				float dist_x = msg->bounding_boxes[i].X;
				float dist_y = msg->bounding_boxes[i].Y; 
				float dist_z = msg->bounding_boxes[i].Z;
				float dist_t = sqrt(dist_x*dist_x + dist_z*dist_z);

				//If we are not navigating against a coin and the coin is close
				if (Coin_Exception == false)
				{
					if (dist_t <= 3.5)
					{
						ROS_INFO(ANSI_COLOR_YELLOW "Near coin found! Heading against it to win some points!" ANSI_COLOR_YELLOW);
						Coin_Exception = true;
						Coin_Issue = 0;
						DistanceUpdate = 1.5;

						ROS_INFO(ANSI_COLOR_YELLOW "KINECT: Coin in [%f, %f, %f]" ANSI_COLOR_YELLOW,c_x,c_y,c_z);
						Coin_ProbabilityPose[0] = msg->bounding_boxes[i].probability;
						Coin_ProbabilityPose[1] = c_x;
						Coin_ProbabilityPose[2] = c_y;
						Coin_ProbabilityPose[3] = c_z;

						int x_trend = -1; int y_trend = -1;
						if (c_x - rob_x > 0){x_trend = 1;}
						if (c_y - rob_y > 0){y_trend = 1;}
						float margin = 0.3;
 
						Send_Goal(c_x + margin*x_trend, c_y + margin*y_trend);
					}
				}

				else //we are already navigating against a coin
				{
					if (dist_t <= DistanceUpdate)
					{
						DistanceUpdate = 0; //Only one updates possibles 

						ROS_INFO(ANSI_COLOR_YELLOW "Same coin Founded! Updated position from [%f,%f] to [%f,%f]" ANSI_COLOR_YELLOW, Coin_ProbabilityPose[1],Coin_ProbabilityPose[2],c_x,c_y);

						Coin_ProbabilityPose[0] = msg->bounding_boxes[i].probability;
						Coin_ProbabilityPose[1] = c_x;
						Coin_ProbabilityPose[2] = c_y;
						Coin_ProbabilityPose[3] = c_z;
           
						int x_trend = -1; int y_trend = -1;
						if (c_x - rob_x > 0) {x_trend = 1;}
						if (c_y - rob_y > 0){y_trend = 1;}
						float margin = 0.3;
 
						Send_Goal(c_x + margin*x_trend, c_y + margin*y_trend);
					}
				}
			}
		}
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "Scheduler");
	ros::NodeHandle n;

	Scheduler Scheduler;

	//Aruco publishers and subscriptors
	ros::Subscriber aruco_id_sub = n.subscribe("aruco_simple/arucoID", 1, &Scheduler::ArucoID_Callback, &Scheduler);
	ros::Subscriber Aruco_Pose_sub = n.subscribe("aruco_simple/pose", 1, &Scheduler::ArucoPose_Callback, &Scheduler);
	ros::Subscriber yolo_sub = n.subscribe("darknet_ros/bounding_boxes", 1, &Scheduler::YOLO_Callback, &Scheduler);
	Scheduler.tf_map_cam_client = n.serviceClient<simple_navigation_goals::getWorldPose>("get_world_pose");
   Scheduler.exp_client = n.serviceClient<explore_lite::setExpRes>("/explore_lite_server");

	ros::Subscriber coin_bumper = n.subscribe("coin_bumper", 10, &Scheduler::Coin_Callback, &Scheduler);
 
	ROS_INFO("Stage 1: Searching for the Aruco Marker");
	Scheduler.Send_Goal(Scheduler.Aruco_PoseSearch[0], Scheduler.Aruco_PoseSearch[1]);
	Scheduler.Stage = 0;

	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		if (Scheduler.Coin_Exception == true)
		{
			Scheduler.Coin_IssueChecker(); //Problem with coin
		}
		else
		{
			//Stage 0 - Searching for Aruco
			if (Scheduler.Stage == 0)
			{
				if (Scheduler.Aruco_Found == true) 
				{
					Scheduler.Stage=1;
					Scheduler.ac.cancelGoal();
					ROS_INFO("The desired object is the number %i", Scheduler.DesiredObject);
					
					if (Scheduler.ObjectPose[Scheduler.DesiredObject-1][0] == 1)
					{
						ROS_INFO("Object Already detected at [%f, %f]", Scheduler.ObjectPose[Scheduler.DesiredObject-1][3], Scheduler.ObjectPose[Scheduler.DesiredObject-1][4]);
						ROS_INFO("Stage 1: Exploring the area to search all the objects");
					}
					else
					{
						ROS_INFO("Stage 1: Exploring the area to search all the objects and the desired one");
					}
	
					//Launch of Stage 1 - Exploration
					Scheduler.Explorer_Comunication(1);
				}
	
				else if (Scheduler.ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || Scheduler.ac.getState() == actionlib::SimpleClientGoalState::REJECTED ||
						   Scheduler.ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
				{
					Scheduler.Aruco_PosCorrection=Scheduler.Aruco_PosCorrection + 1;
					if (Scheduler.Aruco_PosCorrection <= 7)
					{
						ROS_INFO("Searching the Aruco in a new position, attempt %i", Scheduler.Aruco_PosCorrection);
	
						if (Scheduler.Aruco_PosCorrection == 1)
						{
							Scheduler.Aruco_PoseSearch[0] = Scheduler.Aruco_Pose[0] + Scheduler.DistanceSearch;
							Scheduler.Aruco_PoseSearch[1] = Scheduler.Aruco_Pose[1] ;
						}
						if (Scheduler.Aruco_PosCorrection == 2)
						{
							Scheduler.Aruco_PoseSearch[0] = Scheduler.Aruco_Pose[0] - Scheduler.DistanceSearch;
							Scheduler.Aruco_PoseSearch[1] = Scheduler.Aruco_Pose[1] - Scheduler.DistanceSearch;
						}
						if (Scheduler.Aruco_PosCorrection == 3)
						{
							Scheduler.Aruco_PoseSearch[0] = Scheduler.Aruco_Pose[0] + Scheduler.DistanceSearch;
							Scheduler.Aruco_PoseSearch[1] = Scheduler.Aruco_Pose[1] + Scheduler.DistanceSearch;
						}
						if (Scheduler.Aruco_PosCorrection == 4)
						{
							Scheduler.Aruco_PoseSearch[0] = Scheduler.Aruco_Pose[0];
							Scheduler.Aruco_PoseSearch[1] = Scheduler.Aruco_Pose[1] - Scheduler.DistanceSearch;
						}
						if (Scheduler.Aruco_PosCorrection == 5)
						{
							Scheduler.Aruco_PoseSearch[0] = Scheduler.Aruco_Pose[0];
							Scheduler.Aruco_PoseSearch[1] = Scheduler.Aruco_Pose[1] + Scheduler.DistanceSearch;
						}
						if (Scheduler.Aruco_PosCorrection == 6)
						{
							Scheduler.Aruco_PoseSearch[0] = Scheduler.Aruco_Pose[0] + Scheduler.DistanceSearch;
							Scheduler.Aruco_PoseSearch[1] = Scheduler.Aruco_Pose[1] - Scheduler.DistanceSearch;
						}
						if (Scheduler.Aruco_PosCorrection == 7)
						{
							Scheduler.Aruco_PoseSearch[0] = Scheduler.Aruco_Pose[0] - Scheduler.DistanceSearch;
							Scheduler.Aruco_PoseSearch[1] = Scheduler.Aruco_Pose[1] + Scheduler.DistanceSearch;
						}
						Scheduler.Send_Goal(Scheduler.Aruco_PoseSearch[0], Scheduler.Aruco_PoseSearch[1]);
					}
					else
					{
						Scheduler.Stage=1;
						ROS_INFO("Aruco search faild");
						ROS_INFO("Stage 1: Exploring the area to search the Aruco and all the objects");
						//Launch of Stage 1 - Exploration
						Scheduler.Explorer_Comunication(1);
					}
				}
			}
	  
			//Stage 1 - Exploring 
			else if (Scheduler.Stage == 1)
			{
				if (Scheduler.Aruco_Found == true)
				{
					n.getParam("/Exploring", Scheduler.Exploring);
					if ((int)Scheduler.ObjectPose[0][0]==1 && (int)Scheduler.ObjectPose[1][0]==1 && (int)Scheduler.ObjectPose[2][0]==1 && (int)Scheduler.ObjectPose[3][0]==1 &&
						 (int)Scheduler.ObjectPose[4][0]==1 && (int)Scheduler.ObjectPose[5][0]==1 && (int)Scheduler.ObjectPose[6][0]==1 && (int)Scheduler.ObjectPose[7][0]==1 &&
						 (int)Scheduler.ObjectPose[8][0]==1 && (int)Scheduler.ObjectPose[9][0]==1 && (int)Scheduler.ObjectPose[10][0]==1 && (int)Scheduler.ObjectPose[11][0]==1)
					{
						ROS_INFO("All the objects have been found");
						ROS_INFO("Stage 2: Rescuing the desired object");
						ROS_INFO("Object detected at [%f, %f]", Scheduler.ObjectPose[Scheduler.DesiredObject-1][3], Scheduler.ObjectPose[Scheduler.DesiredObject-1][4]);
						Scheduler.Send_Goal((Scheduler.ObjectPose[Scheduler.DesiredObject-1][3]-Scheduler.DistanceSearch), (Scheduler.ObjectPose[Scheduler.DesiredObject-1][4]));
						Scheduler.Stage=2;
					}
					else if (Scheduler.Exploring == 0)
					{
						if ((int)Scheduler.ObjectPose[Scheduler.DesiredObject-1][0]==1)
						{
							ROS_INFO("Exploration completed");
							ROS_INFO("Stage 2: Rescuing the desired object");
							ROS_INFO("Object detected at [%f, %f]", Scheduler.ObjectPose[Scheduler.DesiredObject-1][3], Scheduler.ObjectPose[Scheduler.DesiredObject-1][4]);
							Scheduler.Send_Goal((Scheduler.ObjectPose[Scheduler.DesiredObject-1][3]-Scheduler.DistanceSearch), (Scheduler.ObjectPose[Scheduler.DesiredObject-1][4]));
							Scheduler.Stage=2;
						}
						else
						{
							ROS_INFO("The desired object was not found");
							ROS_INFO("Stage 3:  Going to the initial point");
						   Scheduler.Send_Goal(0, 0);
							Scheduler.Stage = 3;
							Scheduler.DesiredObject_NotFoundPath = true;
						}
					}
				}
	
				else
				{
					n.getParam("/Exploring", Scheduler.Exploring);
					if (Scheduler.Exploring == 0)
					{
						ROS_INFO("The Aruco was not found");
						ROS_INFO("Stage 3:  Going to the initial point");
						Scheduler.Send_Goal(0, 0);
						Scheduler.Stage = 3;
						Scheduler.Aruco_NotFoundPath = true;
					}
				}
			}
	
			//Stage 2 - Rescuing object 
			else if (Scheduler.Stage == 2)
			{
				if (Scheduler.DesiredObj_InFront == true && Scheduler.ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				{
					ROS_INFO(ANSI_COLOR_GREEN "Desired object is in front of the robot" ANSI_COLOR_GREEN);
					ros::Duration(5).sleep();
					ROS_INFO("Stage 3: Going to the initial point");
					Scheduler.Send_Goal(0,0);
					Scheduler.Stage = 3;
				}

				else if (Scheduler.DesiredObject_Approuch >= 1 && Scheduler.DesiredObj_InFront == true)
				{
					ROS_INFO(ANSI_COLOR_GREEN "Desired object is in front of the robot" ANSI_COLOR_GREEN);
					ros::Duration(5).sleep();
					ROS_INFO("Stage 3: Going to the initial point");
					Scheduler.Send_Goal(0,0);
					Scheduler.Stage = 3;
					Scheduler.DesiredObj_InFront = false;
				}
	
				else if (Scheduler.DesiredObj_Update == true)
				{
					ROS_INFO("Updating desired object position");
					Scheduler.Send_Goal((Scheduler.ObjectPose[Scheduler.DesiredObject - 1][3] - Scheduler.DistanceSearch), (Scheduler.ObjectPose[Scheduler.DesiredObject - 1][4]));
					Scheduler.DesiredObject_Approuch=0;
					Scheduler.DesiredObj_Update = false;
				}
	
				else if (Scheduler.ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || Scheduler.ac.getState() == actionlib::SimpleClientGoalState::REJECTED ||
						   Scheduler.ac.getState() == actionlib::SimpleClientGoalState::ABORTED   || Scheduler.ac.getState() == actionlib::SimpleClientGoalState::LOST)
				{
					Scheduler.DesiredObject_Approuch=Scheduler.DesiredObject_Approuch + 1;
	
					if (Scheduler.DesiredObject_Approuch <= 7)
					{
						ROS_INFO("Searching the Object in the surroundings, attempt %i", Scheduler.DesiredObject_Approuch);
	
						if (Scheduler.DesiredObject_Approuch == 1)
						{
							Scheduler.DesiredObject_PoseSearch[0] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][3] + Scheduler.DistanceSearch;
							Scheduler.DesiredObject_PoseSearch[1] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][4];
						}
						if (Scheduler.DesiredObject_Approuch == 2)
						{
							Scheduler.DesiredObject_PoseSearch[0] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][3] - Scheduler.DistanceSearch;
							Scheduler.DesiredObject_PoseSearch[1] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][4] - Scheduler.DistanceSearch;
						}
						if (Scheduler.DesiredObject_Approuch == 3)
						{
							Scheduler.DesiredObject_PoseSearch[0] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][3] + Scheduler.DistanceSearch;
							Scheduler.DesiredObject_PoseSearch[1] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][4] + Scheduler.DistanceSearch;
						}
						if (Scheduler.DesiredObject_Approuch == 4)
						{
							Scheduler.DesiredObject_PoseSearch[0] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][3];
							Scheduler.DesiredObject_PoseSearch[1] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][4] - Scheduler.DistanceSearch;
						}
						if (Scheduler.DesiredObject_Approuch == 5)
						{
							Scheduler.DesiredObject_PoseSearch[0] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][3];
							Scheduler.DesiredObject_PoseSearch[1] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][4] + Scheduler.DistanceSearch;
						}
						if (Scheduler.DesiredObject_Approuch == 6)
						{
							Scheduler.DesiredObject_PoseSearch[0] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][3] + Scheduler.DistanceSearch;
							Scheduler.DesiredObject_PoseSearch[1] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][4] - Scheduler.DistanceSearch;
						}
						if (Scheduler.DesiredObject_Approuch == 7)
						{
							Scheduler.DesiredObject_PoseSearch[0] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][3] - Scheduler.DistanceSearch;
							Scheduler.DesiredObject_PoseSearch[1] = Scheduler.ObjectPose[Scheduler.DesiredObject - 1][4] + Scheduler.DistanceSearch;
						}
						Scheduler.Send_Goal(Scheduler.DesiredObject_PoseSearch[0], Scheduler.DesiredObject_PoseSearch[1]);
					}
	
					else
					{
						Scheduler.Stage=3;
						ROS_INFO("Object search faild");
						ROS_INFO("Stage 3: Going to the initial point");
						Scheduler.Stage = 3;
						Scheduler.Send_Goal(0, 0);
					}
				}
			}
			
			else if (Scheduler.Stage ==3)
			{
				if (Scheduler.Aruco_NotFoundPath == true && Scheduler.Aruco_Found == true)
				{
					Scheduler.Aruco_NotFoundPath = false;
					Scheduler.Stage = 0;
				}
				else if (Scheduler.DesiredObject_NotFoundPath == true && (int)Scheduler.ObjectPose[Scheduler.DesiredObject-1][0]==1)
				{
					Scheduler.DesiredObject_NotFoundPath = false;
					Scheduler.Stage = 1;
				}
			}
		}
   	
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}	
