#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <simple_navigation_goals/ObjectRVIZMappingAction.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <string>

//Constant paths to the meshes of each object
std::string PLANE = "package://labrob_gazebo/models/airplane_blue/meshes/plane_fixed.dae";
std::string BIKE = "package://labrob_gazebo/models/bike_blue/meshes/bike.dae";
std::string BOAT = "package://labrob_gazebo/models/boat_blue/meshes/boat_fixed.dae";
std::string BOTTLE = "package://labrob_gazebo/models/bottle_blue/meshes/bottle.dae";

//Object RVIZ Mapping Action Class
class ObjectRVIZMappingAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<simple_navigation_goals::ObjectRVIZMappingAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  simple_navigation_goals::ObjectRVIZMappingFeedback feedback_;
  simple_navigation_goals::ObjectRVIZMappingResult result_;
  
  //Object parameters
  std::string model;
  std::string model_mesh;
  std::string color;  
  ros::Publisher object_pub;

public:

  ObjectRVIZMappingAction(std::string name) :
    as_(nh_, name, boost::bind(&ObjectRVIZMappingAction::executeCB, this, _1), false),
    action_name_(name)
  {
    //Marker publisher
    object_pub = nh_.advertise<visualization_msgs::Marker>("object_mapping/pos",100);

    ROS_INFO("Object_RVIZ_Mapping_Server created.");
    as_.start();
  }

  ~ObjectRVIZMappingAction(void)
  {
  }

  void object_parameters(int object_id)
  {
    //Get object mesh
    if (object_id == 1 || object_id == 2 || object_id == 3)
    {
      model = "PLANE";
      model_mesh = PLANE;
    }
    else if (object_id == 4 || object_id == 5 || object_id == 6)
    {
      model = "BIKE";
      model_mesh = BIKE;
    }
    else if (object_id == 7 || object_id == 8 || object_id == 9)
    {
      model = "BOAT";
      model_mesh = BOAT;
    } 
    else if (object_id == 10 || object_id == 11 || object_id == 12)
    {
      model = "BOTTLE";
      model_mesh = BOTTLE;
    }

    //And color of the object
    if (object_id == 1 || object_id == 4 || object_id == 7 || object_id == 10)
    {
      color = "BLUE";
    }
    else if (object_id == 2 || object_id == 5 || object_id == 8 || object_id == 11)
    {
      color = "GREEN";
    }
    else if (object_id == 3 || object_id == 6 || object_id == 9 || object_id == 12)
    {
      color = "RED";
    } 
  }

  void executeCB(const simple_navigation_goals::ObjectRVIZMappingGoalConstPtr &goal)
  {
    
    // helper variables
    bool success = true;

    // start executing the action
    ROS_INFO("Starting action...");
    //get parameters of the found object
    object_parameters(goal->id);
    ROS_INFO("The object found whose id is %i corresponds to the %s %s",goal->id, color.c_str(), model.c_str());
    //ROS_INFO("Sending goals %f, %f, %f...", goal->pos_x, goal->pos_y, goal->pos_z);

    //create message to visualize the object
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map"; 
    marker.header.stamp = ros::Time();
    //marker.ns = "my_namespace";
    marker.id = goal->id;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD; 
    marker.pose.position.x = goal->pos_x; 
    marker.pose.position.y = goal->pos_y;
    marker.pose.position.z = goal->pos_z;
    marker.pose.orientation.x = -0.707;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.707;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    if (color == "RED"){marker.color.r=1.0;}
    if (color == "GREEN"){marker.color.g=1.0;}
    if (color == "BLUE"){marker.color.b=1.0;}
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = model_mesh;
    object_pub.publish(marker);

    // Send sucess result    
    if(success)
    {
      result_.result = 1;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Object_RVIZ_Mapping_Server");

  ObjectRVIZMappingAction map_object("map_Object");
  ros::spin();

  return 0;
}

// start executing the action
    // check that preempt has not been requested by the client
    /*if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
    }
    
    feedback_.feedback=1; //1=OK, 0=PROBLEM
    as_.publishFeedback(feedback_);
    */
