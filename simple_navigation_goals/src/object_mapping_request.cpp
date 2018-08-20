#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <simple_navigation_goals/ObjectRVIZMappingAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "object_mapping_request");
  if(argc < 4) {
        printf("You must provide the id number of the object and the [X,Y,Z] coordinates to visualize\n");
        exit(0);
    }

  int object_ID = atoi(argv[1]);
  float pos_x = atof(argv[2]);
  float pos_y = atof(argv[3]);
  float pos_z = atof(argv[4]);

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<simple_navigation_goals::ObjectRVIZMappingAction> ac("map_Object", true);

  ROS_INFO("Waiting for Object_RVIZ_Mapping_Server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending detected object ID...");
  // send a goal to the action
  simple_navigation_goals::ObjectRVIZMappingGoal goal;
  goal.id = object_ID;
  goal.pos_x = pos_x;
  goal.pos_y = pos_y;
  goal.pos_z = pos_z;

  //ROS_INFO("Sending goals %f, %f, %f...", pos_x, pos_y, pos_z);

  ac.sendGoal(goal);

  ROS_INFO("Object ID sent, skipping...");
  
  return 0;
}

