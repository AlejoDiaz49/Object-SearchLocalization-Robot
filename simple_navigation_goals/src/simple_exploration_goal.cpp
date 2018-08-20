#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <test_client/ExploreTaskAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "frontier_exploration_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<test_client::ExploreTaskAction> ac("explore_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  test_client::ExploreTaskGoal goal;
  goal.explore_boundary.header.seq = 1;
  goal.explore_boundary.header.frame_id = "map";
  //goal.explore_center.point.x = 1;
  //goal.explore_center.point.y = 0;
  //goal.explore_center.point.z = 0;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));


  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
