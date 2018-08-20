#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "simple_navigation_goals/getWorldPose.h"

class echoListener
{
public:

  tf::TransformListener tf;
  tf::StampedTransform echo_transform;

  bool getWorldPose(simple_navigation_goals::getWorldPose::Request  &req,
                    simple_navigation_goals::getWorldPose::Response &res);

  //constructor with name
  echoListener()
  {

  };

  ~echoListener()
  {

  };

private:

};

bool echoListener::getWorldPose(simple_navigation_goals::getWorldPose::Request  &req,
                    simple_navigation_goals::getWorldPose::Response &res)
  {
    tf::Vector3 point(req.cam_x, req.cam_y, req.cam_z);
    tf::Vector3 point_bl = echo_transform * point;
    res.world_x = point_bl[0];
    res.world_y = point_bl[1];
    res.world_z = point_bl[2];
    ROS_INFO("Camera 3D point converted to world frame!");
    return true;
  }

int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "tf_echo_map_camera", ros::init_options::AnonymousName);

  /*if (argc != 1)
  {
    printf("Usage: tf_echo source_frame target_frame\n\n");
    printf("This will echo the transform from the coordinate frame of the source_frame\n");
    printf("to the coordinate frame of the target_frame. \n");
    printf("Note: This is the transform to get data from target_frame into the source_frame.\n");
    return -1;
  }*/

  ros::NodeHandle nh;
  //Instantiate a local listener
  echoListener echoListener;
  //ros::Publisher pub_transform = nh.advertise<geometry_msgs::Pose>("map_camera_transform",1);
  //Service
  ros::ServiceServer pose_service = nh.advertiseService("get_world_pose", &echoListener::getWorldPose, &echoListener);

  std::string source_frameid = "/map";
  std::string target_frameid = "/camera_depth_optical_frame";

  // Wait for up to one second for the first transforms to become avaiable. 
  echoListener.tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));

  //Nothing needs to be done except wait for a quit
  //The callbacks withing the listener class
  //will take care of everything
  while(nh.ok())
    {
      try
      {
        //tf::StampedTransform echo_transform;
        echoListener.tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echoListener.echo_transform);
        std::cout.precision(3);
        std::cout.setf(std::ios::fixed,std::ios::floatfield);
        //std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl;
        double yaw, pitch, roll;
        echoListener.echo_transform.getBasis().getRPY(roll, pitch, yaw);
        tf::Quaternion q = echoListener.echo_transform.getRotation();
        tf::Vector3 v = echoListener.echo_transform.getOrigin();
        //std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
        //std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " 
        //          << q.getZ() << ", " << q.getW() << "]" << std::endl
        //          << "            in RPY [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl;

        /*geometry_msgs::Pose map_cam_pose;
        map_cam_pose.position.x=v.getX();
   	map_cam_pose.position.y=v.getY();	
	map_cam_pose.position.z=v.getZ();
        map_cam_pose.orientation.x=q.getX();
	map_cam_pose.orientation.y=q.getY();
	map_cam_pose.orientation.z=q.getZ();
	map_cam_pose.orientation.w=q.getW();

   	pub_transform.publish(map_cam_pose);
        */
        //tf::Vector3 point(-0.019, -0.048, 2.157);
        //tf::Vector3 point_bl = echo_transform * point;
        //ROS_INFO("point %f, %f, %f",point_bl[0], point_bl[1], point_bl[2]);

        //print transform
      }
      catch(tf::TransformException& ex)
      {
        std::cout << "Failure at "<< ros::Time::now() << std::endl;
        std::cout << "Exception thrown:" << ex.what()<< std::endl;
        std::cout << "The current list of frames is:" <<std::endl;
        std::cout << echoListener.tf.allFramesAsString()<<std::endl;
        
      }
      sleep(1);
      ros::spinOnce();
    }

  return 0;
};

