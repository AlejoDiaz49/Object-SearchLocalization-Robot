#include <image_rectification/ImageRectificationClass.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "image_rectification");
   ros::NodeHandle nodeHandle("~");
   image_rectification::DepthImageRectification DepthImageRectification(nodeHandle);

   ros::spin();
   return 0;
}
