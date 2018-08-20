#pragma once

   // c++
   #include <math.h>
   #include <string>
   #include <vector>
   #include <iostream>
//   #include <pthread.h>
//   #include <thread>
//   #include <chrono>
   #include <stdio.h>
   #include <stdint.h>
   #include <inttypes.h>

   // ROS
   #include <ros/ros.h>
   #include <std_msgs/Header.h>
   #include <std_msgs/Int8.h>
   #include <std_msgs/String.h>
   #include <actionlib/server/simple_action_server.h>
   #include <sensor_msgs/image_encodings.h>
   #include <sensor_msgs/Image.h>
   #include <sensor_msgs/CameraInfo.h>
   #include <geometry_msgs/Point.h>
   #include <image_transport/image_transport.h>
   #include <message_filters/subscriber.h>
   #include <message_filters/synchronizer.h>
   #include <message_filters/time_synchronizer.h>
   #include <message_filters/sync_policies/approximate_time.h>
   #include <image_transport/subscriber_filter.h>

   // OpenCv
   #include <opencv2/imgproc/imgproc.hpp>
   #include <opencv2/highgui/highgui.hpp>
   #include <opencv2/objdetect/objdetect.hpp>
   #include <cv_bridge/cv_bridge.h>
   #include <opencv2/core/utility.hpp>

namespace image_rectification
{
   class DepthImageRectification
   {
      public:
      
      // Constructor.
      explicit DepthImageRectification(ros::NodeHandle nh);

      // Destructor.
      ~DepthImageRectification();

      private:
  
      // Initialize the ROS connections.
      void init();

      // Callback of camera
      void cameraCallback(const sensor_msgs::ImageConstPtr& rgbmsg, 
                          const sensor_msgs::ImageConstPtr& depthmsg,
                          const sensor_msgs::CameraInfoConstPtr& rgbinfo, 
                          const sensor_msgs::CameraInfoConstPtr& depthinfo);

      // Publishes the detection image - @return true if successful.
      bool publishRectifiedImage(const cv::Mat& rectifiedImage);

      // Publishes the rgb image - @return true if successful.
      //bool publishRgbImage(const sensor_msgs::ImageConstPtr& rgbImage);

      // Publishes camera info - @return true if successful.
      bool publishCameraInfo(const sensor_msgs::CameraInfoConstPtr& caminfo);

      // ROS node handle.
      ros::NodeHandle nodeHandle_;

      // Advertise and subscribe to image topics.
      image_transport::ImageTransport imageTransport_;

      // ROS subscriber and publisher.
      //image_transport::Subscriber imageSubscriber_;

      // Syncronizing Image messages - For depth inclussion
      typedef image_transport::SubscriberFilter ImageSubscriberFilter;
      ImageSubscriberFilter imagergb_sub;
      ImageSubscriberFilter imagedepth_sub;
      
      message_filters::Subscriber<sensor_msgs::CameraInfo> depthinfo_sub, rgbinfo_sub;
      //ImageSubscriberFilter camerainfo_sub; //ADDED
      
      //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy_1;
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo,sensor_msgs::CameraInfo> MySyncPolicy_1;
      message_filters::Synchronizer<MySyncPolicy_1> sync_1;

      // Depth Image
      cv::Mat DepthImageCopy_;
      cv::Mat RGBImageCopy_;
      cv::Mat Binaria;
		int dilation_size=5;
		cv::Mat Element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2*dilation_size+1,2*dilation_size+1), cv::Point(dilation_size,dilation_size));

      // Publisher of the bounding box image.
      ros::Publisher depthRecImagePublisher_;
      //ros::Publisher rgbImagePublisher_;
      ros::Publisher cameraInfoPublisher_;
      sensor_msgs::CameraInfo CamInfo;
   };
}
