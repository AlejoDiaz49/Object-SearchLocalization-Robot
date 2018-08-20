#include "image_rectification/ImageRectificationClass.hpp"

namespace image_rectification 
{
   DepthImageRectification::DepthImageRectification(ros::NodeHandle nh)
       : nodeHandle_(nh),
         imageTransport_(nodeHandle_),
         imagergb_sub(imageTransport_,"/camera/rgb/image_raw",1),
         imagedepth_sub(imageTransport_,"/camera/depth/image_raw",1),
         rgbinfo_sub(nodeHandle_,"/camera/rgb/camera_info",1),
         depthinfo_sub(nodeHandle_,"/camera/depth/camera_info",1),
         sync_1(MySyncPolicy_1(5), imagergb_sub, imagedepth_sub, rgbinfo_sub, depthinfo_sub) 

   {
      ROS_INFO("[DepthImageRectification] Node started.");
      init();
   }
   
   DepthImageRectification::~DepthImageRectification(){}

   void DepthImageRectification::init()
   {
      ROS_INFO("[DepthImageRectification] init().");

      std::string cameraTopicName;
      int cameraQueueSize;
      std::string depthTopicName;  
      int depthQueueSize;          

      std::string depthRecImageTopicName;
      int depthRecImageQueueSize;
      bool depthRecImageLatch;

      nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName, std::string("/camera/image_raw"));
      nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);

      nodeHandle_.param("subscribers/camera_depth/topic", depthTopicName, std::string("/depth/image_raw"));
      nodeHandle_.param("subscribers/camera_depth/queue_size", depthQueueSize, 1);

      nodeHandle_.param("publishers/depth_image/topic", depthRecImageTopicName, std::string("depth_image"));
      nodeHandle_.param("publishers/depth_image/queue_size", depthRecImageQueueSize, 1);
      nodeHandle_.param("publishers/depth_image/latch", depthRecImageLatch, true);

      depthRecImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(depthRecImageTopicName, depthRecImageQueueSize, depthRecImageLatch);

      sync_1.registerCallback(boost::bind(&DepthImageRectification::cameraCallback,this,_1,_2,_3,_4));  

      cameraInfoPublisher_ = nodeHandle_.advertise<sensor_msgs::CameraInfo>("/image_rectification/camera_info",1);

      //rgbImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>("/image_rectification/rgb_image",1);
   }

   void DepthImageRectification::cameraCallback(const sensor_msgs::ImageConstPtr& rgbmsg, const sensor_msgs::ImageConstPtr& depthmsg, const sensor_msgs::CameraInfoConstPtr& rgbinfo, const sensor_msgs::CameraInfoConstPtr& depthinfo)
   {

      ROS_DEBUG("[DepthImageRectification] Image received.");
      cv_bridge::CvImagePtr cam_rgb;
      cv_bridge::CvImageConstPtr cam_depth;

      try
      {
         cam_rgb = cv_bridge::toCvCopy(rgbmsg, sensor_msgs::image_encodings::BGR8);
         cam_depth = cv_bridge::toCvCopy(depthmsg, sensor_msgs::image_encodings::TYPE_32FC1);
      }

      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("CvBridge exception: %s", e.what());
         return;
      }

      if (cam_rgb && cam_depth)
      {
         //RGBImageCopy_ = cam_rgb->image.clone();
         DepthImageCopy_ = cam_depth->image.clone();

         cv::cvtColor(cam_rgb->image.clone(),RGBImageCopy_, cv::COLOR_BGR2HSV);

         cv::Scalar AmarillosBajos(20,100,100);
         cv::Scalar AmarillosAltos(30,255,255);
   
         cv::inRange(RGBImageCopy_, AmarillosBajos, AmarillosAltos, Binaria);
			cv::dilate(Binaria,Binaria,Element);

         for(int i=0; i<=(RGBImageCopy_.cols-1); i++)
            for(int j=0; j<=(RGBImageCopy_.rows-1); j++)
                  if (((int)Binaria.at<uchar>(j,i))!=0)
							DepthImageCopy_.at<float>(j,i)=nanf(" ");

         if (!publishRectifiedImage(cv::Mat(DepthImageCopy_)))
            ROS_DEBUG("Detection image has not been broadcasted.");

         publishCameraInfo(rgbinfo);
         //publishRgbImage(rgbmsg);
            
      }
      return;
   }

   bool DepthImageRectification::publishRectifiedImage(const cv::Mat& rectifiedImage)
   {
      if (depthRecImagePublisher_.getNumSubscribers() < 1)
         return false;

      cv_bridge::CvImage cvImage;
      cvImage.header.stamp = ros::Time::now();
      cvImage.header.frame_id = "rectified_depth_image";
      cvImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      cvImage.image = rectifiedImage;
      depthRecImagePublisher_.publish(*cvImage.toImageMsg());


      ROS_DEBUG("Rectified depth image has been published.");
      return true;
   }
   
   bool DepthImageRectification::publishCameraInfo(const sensor_msgs::CameraInfoConstPtr& caminfo)
   {
      CamInfo = sensor_msgs::CameraInfo(*caminfo);
      CamInfo.header.stamp = ros::Time::now();
      cameraInfoPublisher_.publish(CamInfo);
      return true;
   }
   
   /*bool DepthImageRectification::publishRgbImage(const sensor_msgs::ImageConstPtr& rgbImage)
   {
      sensor_msgs::Image RGBImage = sensor_msgs::Image(*rgbImage);
      RGBImage.header.stamp = ros::Time::now();
      cameraInfoPublisher_.publish(RGBImage);
      return true;
   }*/
}
