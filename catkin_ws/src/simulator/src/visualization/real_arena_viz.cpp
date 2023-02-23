#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

bool show_image;
bool dispayed = false;
bool last_status = false;

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
  ros::param::get("/show_image", show_image);
  
  
  if(show_image) {
    try
    {
      //std::cout << "Display image.->" << dispayed << std::endl;
      cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);

      if(cv::getWindowProperty("real_time_view", cv::WND_PROP_AUTOSIZE) > 0) dispayed = true;
      else dispayed = false;

      if(!dispayed && last_status) ros::param::set("/show_image", false);

      cv::imshow("real_time_view", image);
      cv::waitKey(1);
      last_status = dispayed;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert to image!");
    }
  }
  else {
    cv::destroyAllWindows();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "real_arena_viz");
  ros::NodeHandle nh;
  std::cout << "Starting real_arena_viz by Luis Nava..." << std::endl;
  nh.setParam("/show_image", show_image);
  image_transport::ImageTransport it(nh);
  ros::Subscriber sub = nh.subscribe("/camera/image/compressed", 1, imageCallback);
  ros::spin();
}   