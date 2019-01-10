#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

void imageCallback(const sensor_msgs::image::ConstPtr& msg)
{
  frame_pointer = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  imshow("Test",frame_pointer->image);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imageprocessing");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/camera/image", 1000, imageCallback);

  ros::spin();

  return 0;
}