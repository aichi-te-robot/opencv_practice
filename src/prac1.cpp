#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#define H_MAX 117
#define H_MIN 105
#define V_MAX 240
#define V_MIN 0
cv::Mat raw;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
public:
  ImageConverter()
    : it_(nh_)
  {
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
  }
  ~ImageConverter()
  {
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60){
      cv::Mat hsv_image,mask_image;
      raw=cv_ptr->image;
      cv::circle(cv_ptr->image, cv::Point(50, 50), 100, CV_RGB(255,0,0));
      cv::cvtColor(cv_ptr->image,hsv_image,CV_BGR2HSV,3);
     cv::Scalar s_min = cv::Scalar(H_MIN, 0, V_MIN);
     cv::Scalar s_max = cv::Scalar(H_MAX, 255, V_MAX);
      inRange(hsv_image, s_min, s_max,cv_ptr->image);
    }
    cv::imshow("RawImage", raw);
    cv::imshow("HsvImage", cv_ptr->image);
    cv::waitKey(3);
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
