#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
/*blue
#define H_MAX 117
#define H_MIN 105
#define V_MAX 240
#define V_MIN 0
*/
#define H_MAX 33
#define H_MIN 29
#define V_MAX 220
#define V_MIN 120
Mat b_output;
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
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    //namedWindow(OPENCV_WINDOW);
  }
  ~ImageConverter()
  {
    //destroyWindow(OPENCV_WINDOW);
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
      cvtColor(cv_ptr->image,b_output,CV_BGR2HSV,3);
      Scalar s_min = Scalar(H_MIN, 10, V_MIN);
      Scalar s_max = Scalar(H_MAX, 235, V_MAX);
      inRange(b_output, s_min, s_max,b_output);
      erode(b_output, b_output, noArray(), Point(-1, -1), 5);
      dilate(b_output,b_output, noArray(), Point(-1, -1), 5);
      dilate(b_output,b_output, noArray(), Point(-1, -1), 10);
    }
    imshow("OutputImage", b_output);
    printf("yellow\n");
    waitKey(3);
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
