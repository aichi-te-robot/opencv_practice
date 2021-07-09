#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;

/*black*/
#define kuH_MAX 180
#define kuH_MIN 0
#define kuS_MAX 255
#define kuS_MIN 0
#define kuV_MAX 100
#define kuV_MIN 0

Mat ku_output;
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
  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    {
      //black
      cvtColor(cv_ptr->image, ku_output, CV_BGR2HSV, 3);
      Scalar kus_min = Scalar(kuH_MIN, kuS_MIN, kuV_MIN);
      Scalar kus_max = Scalar(kuH_MAX, kuS_MAX, kuV_MAX);
      inRange(ku_output, kus_min, kus_max, ku_output);
      imshow("0KOutputImage", ku_output);
      
      printf("%d\n",countNonZero(ku_output));

      int flag = 0;
      if (countNonZero(ku_output)>36300&&countNonZero(ku_output)<38250)
      {
        flag = 1;
      }
      else if (countNonZero(ku_output)>38250&&countNonZero(ku_output)<39800)
      {
        flag = 2;
      }
      else if (countNonZero(ku_output)>33900&&countNonZero(ku_output)<36300)
      {
        flag = 3;
      }
      else if (countNonZero(ku_output)>31900&&countNonZero(ku_output)<33900)
      {
        flag = 4;
      }

      switch (flag)
      {
      case 1:
        printf("Aaaa!\n");
        break;
      case 2:
        printf("Bbbb!\n");
        break;
      case 3:
        printf("Cccc!\n");
        break;
      case 4:
        printf("Tttt!\n");
        break;
      default:
        printf("not\n");
        break;
      }
      waitKey(3);
      image_pub_.publish(cv_ptr->toImageMsg());
    }
  }
};
  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
  }
