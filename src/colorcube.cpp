#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
/*yellowcube*/
#define yH_MAX 27
#define yH_MIN 23
#define yS_MAX 221
#define yS_MIN 79
#define yV_MAX 252
#define yV_MIN 204

/*redcube*/
#define rH_MAX 5
#define rH_MIN 2
#define rS_MAX 239
#define rS_MIN 153
#define rV_MAX 225
#define rV_MIN 204

/*bluecube*/
#define bH_MAX 100
#define bH_MIN 95
#define bS_MAX 188
#define bS_MIN 74
#define bV_MAX 255
#define bV_MIN 155

Mat y_output,r_output,b_output;
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
	//yellow
      cvtColor(cv_ptr->image,y_output,CV_BGR2HSV,3);
      Scalar ys_min = Scalar(yH_MIN, yS_MIN, yV_MIN);
      Scalar ys_max = Scalar(yH_MAX, yS_MAX, yV_MAX);
      inRange(y_output, ys_min, ys_max,y_output);
      erode(y_output, y_output, noArray(), Point(-1, -1), 2);
      dilate(y_output,y_output, noArray(), Point(-1, -1), 3);
      dilate(y_output,y_output, noArray(), Point(-1, -1), 5);
	//red
      cvtColor(cv_ptr->image,r_output,CV_BGR2HSV,3);
      Scalar rs_min = Scalar(rH_MIN, rS_MIN, rV_MIN);
      Scalar rs_max = Scalar(rH_MAX, rS_MAX, rV_MAX);
      inRange(r_output, rs_min, rs_max,r_output);
      erode(r_output, r_output, noArray(), Point(-1, -1), 2);
      dilate(r_output,r_output, noArray(), Point(-1, -1), 3);
      dilate(r_output,r_output, noArray(), Point(-1, -1), 5);
	//blue
      cvtColor(cv_ptr->image,b_output,CV_BGR2HSV,3);
      Scalar bs_min = Scalar(bH_MIN, bS_MIN, bV_MIN);
      Scalar bs_max = Scalar(bH_MAX, bS_MAX, bV_MAX);
      inRange(b_output, bs_min, bs_max,b_output);
      erode(b_output, b_output, noArray(), Point(-1, -1), 2);
      dilate(b_output,b_output, noArray(), Point(-1, -1), 3);
      dilate(b_output,b_output, noArray(), Point(-1, -1), 5);
    }
    imshow("YOutputImage", y_output);
    imshow("ROutputImage", r_output);
    imshow("BOutputImage", b_output);

    int flag=0;
    if(countNonZero(y_output)>countNonZero(r_output)&&countNonZero(y_output)>countNonZero(b_output)){
	flag=1;
    }
  else if(countNonZero(r_output)>countNonZero(y_output)&&countNonZero(r_output)>countNonZero(b_output)){
	flag=2;
    }
    else if(countNonZero(b_output)>countNonZero(y_output)&&countNonZero(b_output)>countNonZero(r_output)){
	flag=3;
    }

    switch(flag){
	case 1:
		printf("yellow_cube\n");
		break;
	case 2:
		printf("red_cube\n");
		break;
	case 3:
		printf("blue_cube\n");
		break;
	default:
		printf("not color\n");
		break;
    }
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
