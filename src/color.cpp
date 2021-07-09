#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
/*yellowcard*/
#define yH_MAX 33
#define yH_MIN 29
#define yV_MAX 220
#define yV_MIN 120

/*redcard*/
#define rH_MAX 179
#define rH_MIN 176
#define rV_MAX 215
#define rV_MIN 204

/*orangecard*/
#define oH_MAX 5
#define oH_MIN 2
#define oV_MAX 252
#define oV_MIN 229

Mat y_output,r_output,o_output;
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
      Scalar ys_min = Scalar(yH_MIN, 10, yV_MIN);
      Scalar ys_max = Scalar(yH_MAX, 235, yV_MAX);
      inRange(y_output, ys_min, ys_max,y_output);
      erode(y_output, y_output, noArray(), Point(-1, -1), 5);
      dilate(y_output,y_output, noArray(), Point(-1, -1), 5);
      dilate(y_output,y_output, noArray(), Point(-1, -1), 10);
	//red
      cvtColor(cv_ptr->image,r_output,CV_BGR2HSV,3);
      Scalar rs_min = Scalar(rH_MIN, 10, rV_MIN);
      Scalar rs_max = Scalar(rH_MAX, 235, rV_MAX);
      inRange(r_output, rs_min, rs_max,r_output);
      erode(r_output, r_output, noArray(), Point(-1, -1), 5);
      dilate(r_output,r_output, noArray(), Point(-1, -1), 5);
      dilate(r_output,r_output, noArray(), Point(-1, -1), 10);
	//orange
      cvtColor(cv_ptr->image,o_output,CV_BGR2HSV,3);
      Scalar os_min = Scalar(oH_MIN, 10, oV_MIN);
      Scalar os_max = Scalar(oH_MAX, 235, oV_MAX);
      inRange(o_output, os_min, os_max,o_output);
      erode(o_output, o_output, noArray(), Point(-1, -1), 5);
      dilate(o_output,o_output, noArray(), Point(-1, -1), 5);
      dilate(o_output,o_output, noArray(), Point(-1, -1), 10);
    }
    imshow("YOutputImage", y_output);
    imshow("ROutputImage", r_output);
    imshow("OOutputImage", o_output);

    Mat output;
    int flag=0;
    if(countNonZero(y_output)>countNonZero(r_output)&&countNonZero(y_output)>countNonZero(o_output)){
	output=y_output;
	flag=1;
    }
    if(countNonZero(r_output)>countNonZero(y_output)&&countNonZero(r_output)>countNonZero(o_output)){
	output=r_output;
	flag=2;
    }
    if(countNonZero(o_output)>countNonZero(y_output)&&countNonZero(o_output)>countNonZero(r_output)){
	output=o_output;
	flag=3;
    }

    switch(flag){
	case 1:
		printf("yellow\n");
		break;
	case 2:
		printf("red\n");
		break;
	case 3:
		printf("orange\n");
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
