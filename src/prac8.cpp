#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point.h>
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
    ros::Publisher odepoint;
    geometry_msgs::Point vel;    
    public:
    ImageConverter()
    : it_(nh_){
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,&ImageConverter::imageCb, this);
        odepoint = nh_.advertise<geometry_msgs::Point>("/ode_pos", 10);
        //namedWindow(OPENCV_WINDOW);
    }
    ~ImageConverter(){
        //destroyWindow(OPENCV_WINDOW);
    }
    void imageCb(const sensor_msgs::ImageConstPtr &msg){
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60){
            //とりあえず出力テスト
            vel.x=0;
            vel.y=0;
            vel.z=0;
            odepoint.publish(vel);
            waitKey(3);
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
