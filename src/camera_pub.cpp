#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv)
{
  // Check if video source has been passed as a parameter

  ros::init(argc, argv, "camera_pub");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);

  // Convert the passed as command line parameter index for the video device to an integer

  // Check if it is indeed a number


 // cv::VideoCapture cap(video_source);
   cv::VideoCapture cap(0);
   //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 400);  cap.set(cv::CAP_PROP_FRAME_WIDTH, 680);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) {std::cout<< "cannot open video stream!"<< std::endl;return 1;}
  cv::Mat frame;

  sensor_msgs::ImagePtr msg;
  std::cout<<"fps: " << cap.get(cv::CAP_PROP_FPS)<< std::endl;
  ros::Rate loop_rate(30);
  while (nh.ok()) {
    cap >> frame;
    //cv::resize(frame,frame,cv::Size(1000,1000),0,0,cv::INTER_CUBIC);
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
