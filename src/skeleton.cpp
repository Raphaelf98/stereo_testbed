#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include<opencv2/core/operations.hpp>
#include<opencv2/ximgproc.hpp>

#include<math.h>
#include <vector>
#include <string>
#include<geometry_msgs/Point.h>
#include<package1/vectorOfPoints.h>
#include<package1/cameraTransformation.h>
#include<sensor_msgs/PointCloud2.h>




cv::Mat gray;
cv::Mat blur;
int threshold_value = 0;
int const max_value = 255;
const int ratio = 3;
const int kernel_size = 3;
int const max_BINARY_value = 255;
const std::string window = "skeleton";
char* trackbar_value = "Value";


struct Point
{
  int x;
  int y;
  Point(){}
  Point(int x , int y):x(x),y(y)
  {

  }

};


void binarize(int, void*)
{
 cv::threshold(gray,gray,threshold_value, max_BINARY_value, 1);
}

//method for converting std::vector<cv::Point> type to ROS message. Returns ROS message type
std::vector<Point> convertToROSMsg(cv::Mat frame1)
{
  std::vector<cv::Point>  coordinates;
  cv::findNonZero(frame1,coordinates);

  Point my_array[coordinates.size()];
  for (int t=0; t < coordinates.size(); t++)
  {
    int x = coordinates[t].x;
    int y = coordinates[t].y;
    Point pt(x,y);
    my_array[t] = pt;

  }
  std::vector<Point> my_vector (my_array, my_array + sizeof(my_array) / sizeof(Point));
  return my_vector;
 }

class SubscribeAndPublish
{

private:

  package1::vectorOfPoints points1;


  std::vector<geometry_msgs::Point> geomvec;
  geometry_msgs::Point PointXYZ;
  std::vector<Point> SkeletonVect;
  std::vector<geometry_msgs::Point> SpatialSkeleton;

  cv::Mat RotationMatrix;
  cv::Mat InvertedRotation;
  std::vector<float> rot = {0,0,0};
  cv::Vec3f translation;
  bool transformSet = false;
  geometry_msgs::Point circle;
  geometry_msgs::Point circle3D;
public:


  void extrinsicParameterCallback(const package1::cameraTransformation::ConstPtr& parameter_msg)
  {
    if (transformSet)return;
    translation[0]= parameter_msg->tvec.x;
    translation[1]= parameter_msg->tvec.y;
    translation[2]= parameter_msg->tvec.z;

    rot[0] = parameter_msg->rvec.x;
    rot[1] = parameter_msg->rvec.y;
    rot[2] = parameter_msg->rvec.z;
    transformSet=true;
    std::cout<<"received extrinsic parameters"<<std::endl;
  }

  void rigidBodyTransform(geometry_msgs::Point inputPoint , geometry_msgs::Point &transformedPoint )
  {
    cv::Vec3f input;
    if (std::isnan(inputPoint.x))
    {
      std::cout<<"no information" << std::endl;
      inputPoint.x =1000;
      inputPoint.y =1000;
      inputPoint.z =1000;
    }
    input[0]= inputPoint.x;
    input[1]= inputPoint.y;
    input[2]= inputPoint.z;

    cv::Vec3f subs;
    cv::Vec3f add;

    cv::Rodrigues(rot,RotationMatrix);
    cv::transpose(RotationMatrix,InvertedRotation);

    cv::subtract(input, translation,subs);
    cv::Mat output = InvertedRotation*cv::Mat(subs);


    transformedPoint.x = output.at<float>(0);
    transformedPoint.y = output.at<float>(1);
    transformedPoint.z = output.at<float>(2);

  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat normal, frame2;

    try
    {
      cv::normalize(cv_bridge::toCvShare(msg, "bgr8")->image , normal,0,255,cv::NORM_MINMAX,-1,cv::Mat());

      cv::cvtColor(normal, gray , cv::COLOR_RGB2GRAY);
      cv::imshow("gray", gray);

      binarize(0,0);

      cv::ximgproc::thinning(gray,frame2,cv::ximgproc::THINNING_ZHANGSUEN);

      cv::imshow("skeleton", frame2);

      cv::waitKey(1);

      SkeletonVect = convertToROSMsg(frame2);


    }

    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

  }
  void point2CloudCallback(const sensor_msgs::PointCloud2Ptr& pCloud_msg)
  {
    if (SkeletonVect.size() < 3000)
    {
    for (int iter = 0; iter < SkeletonVect.size(); iter++)
    {
      geometry_msgs::Point buff;
      int u = SkeletonVect[iter].x;
      int v = SkeletonVect[iter].y;

      pixelTo3DPoint(*pCloud_msg, u, v, buff);

      if( -10 < buff.x && buff.x <10 )
      {
      SpatialSkeleton.push_back(buff);
      }

    }
     points1.points = SpatialSkeleton;
    SpatialSkeleton.clear();
    }
  }

  void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const double u, const double v, geometry_msgs::Point &p)
  {
    // get width and height of 2D point cloud data
          int width = pCloud.width;
          int height = pCloud.height;

          // Convert from u (column / width), v (row/height) to position in array
          // where X,Y,Z data starts
          int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

          // compute position in array where x,y,z data start
          int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
          int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
          int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

          float X=0.0;
          float Y=0.0;
          float Z=0.0;

          memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
          memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
          memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));


          geometry_msgs::Point input_;

          input_.x = X;
          input_.y = Y;
          input_.z = Z;
          rigidBodyTransform(input_,p);

}




  void POIPublisher(ros::Publisher* pub)
  {


    pub->publish(points1);

  }


};


int main(int argc, char **argv)
{
  bool extrinsicCalibration = false;
  ros::init(argc, argv, "skeleton");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<package1::vectorOfPoints>("SkeletonPoints", 1);

  ros::Rate loop_rate(10);
  SubscribeAndPublish listener;

  cv::namedWindow(window);

  cv::createTrackbar( trackbar_value, window, &threshold_value, max_value, binarize);

  cv::startWindowThread();

  image_transport::ImageTransport it(nh);

  ros::Subscriber extrinsicParameter = nh.subscribe("/CameraTransform", 1, &SubscribeAndPublish::extrinsicParameterCallback, &listener);

  image_transport::Subscriber sub = it.subscribe("/stereo/left/image_rect", 10, &SubscribeAndPublish::imageCallback, &listener);
  ros::Subscriber sub2 = nh.subscribe("stereo/points2",10,&SubscribeAndPublish::point2CloudCallback, &listener);




   while (ros::ok())
     {


      listener.POIPublisher(&pub);

      ros::spinOnce();

   }


  cv::destroyWindow(window);

}
