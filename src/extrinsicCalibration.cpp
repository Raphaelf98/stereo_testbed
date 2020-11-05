#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include<package1/cameraTransformation.h>
#include<math.h>
#include <vector>
#include <std_msgs/Float64.h>
#include<boost/array.hpp>

cv::Size boardSize(7,6);

float squareEdgeLength = 0.015;

class SubscribeAndPublish
{
private:
   package1::cameraTransformation CameraTransform_msg;


   std::vector<float> rotation= {0,0,0};
   std::vector<float> translation= {0,0,0};
   boost::array<double,9> CalibrationMatrix;
   std::vector<double> DistortionMatrix;
   bool corners_found = false;
public:

   //std::vector<cv::Point2f> findCorners(cv::Mat img)
   void findCorners(cv::Mat img,std::vector<cv::Point2f>& cornersFound)
   {

      corners_found = findChessboardCorners(img, boardSize, cornersFound, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
       if (corners_found)
       {
         cv::drawChessboardCorners(img, boardSize, cornersFound, corners_found);

          cv::circle(img, cornersFound[0] , 8, cv::Scalar( 255, 255, 255 ),-2,8,0);
          imshow("detected corners", img);

       }
       else
       {
         std::cout<<"ERROR couldn't detect corners! Please move Chessboard into field of view"<< std::endl;
       }
   }

   //populate
   void realWorld3DChessboardPosition( cv::Size boardSize, float squareEdgeLength_, std::vector<cv::Point3f>& corners_1)
   {
     for (int i = 0; i < boardSize.height; i++)
         {
             for (int j = 0; j < boardSize.width; j++)
             {
                 corners_1.push_back(cv::Point3f(j * squareEdgeLength_, i * squareEdgeLength_, 0.0f));

             }

         }



   }


  void imageCallback(const sensor_msgs::ImageConstPtr& image_, const sensor_msgs::CameraInfoConstPtr& camera_info)
  {
    cv::Mat frame = cv_bridge::toCvShare(image_, "bgr8")->image;
    std::vector<cv::Point3f> worldSpaceCornerPoints;
    std::vector<cv::Point2f> chessboardImageSpacePoints;
    std::vector<std::vector<double>> CameraMatrix = {{0,0,0},{0,0,0},{0,0,0}};
    cv::Mat_<double> camMatrix(3,3);
    try
    {
      cv::imshow("view", frame);

      CalibrationMatrix = camera_info->K;
      DistortionMatrix = camera_info->D;

      int k = 0;
      for (int i=0 ; i<3 ;  i++)
      {
        for (int c=0 ; c<3 ;  c++){

        camMatrix(i,c) = CalibrationMatrix[k];
        k++;
      }

      }

      findCorners(frame,chessboardImageSpacePoints);
      realWorld3DChessboardPosition(boardSize,squareEdgeLength, worldSpaceCornerPoints);

      if (corners_found)
      {
      cv::solvePnP(worldSpaceCornerPoints, chessboardImageSpacePoints,camMatrix,DistortionMatrix, rotation,translation );
      std::cout<< "rotation: ";
      CameraTransform_msg.rvec.x=  rotation[0];
       CameraTransform_msg.rvec.y=  rotation[1];
        CameraTransform_msg.rvec.z=  rotation[2];
                  for (int j=0; j<3 ; j++)
      {

      std::cout << rotation[j];
      }

       CameraTransform_msg.tvec.x=  translation[0];//-0.015;
        CameraTransform_msg.tvec.y=  translation[1];
         CameraTransform_msg.tvec.z=  translation[2];
      std::cout<<  "    translation: ";
                   for (int k=0 ; k<3 ; k++)
      {

      std::cout << translation[k] ;
      }
      }

      cv::waitKey(10);
      }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_->encoding.c_str());
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void CameraTransformationPublisher(ros::Publisher* pub)
  {
    pub->publish(CameraTransform_msg);

  }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "extrinsicCalibration");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  SubscribeAndPublish listener;
  ros::Publisher pub = nh.advertise<package1::cameraTransformation>("CameraTransform", 1);


  image_transport::CameraSubscriber camerasub = it.subscribeCamera("/stereo/left/image_mono", 10, &SubscribeAndPublish::imageCallback, &listener);

  //ros::spin();
  while(ros::ok())
  {
    listener.CameraTransformationPublisher(&pub);
    ros::spinOnce();
  }
  cv::destroyWindow("view");
}
