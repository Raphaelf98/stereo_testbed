#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include<math.h>
#include <vector>

#include<geometry_msgs/Point.h>
#include<visualization_msgs/Marker.h>
#include<package1/vectorOfPoints.h>
#include<rviz_visual_tools/rviz_visual_tools.h>


#include<libalglib/linalg.h>

#include<libalglib/interpolation.h>
#include<libalglib/ap.h>

cv::Mat gray;
//cv::Mat *m;
cv::Mat normal, frame2;
cv::Mat blur;
cv::Mat img;
cv::Mat canny;
cv::Vec4f line;
cv::Point pt1, pt2;
int splineDivider = 5;

std::array<int,2> arr;
float d, t;
int threshold_value = 0;
int threshold_type = 3;;
int const max_value = 255;

const int ratio = 3;
const int kernel_size = 3;

int const max_BINARY_value = 255;
char* window_name= "skeleton";
char* window_nameCH= "convexHull";
char* trackbar_value = "Value";

std::vector<cv::Point> locations;

const int max_thresh = 255;
int thresh = 100;

int curve_thresh_max = 0;
int curve_thresh_min = 0;
int max_curve_thresh =1000;
cv::Point2i  tcp;
int step_min = 0;
int step_max = 100;

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
  //cv::threshold(gray,gray,threshold_value, max_BINARY_value, 1);
   cv::threshold(gray,gray,70, max_BINARY_value, 1);
}




//method for converting std::vector<cv::Point> type to ROS message. Returns ROS message type
std::vector<Point> convertToROSMsg(cv::Mat frame1)

{ std::vector<cv::Point>  coordinates;

    cv::findNonZero(frame1,coordinates);

   Point my_array[coordinates.size()];
  for (int t=0; t < coordinates.size(); t++)
  {
  int x = coordinates[t].x;
  int y = coordinates[t].y;
  Point pt(x,y);
  my_array[t] = pt;
  //std::cout << "x_0 coordinate: " << pt.x << std::endl;
  }
  std::vector<Point> my_vector (my_array, my_array + sizeof(my_array) / sizeof(Point));
  return my_vector;
 }




void LinearEstimator(std::vector<cv::Point> points)
  {
  cv::fitLine(points, line, cv::DIST_L1, 1,0.001,0.001);
  //draw Points
 cv::Mat img(normal.size(), CV_8UC1, cv::Scalar(0));

  for (int i=0; i < points.size(); i++ )
  {
    cv::circle(img,points[i],1,cv::Scalar(100, 100, 255),cv::FILLED,16,cv::LINE_8);


  }
  // ... and the long enough line to cross the whole image
  d = sqrt( (double)line[0]*line[0] + (double)line[1]*line[1] );
  line[0] /= d;
  line[1] /= d;
  t = (float)(img.cols + img.rows);
  pt1.x = cvRound(line[2] - line[0]*t);
  pt1.y = cvRound(line[3] - line[1]*t);
  pt2.x = cvRound(line[2] + line[0]*t);
  pt2.y = cvRound(line[3] + line[1]*t);
  cv::line( img, pt1, pt2, cv::Scalar(255,255,255), 1, 16, 0 );

  cv::imshow( "Approximation", img );
}


//Skeletonization and returns std::vector<cv::Points>
cv::Mat skeletonize(cv::Mat frame)
{
  bool done;
   cv::Mat skel(frame.size(), CV_8UC1, cv::Scalar(0));
  //cv::Mat temp(frame.size(), CV_8UC1);
  cv::Mat temp;
  cv::Mat eroded;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
  cv::Mat element1= cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));
// source: http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/


    //cv::imshow("input ", frame);
  do

  { //cv::imshow("frame", frame);
      //input frame is opened and saved to temp
    cv::erode(frame, eroded, element);
    cv::dilate(eroded, temp, element); // temp = open(img)
    //cv::imshow("opening", temp);

    //substract input frame from temp (opening result)´
    //Def. substract: Calculates the per-element difference between two arrays or array and a scalar
    cv::subtract(frame, temp, temp);
    cv::imshow("subtracted ", temp);
    //cv::imshow("skel ", skel);
    // add the result from substract to every skel from last iteration
    cv::bitwise_or(skel, temp, skel);
    //cv::imshow("or",skel);
    //every new iteration uses the frame eroded
    eroded.copyTo(frame);
    //until no frame exists, then the loop is exited
    done = (cv::countNonZero(frame) == 0);
  } while (!done);





cv::imshow("skeleton", skel);
//cv::erode(skel, skel, element1);
//cv::dilate(skel,skel,element1);



return skel;



}
//https://stackoverflow.com/questions/32629806/how-can-i-calculate-the-curvature-of-an-extracted-contour-by-opencv/32630881
cv::Point2i toolCenterPoint1(std::vector<std::vector<cv::Point>> vecContourPoints){

std::cout <<"number of contours: "<< vecContourPoints.size()<<std::endl;
cv::Point2f posOld, posOlder;
cv::Point2f f1stDerivative, f2ndDerivative;
// the outer for loop iterates over the amount of countours, the inner one iterates over the the individual contour
for( size_t j= 0; j < vecContourPoints.size(); j++ ){
  if (vecContourPoints.size() > 1)
  {
    std::cout<< "     The contour is not closed!"<<std::endl;
  }

  cv::Mat tcp(normal.size(), CV_8UC1, cv::Scalar(0));
  std::vector <float> vecCurvature(vecContourPoints[j].size());
  cv::Point2i TCP;
  std::vector<cv::Point2i> vecCurvaturePosition(vecContourPoints[j].size());
  for (size_t i = 0; i < vecContourPoints[j].size(); i++ )
      {
       const cv::Point2f &pos = vecContourPoints[j][i];

       if ( i == 0 ){ posOld = posOlder = pos; }

       f1stDerivative.x =   pos.x -        posOld.x;
       f1stDerivative.y =   pos.y -        posOld.y;
       //f2ndDerivative.x = - pos.x + 2.0f * posOld.x - posOlder.x;
       //f2ndDerivative.y = - pos.y + 2.0f * posOld.y - posOlder.y;
       f2ndDerivative.x =  pos.x - 2.0f * posOld.x + posOlder.x;
       f2ndDerivative.y =  pos.y - 2.0f * posOld.y + posOlder.y;
       float curvature2D = 0.0f;
       if ( std::abs(f2ndDerivative.x) > 10e-4 && std::abs(f2ndDerivative.y) > 10e-4 )
       {
           curvature2D = sqrt( std::abs(
               pow( f2ndDerivative.y*f1stDerivative.x - f2ndDerivative.x*f1stDerivative.y, 2.0f ) /
               pow( f2ndDerivative.x + f2ndDerivative.y, 3.0 ) ) );


       if (curvature2D < (double)curve_thresh_max/1000 && curvature2D > (double)curve_thresh_min/1000)
       {

       vecCurvature[i] = curvature2D;
       std::cout<< "curvature: "<< vecCurvature[i] << std::endl;
       vecCurvaturePosition[i].x = pos.x;
       vecCurvaturePosition[i].y =pos.y;
       }
       posOlder = posOld;
       posOld = pos;
       }
      }

  int maxElementIndex = std::max_element(vecCurvature.begin(), vecCurvature.end())- vecCurvature.begin();
  //return the amount of curvature vectors found
  std::cout << vecCurvaturePosition[maxElementIndex]<<std::endl;

  TCP =  vecCurvaturePosition[maxElementIndex];


  return TCP;
}

}





cv::Point2i toolCenterPoint2(std::vector<std::vector<cv::Point>> vecContourPoints, int step)
{

std::cout <<"number of contours: "<< vecContourPoints.size()<<std::endl;
cv::Point2f posOld, posOlder;
cv::Point2f f1stDerivative, f2ndDerivative;
// the outer for loop iterates over the amount of countours, the inner one iterates over the the individual contour
for( int j= 0; j < vecContourPoints.size(); j++ )
{
  if (vecContourPoints.size() > 1)
  {
    std::cout<< "     The contour is not closed!"<<std::endl;
  }

  cv::Mat tcp(normal.size(), CV_8UC1, cv::Scalar(0));
  std::vector <float> vecCurvature(vecContourPoints[j].size());
  cv::Point2i TCP;
  std::vector<cv::Point2i> vecCurvaturePosition(vecContourPoints[j].size());

  auto frontToBack = vecContourPoints[j].front() - vecContourPoints[j].back();

  bool isClosed = ((int)std::max(std::abs(frontToBack.x), std::abs(frontToBack.y))) <= 1; //check if contour is closed
  if (isClosed) std:: cout << " contour is closed!!!"<< std::endl;
  cv::Point2f pplus, pminus;
  cv::Point2f f1stDerivative, f2ndDerivative;
  for (int i = 0; i < vecContourPoints[j].size(); i++ )
      {

       const cv::Point2f &pos = vecContourPoints[j][i];
       int maxStep = step;
       /*if (!isClosed)
               {
                 maxStep = std::min( std::min(step, i) , (int) vecContourPoints[j].size()-1-i);
                 if (maxStep == 0)
                   {
                     vecCurvature[i] = std::numeric_limits<double>::infinity();
                     continue;
                   }
               }*/

        int iminus = i-maxStep;
        int iplus = i+maxStep;
        pminus = vecContourPoints[j][iminus < 0 ? iminus + vecContourPoints[j].size() : iminus];
        pplus = vecContourPoints[j][iplus > vecContourPoints[j].size() ? iplus - vecContourPoints[j].size() : iplus];


        f1stDerivative.x =   (pplus.x -        pminus.x) / (iplus-iminus);
        f1stDerivative.y =   (pplus.y -        pminus.y) / (iplus-iminus);
        f2ndDerivative.x = (pplus.x - 2*pos.x + pminus.x) / ((iplus-iminus)/2*(iplus-iminus)/2);
        f2ndDerivative.y = (pplus.y - 2*pos.y + pminus.y) / ((iplus-iminus)/2*(iplus-iminus)/2);

        double curvature2D;
        double divisor = f1stDerivative.x*f1stDerivative.x + f1stDerivative.y*f1stDerivative.y;
        if ( std::abs(divisor) > 10e-8 )
          {
            curvature2D =  std::abs(f2ndDerivative.y*f1stDerivative.x - f2ndDerivative.x*f1stDerivative.y) /
                  pow(divisor, 3.0/2.0 )  ;
          }
        else
          {
            curvature2D = std::numeric_limits<double>::infinity();
          }

        vecCurvature[i] = curvature2D;
        vecCurvaturePosition[i].x = pos.x;
        vecCurvaturePosition[i].y =pos.y;
      }



  int maxElementIndex = std::max_element(vecCurvature.begin(), vecCurvature.end())- vecCurvature.begin();
  //return the amount of curvature vectors found
  std::cout << vecCurvaturePosition[maxElementIndex]<<std::endl;

  TCP =  vecCurvaturePosition[maxElementIndex];


  return TCP;
}

}





















void convexHull(int, void*)
{
  cv::Mat canny_output;
  //cv::imshow("cannyinput", canny);
  cv::Canny(canny, canny_output, thresh, thresh*ratio,kernel_size);
  //cv :: imshow("canny", canny_output);
  std::vector<std::vector<cv::Point>> contours;
  //std::vector<cv::Point> contours;
  cv::findContours(canny_output, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  cv::Mat drawing = cv::Mat::zeros( canny_output.size() , CV_8UC3 );
  tcp =  toolCenterPoint2(contours,step_min);
  cv::circle(drawing, tcp , 5, cv::Scalar( 255, 255, 255 ),-2,8,0);
  //cv::circle(drawing, toolCenterPoint1(contours), 5, cv::Scalar( 255, 255, 255 ),-2,8,0);

  std::vector<std::vector<cv::Point>>  hull( contours.size() );
  for(size_t i = 0; i< contours.size(); i++)
  {
    cv::convexHull(contours[i],hull[i]);

  }

  for( size_t i = 0; i< contours.size(); i++ )
  {
      cv::Scalar color = cv::Scalar( 0,255,0 );
      cv::Scalar color1 = cv::Scalar( 0,0,255 );
      cv::drawContours( drawing, contours, (int)i, color1 );
      cv::drawContours( drawing, hull, (int)i, color );
  }
  cv::imshow( window_nameCH, drawing );

}


/*void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try

  {
    cv::normalize(cv_bridge::toCvShare(msg, "bgr8")->image , normal,0, 255,cv::NORM_MINMAX,-1,cv::Mat());
    cv::medianBlur(normal,blur,3);
    cv::cvtColor(blur, gray , cv::COLOR_RGB2GRAY);
    binarize(0,0);
    frame2 = skeletonize(gray);

    cv::imshow("skeleton", frame2);
    cv::imshow("normal",normal);
    cv::waitKey(10);

    std::vector<Point> vect = convertToROSMsg(frame2);
    package1::vectorOfPoints msg;

    for (std::vector<Point>::iterator it = vect.begin(); it != vect.end(); ++it)
    {   //std::cout << "x_0 coordinate: " << (*it).x ;
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = 0;

        msg.points.push_back(point);
        std::cout << "x_0 coordinate: " << msg.points[0].x<< "             y_0 coordinate: " << msg.points[0].y << std::endl;
    }

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}*/

class SubscribeAndPublish
{

private:
  ros::NodeHandle p;
  package1::vectorOfPoints points1;
  geometry_msgs::Point tcp_msg;
  std::vector<geometry_msgs::Point> geomvec;



public:
  rviz_visual_tools::RvizVisualToolsPtr visual_tools;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv::normalize(cv_bridge::toCvShare(msg, "bgr8")->image , normal,0,255,cv::NORM_MINMAX,-1,cv::Mat());
      //cv::medianBlur(normal,blur,3);
      //cv::GaussianBlur( normal, normal, cv::Size(9,9), 0, 0, cv::BORDER_DEFAULT );
      cv::cvtColor(normal, gray , cv::COLOR_RGB2GRAY);

      canny = gray;
      cv::blur(canny, canny, cv::Size(5,5));
      convexHull(0,0);
      binarize(0,0);
      //cv::imshow("gray", gray);
      frame2 = skeletonize(gray);

      //cv::imshow("convexHull", frame2);
      cv::imshow("normal",normal);
      cv::waitKey(10);

      std::vector<Point> vect = convertToROSMsg(frame2);
      package1::vectorOfPoints msg;

      for (std::vector<Point>::iterator it = vect.begin(); it != vect.end(); ++it)
      {   //std::cout << "x_0 coordinate: " << (*it).x ;
          geometry_msgs::Point point;
          point.x = (*it).x;
          point.y = (*it).y;
          point.z = 0;
          geomvec.push_back(point);


          msg.points.push_back(point);
          //std::cout << "x_0 coordinate: " << msg.points[0].x<< "             y_0 coordinate: " << msg.points[0].y << std::endl;
      }
      points1 = msg;

      tcp_msg.x =tcp.x;
      tcp_msg.y =tcp.y;
      tcp_msg.z = 0;

      //definition of visualization_msgs
      std::cout<< "blabla"<< tcp.x<< std::endl;
      visual_tools->publishSpheres(geomvec, rviz_visual_tools::BLUE , rviz_visual_tools::XXXLARGE   , "Spheres");
      visual_tools->trigger();
      //visual_tools->publishSpheres(tcp_msg, rviz_visual_tools::BLUE , rviz_visual_tools::XXXLARGE   , "Spheres");
      //visual_tools->publishSphere(tcp_msg, rviz_visual_tools::BLUE , rviz_visual_tools::MEDIUM   , "Sphere" ,0);
      //visual_tools->trigger();
      visual_tools->deleteAllMarkers();
      geomvec.clear();



    }

    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

  }

  void imagePublisher(ros::Publisher* pub)
  {

    pub->publish(points1);

  }


};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "skeleton");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<package1::vectorOfPoints>("SkeletonPoints", 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("SkeletonPointsrviz", 10);
  ros::Rate loop_rate(30);



  cv::namedWindow(window_name);
  cv::namedWindow(window_nameCH);
  cv::createTrackbar( trackbar_value, window_name, &threshold_value, max_value, binarize);
  cv::createTrackbar("Canny threshold: ", "convexHull", &thresh, max_thresh, convexHull);
  cv::createTrackbar("[tcp1]Curvature threshold Max: ", "convexHull", &curve_thresh_max, max_curve_thresh, convexHull);
  cv::createTrackbar("[tcp1]Curvature threshold Min: ", "convexHull", &curve_thresh_min, max_curve_thresh, convexHull);
  cv::createTrackbar("[tcp2]step size: ", "convexHull", &step_min, step_max, convexHull);
  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  SubscribeAndPublish listener;
  image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, &SubscribeAndPublish::imageCallback, &listener);
  listener.visual_tools.reset(new rviz_visual_tools::RvizVisualTools("world", "/rvizskel"));

  listener.visual_tools->loadMarkerPub();

  listener.visual_tools->deleteAllMarkers();
  listener.visual_tools->trigger();


  //ros::Publisher pub = nh.advertise<package1::vectorOfPoints>("SkeletonPoints", 1);
  //ros::Rate loop_rate(0.5);
  // the message to be published
   //package1::vectorOfPoints msg;

   while (ros::ok())
     {


   listener.imagePublisher(&pub);



    ros::spinOnce();


   }

  ros::spin();

  cv::destroyWindow(window_name);
}
