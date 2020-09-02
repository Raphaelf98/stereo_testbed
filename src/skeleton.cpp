#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include<opencv2/core/operations.hpp>
#include<math.h>
#include <vector>
#include<Eigen/Dense>
#include<geometry_msgs/Point.h>
#include<visualization_msgs/Marker.h>
#include<package1/vectorOfPoints.h>
#include<package1/cameraTransformation.h>
#include<sensor_msgs/PointCloud2.h>
#include<rviz_visual_tools/rviz_visual_tools.h>


//#include<libalglib/linalg.h>

//#include<libalglib/interpolation.h>
//#include<libalglib/ap.h>

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
int k=1;
int curve_thresh_max = 0;
int curve_thresh_min = 0;
int max_curve_thresh =1000;
cv::Point2i  tcp;
int step_min = 0;
int step_max = 100;
bool snap = true;
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
   //cv::threshold(gray,gray,70, max_BINARY_value, 1);
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

  //cv::imshow( "Approximation", img );
}


//Skeletonization and returns std::vector<cv::Points>
cv::Mat skeletonize(cv::Mat frame)
{
  bool done;
   cv::Mat skel(frame.size(), CV_8UC1, cv::Scalar(0));
  //cv::Mat temp(frame.size(), CV_8UC1);
  cv::Mat temp;
  cv::Mat eroded;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat element1= cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));
// source: http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/


    //cv::imshow("input ", frame);
  //int c = 1;
  do

  { //cv::imshow("frame", frame);
      //input frame is opened and saved to temp
    //if (snap) cv::imwrite("/home/ulzea/RESULTS/skeletonalgorithm/binaryimage/binarayimage_"+ std::to_string(c)+ ".png", frame);
    cv::erode(frame, eroded, element);
    //if (snap) cv::imwrite("/home/ulzea/RESULTS/skeletonalgorithm/eroded/erosion_"+std::to_string(c)+ ".png", eroded);
    cv::dilate(eroded, temp, element); // temp = open(img)
    //cv::imshow("opening", temp);
    //if (snap) cv::imwrite("/home/ulzea/RESULTS/skeletonalgorithm/opened/opening_"+ std::to_string(c)+ ".png", temp);
    //substract input frame from temp (opening result)´
    //Def. substract: Calculates the per-element difference between two arrays or array and a scalar
    cv::subtract(frame, temp, temp);
    //if (snap) cv::imwrite("/home/ulzea/RESULTS/skeletonalgorithm/subtracted/subtraction_"+std::to_string(c)+ ".png", temp);
    cv::imshow("subtracted ", temp);
    //cv::imshow("skel ", skel);
    // add the result from substract to every skel from last iteration
    cv::bitwise_or(skel, temp, skel);
    //if (snap) cv::imwrite("/home/ulzea/RESULTS/skeletonalgorithm/skeleton/skeleton_"+std::to_string(c)+ ".png", skel);
    //cv::imshow("or",skel);
    //every new iteration uses the frame eroded
    eroded.copyTo(frame);
    //until no frame exists, then the loop is exited


    done = (cv::countNonZero(frame) == 0);
    //c++;
  } while (!done);
   //snap = false;


cv::dilate(skel,skel,element);

cv::erode(skel, skel, element);

//cv::dilate(skel,skel,element1);
cv::imshow("skeleton", skel);

//if (snap == true) cv::imwrite("/home/ulzea/RESULTS/skeleton.png", skel); snap= false;
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


       //if (curvature2D < (double)curve_thresh_max/1000 && curvature2D > (double)curve_thresh_min/1000)
      // {

       vecCurvature[i] = curvature2D;
       std::cout<< "curvature: "<< vecCurvature[i] << std::endl;
       vecCurvaturePosition[i].x = pos.x;
       vecCurvaturePosition[i].y =pos.y;
       //}
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
  //std::cout << vecCurvaturePosition[maxElementIndex]<<std::endl;

  TCP =  vecCurvaturePosition[maxElementIndex];


  return TCP;
}

}

void convexHull(int, void*)
{
  cv::Mat canny_output;
  //cv::imshow("cannyinput", canny);
  cv::Canny(canny, canny_output, thresh, thresh*ratio,kernel_size);
  //if (snap == true) cv::imwrite("/home/ulzea/RESULTS/cannyimage.png", canny_output); snap= false;
  //cv :: imshow("canny", canny_output);
  std::vector<std::vector<cv::Point>> contours;
  //std::vector<cv::Point> contours;
  cv::findContours(canny_output, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  cv::Mat drawing = cv::Mat::zeros( canny_output.size() , CV_8UC3 );
  //tcp =  toolCenterPoint2(contours,step_min);
  //tcp =  toolCenterPoint2(contours,4);
 // tcp= toolCenterPoint2(contours,5);
  cv::circle(drawing, tcp , 8, cv::Scalar( 255, 255, 255 ),-2,8,0);
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


 //cv::imwrite("/home/ulzea/RESULTS/worm/frame_ "+std::to_string(k)+ ".png", drawing);
  //k++;
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

  package1::vectorOfPoints points1;
  //geometry_msgs::Point tcp_msg;

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
  //rviz_visual_tools::RvizVisualToolsPtr visual_tools;
    //rviz_visual_tools::RvizVisualToolsPtr tcp_visual_tools;

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

    cv::Rodrigues(rot,RotationMatrix);

    //InvertedRotation = RotationMatrix.inv();
    cv::transpose(RotationMatrix,InvertedRotation);

    cv::subtract(input, translation,subs);


    cv::Mat output = InvertedRotation*cv::Mat(subs);


    transformedPoint.x = output.at<float>(0);
    transformedPoint.y = output.at<float>(1);
    transformedPoint.z = output.at<float>(2);

  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      cv::normalize(cv_bridge::toCvShare(msg, "bgr8")->image , normal,0,255,cv::NORM_MINMAX,-1,cv::Mat());
      spatialPointTest(normal);
      if (snap == true) cv::imwrite("/home/ulzea/RESULTS/thinwire.png", normal); snap= false;
      //cv::medianBlur(normal,blur,3);
      //cv::GaussianBlur( normal, normal, cv::Size(9,9), 0, 0, cv::BORDER_DEFAULT );
      cv::cvtColor(normal, gray , cv::COLOR_RGB2GRAY);

      //if (snap == true) cv::imwrite("/home/ulzea/RESULTS/blackandwhiteimage.png", gray); snap= false;
      canny = gray;
      cv::blur(canny, canny, cv::Size(5,5));
      convexHull(0,0);
      binarize(0,0);
      cv::imshow("gray", gray);
      frame2 = skeletonize(gray);

      //cv::imshow("convexHull", frame2);
      cv::imshow("normal",normal);
      cv::waitKey(10);

      SkeletonVect = convertToROSMsg(frame2);
      /*
      package1::vectorOfPoints msg;

      for (std::vector<Point>::iterator it = SkeletonVect.begin(); it != SkeletonVect.end(); ++it)
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

      points1.tcp.x =tcp.x;
      points1.tcp.y =tcp.y;
      points1.tcp.z = 0;

      Eigen::Vector3d test(0.0, 0.0, 0.0);
        //tcp_visual_tools->deleteAllMarkers();
      //visual_tools->deleteAllMarkers();
        //definition of visualization_msgs
        //std::cout<< "blabla"<< tcp.x<< std::endl;
      //visual_tools->publishSpheres(geomvec, rviz_visual_tools::RED , rviz_visual_tools::XXXXLARGE   , "Spheres");
      //visual_tools->trigger();
        //visual_tools->publishSpheres(tcp_msg, rviz_visual_tools::BLUE , rviz_visual_tools::XXXLARGE   , "Spheres");
        //visual_tools->publishSphere(test, rviz_visual_tools::BLUE , rviz_visual_tools::XXXLARGE );
        //visual_tools->trigger();

      geomvec.clear();
      SkeletonVect.clear();
      */

    }

    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

  }
  void point2CloudCallback(const sensor_msgs::PointCloud2Ptr& pCloud_msg)
  { std::cout<< "circle: ["<< circle.x<<","<<circle.y<<"]"<<std::endl;
    pixelTo3DPoint(*pCloud_msg, circle.x,circle.y,circle3D);

    std::cout<< "circle3D: ["<< circle3D.x<<","<<circle3D.y<<","<<circle3D.z<<"]"<<std::endl;
    std::cout<< "tcp 2D [p]: ["<< tcp.x<<","<<tcp.y<<"]"<<std::endl;
    //pixelTo3DPoint(*pCloud_msg, tcp.x, tcp.y , points1.tcp);
    std::cout<< "tcp 3D [m]: ["<< points1.tcp.x<<","<< points1.tcp.y<<","<< points1.tcp.z<<"]"<<std::endl;
    for (int iter = 0; iter < SkeletonVect.size(); iter++)
    {
      geometry_msgs::Point buff;
      int u = SkeletonVect[iter].x;
      int v = SkeletonVect[iter].y;

      pixelTo3DPoint(*pCloud_msg, u, v, buff);
      if(buff.x != 1000 &buff.y != 1000 &buff.z != 1000)
      {
      SpatialSkeleton.push_back(buff);
      }
      //SpatialSkeleton[iter].x = buff.x;
      //SpatialSkeleton[iter].y = buff.y;
      //SpatialSkeleton[iter].z = buff.z;
    }
     points1.points = SpatialSkeleton;
    SpatialSkeleton.clear();
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

         // put data into the point p

          //p.x = X;
          //p.y = Y;
          //p.z = Z;

          geometry_msgs::Point input_;

          input_.x = X;
          input_.y = Y;
          input_.z = Z;
          rigidBodyTransform(input_,p);

}
  void spatialPointTest(cv::Mat frme)
  {
    cv::Mat  image;
    cv::cvtColor(frme, image , cv::COLOR_RGB2GRAY);

    std::vector<cv::Vec3f> circles;

    GaussianBlur(image, image, cv::Size(9, 9), 2, 2 );
    HoughCircles(image, circles, CV_HOUGH_GRADIENT, 1, image.rows/8, 200, 100, 0, 0 );
    for( size_t i = 0; i < circles.size(); i++ )
      {
          cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          // circle center
          cv::circle( frme, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
          // circle outline
          cv::circle( frme, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
       }
    if (circles.size()>0)
    {
    circle.x = round( circles[0][0]);
    circle.y = round(circles[0][1]);

    cv::putText(frme, "Coordinates: x:"+ std::to_string(circle3D.x) +"  y:" +std::to_string(circle3D.y)+"  z:" + std::to_string(circle3D.z), cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.5,CV_RGB(0,255,255),2);
    }
    //
    cv::namedWindow( "Hough Circle Transform Demo", cv::WINDOW_AUTOSIZE );
    imshow( "Hough Circle Transform Demo", frme );
    circles.clear();
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
  //ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("SkeletonPointsrviz", 10);
  ros::Rate loop_rate(10);
  SubscribeAndPublish listener;





  cv::namedWindow(window_name);
  cv::namedWindow(window_nameCH);
  cv::createTrackbar( trackbar_value, window_name, &threshold_value, max_value, binarize);
  cv::createTrackbar("Canny threshold: ", "convexHull", &thresh, max_thresh, convexHull);
  cv::createTrackbar("[tcp1]Curvature threshold Max: ", "convexHull", &curve_thresh_max, max_curve_thresh, convexHull);
  cv::createTrackbar("[tcp1]Curvature threshold Min: ", "convexHull", &curve_thresh_min, max_curve_thresh, convexHull);
  cv::createTrackbar("[tcp2]step size: ", "convexHull", &step_min, step_max, convexHull);
  cv::startWindowThread();

  image_transport::ImageTransport it(nh);



  ros::Subscriber extrinsicParameter = nh.subscribe("/CameraTransform", 1, &SubscribeAndPublish::extrinsicParameterCallback, &listener);
  //image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, &SubscribeAndPublish::imageCallback, &listener);
  image_transport::Subscriber sub = it.subscribe("/stereo/left/image_rect", 10, &SubscribeAndPublish::imageCallback, &listener);
  ros::Subscriber sub2 = nh.subscribe("stereo/points2",10,&SubscribeAndPublish::point2CloudCallback, &listener);

  //listener.visual_tools.reset(new rviz_visual_tools::RvizVisualTools("world", "/rvizskel"));
      //listener.tcp_visual_tools.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviztcp"));
  //listener.visual_tools->loadMarkerPub();
      //listener.tcp_visual_tools->loadMarkerPub();
  //listener.visual_tools->deleteAllMarkers();
      //listener.tcp_visual_tools->deleteAllMarkers();

  //listener.visual_tools->trigger();
      // listener.tcp_visual_tools->trigger();


   while (ros::ok())
     {


      listener.POIPublisher(&pub);

      ros::spinOnce();

   }


  //ros::spin();

  cv::destroyWindow(window_name);
  cv::destroyWindow(window_nameCH);
}
