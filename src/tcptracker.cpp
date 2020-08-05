#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/tracking.hpp>



cv::Ptr<cv::Tracker> tracker=cv::TrackerTLD::create();;
bool boundingBoxIsSet = false;
 bool InitializeTracker= true;
std::string WINDOW_NAME = "tracker";
cv::Point pointTopLeft, pointBottomRight;
cv::Rect g_rectangle;
cv::Rect2d roi;
cv::Mat frame;
bool roi_ = true;
bool iteration= false;

bool g_bDrawingBox = false;
cv::RNG g_rng(0);  // Generate random number
bool InitialRectangle = true;

void DrawRectangle(cv::Mat& img, cv::Rect box)
{std::cout<<"Halloduda"<< std::endl;
   //Draw a rectangle with random color
   cv::rectangle(img, box.tl(), box.br(), cv::Scalar(g_rng.uniform(0, 255),
       g_rng.uniform(0, 255), g_rng.uniform(0, 255)));
   pointTopLeft = box.tl();
   pointBottomRight = box.br();
}


void on_MouseHandle(int event, int x, int y, int flags, void* param) {
    cv::Mat& image = *(cv::Mat*) param;
    switch (event) {
    case cv::EVENT_MOUSEMOVE: {    // When mouse moves, get the current rectangle's width and height
        if (g_bDrawingBox) {
            g_rectangle.width = x - g_rectangle.x;
            g_rectangle.height = y - g_rectangle.y;
        }
    }
                        break;
    case cv::EVENT_LBUTTONDOWN: {  // when the left mouse button is pressed down,
                               //get the starting corner's coordinates of the rectangle
        g_bDrawingBox = true;
        g_rectangle = cv::Rect(x, y, 0, 0);
    }
                          break;
    case cv::EVENT_LBUTTONUP: {   //when the left mouse button is released,
                              //draw the rectangle
        g_bDrawingBox = false;
        if (g_rectangle.width < 0) {
            g_rectangle.x += g_rectangle.width;
            g_rectangle.width *= -1;
        }

        if (g_rectangle.height < 0) {
            g_rectangle.y += g_rectangle.height;
            g_rectangle.height *= -1;
        }
        DrawRectangle(image, g_rectangle);
        boundingBoxIsSet = true;
        std::cout << "bounding box has been initialised" << std::endl;

    }
                        break;
    }
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{ bool ok =false;
  cv::Rect2d bbox;

   cv::Mat frame1;


  try

  {



    while(false )//InitialRectangle)
      {
      std::cout<<"Hallodu"<< std::endl;

      cv::Mat srcImage(1080,1920, CV_8UC3);
      cv::Mat tempImage;
      cv::namedWindow(WINDOW_NAME);
      cv::setMouseCallback(WINDOW_NAME, on_MouseHandle, (void*)&frame);
        while (!boundingBoxIsSet)
              {
              frame.copyTo(tempImage);
                  if (g_bDrawingBox) DrawRectangle(tempImage, g_rectangle);
                  cv::imshow(WINDOW_NAME, tempImage);
                  if (cv::waitKey(10) == 27) break;  // stop drawing rectanglge if the key is 'ESC'
              }



        InitialRectangle = false;

      }
    while (roi_)
         {
            frame =  cv_bridge::toCvShare( msg, "mono8")->image;

           //tracker = cv::TrackerTLD::create();

           roi = cv::selectROI("tracker",frame);
           tracker->init(frame,roi);

           roi_ = false;
          }





          if(false)//InitializeTracker)
          {

          std::cout<< "point"<< pointTopLeft<< std::endl;
          bbox = cv::Rect2d(pointTopLeft, pointBottomRight);

            cv::rectangle(frame, bbox, cv::Scalar(255, 0, 0), 2, 1);



          InitializeTracker= false;
          }




          std::cout<< "abc"<<std::endl;
          if(iteration)
          {
            frame1 =  cv_bridge::toCvShare(msg, "mono8")->image;

            std::cout<< "lalal" << std::endl;
            ok = tracker->update(frame1, roi);
              std::cout<< ok<< std::endl;
            if (ok)
              {
                  cv::rectangle(frame1, roi, cv::Scalar(255, 0, 0), 2, 1);

                }
               else
              {
               // tracking failure detected
                 cv::  putText(frame1, "Mosse Tracker", cv::Point(100, 20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);
                }

     //cv::putText(frame,  "Mosse Tracker", cv::Point(100, 20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50, 170, 50), 2);
               //Display FPS on frame
               //Display frame
    cv::imshow("WINDOW_NAME", frame1);
    cv::waitKey(1);
     }
          iteration = true;

  }

  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_sub");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);


  image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
