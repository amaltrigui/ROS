#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>



using namespace std;

float circle_center_x=0, circle_center_y=0;
int circle_radius=0;
int count_circles=0;
double x,y;
char room_id;
bool onceSeen =false;


void image_processing(const sensor_msgs::ImageConstPtr& msg){
    // create pointer to cv Image
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cv_image,cv_image_out;
    // Copy the ros image message to cv::Mat.
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }  
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }  
    // create object cv_image of class cv::Mat that stores the image  
    // also create object cv_image_out of class cv::Mat that stores the blurred and flipped image  
    (cv_ptr->image).copyTo(cv_image);
    // flip the cv image
    cv::flip(cv_image,cv_image,1);
    // save the blured cv image in cv_image_out
    cv::blur(cv_image,cv_image_out,cv::Size(10,10));
    // visualise the original image in the scene after transformations
    imshow("Original Image", cv_image_out);

    // RGB vector of type cv::Vec3b that store the RBG VALUES
    cv::Vec3b RGB;
    // Filtering the colors: only focus on the colors of the yellow ball 
    for(int i=0;i<cv_image_out.rows;i++){
        for(int j=0;j<cv_image_out.cols;j++){
            // read the RGB values of each pixel
            RGB=cv_image_out.at<cv::Vec3b>(i,j);
            // The yellow ball as defined in the project task description has the colors RGB : 255, 255 , 0. 
            // It appeared to us that specifying exactly those values in the if-else condition block below did not allow us
            // to follow the ball at all. On the other side, using a range if RBG values to characterize the yellow color of the ball 
            // gave the robot more flexibility to detect the yellow ball.
            // Red RGB [2]: instead of 255, specifiy a range of 160-255
            // Green RGB [1]: instead of 255, specify a range of 80-255
            // Blue RGB [0] : instead of 0, specify a range of 0-140
            if((RGB[2]>=160)&&(RGB[1]>=80)&&(RGB[0]<=140)){
            	// if the color of ball is detected, keep the rbg values
                cv_image_out.at<cv::Vec3b>(i,j)=cv_image_out.at<cv::Vec3b>(i,j);
            }
            else{
                // mark the area around the ball (if ball detected) or all the image (if no ball detected) with black for example
                cv_image_out.at<cv::Vec3b>(i,j)=0;
            }
        } 
    }
    // Process the image: first copy the cv_image_out to a new cv::mat object srcIMage
    cv::Mat srcImage = cv_image_out;
    cv::Mat midImage, dstImage;
    // convert the color of the copie of cv_image_out to grey and save it in midImage serving for the edge dection.
    cv::cvtColor(srcImage, midImage, CV_BGR2GRAY);
    // Smooth the image and save the smoothed image in midImage as well
    cv::GaussianBlur( midImage, midImage, cv::Size(9, 9), 2, 2 );
    // apply the HoughCircles transformation from cv to find the circles in the image.
    // Parameters of the transofrmation are : 
    // *** midImage :grayscale input image		*** circles :Output vector of found circles (each has (x,y,radius))  	***  CV_HOUGH_GRADIENT: detection method
    // *** dp = 1.5 , recommended value for the inverse ratio of the accumulator resolution to the image resolution
    // *** minDist = 10 , the minimum distance between the centers of the detected circles
    // *** param1 = 200, the higher threshold of the two passed to the Canny edge detector 
    // *** param2 = 100, the accumulator threshold for the circle centers at the detection stage
    // *** minRadius = 0, Minumum circle radius.
    // *** maxRadius = 0, Maximum circle radius (if <= 0, uses the maximum image dimension)
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles( midImage, circles, CV_HOUGH_GRADIENT, 1.5, 10, 200, 100, 0, 0 );
    // after obtaining the circle(s), we iterate through them all and draw them
    count_circles = circles.size() ;
    /*circle_center_x_avg= 0;
    circle_center_y_avg =0;
    circle_radius_avg = 0;*/

    for( size_t i = 0; i < circles.size(); i++ ){
        onceSeen =true;
        circle_center_x = circles[i][0];
        circle_center_y=  circles[i][1];
        /* cout<< "number of circles is: " << circles.size(); */
        circle_radius = cvRound(circles[i][2]);
        // draw the circle in the srcImage and its mark contour
        cv::Point center(circle_center_x, circle_center_y);
        cv::circle( srcImage, center, 5, cv::Scalar(0,0,255), -1, 8, 0);
        cv::circle( srcImage, center, circle_radius, cv::Scalar(255,0,0), 3, 8, 0 );
    }
    
    /*if (count_circles > 0) {
    	circle_center_x_avg = circle_center_x_avg / count_circles;
    	circle_center_y_avg = circle_center_y_avg / count_circles;
    	circle_radius_avg = circle_radius_avg / count_circles;
    }*/
    
    cv::imshow("Tracking Yellow Ball", srcImage);
    cv::waitKey(3);
} 
  
int main(int argc, char** argv)  
{  
	// ros messages
    ros::init(argc, argv, "ball");
    /* ROS_INFO("&&&&& the robot will follow the yellow ball &&&&&"); */
	// define publisher and subscriber
    ros::NodeHandle n, nh;
    geometry_msgs::Twist velocityCommand;
    ros::Rate loop_rate(10);
    ros::Publisher  pub = nh.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 3); // cooporates with velocityCommand
    ros::Subscriber sub = n.subscribe("/vrep/image", 1, image_processing); // callable is image_processing
  
    
    while(ros::ok()){
   
        if ( onceSeen==true )
        {
            int horizontalTolerance = 20;
            int radiusGoal = 150;
            int sizeTolerance = 10;
            if (circle_center_x == 0){
                velocityCommand.linear.x  =0;
                velocityCommand.angular.z = 0.8;
            }else {
                if (circle_center_x > (256 - horizontalTolerance)) {
                    velocityCommand.angular.z = -0.2;
                }
                else {
                    if (circle_center_x < (256 + horizontalTolerance)) {
                        velocityCommand.angular.z = 0.2;
                    }
                }
            }
        
            if (circle_radius == 0){
                velocityCommand.linear.x  = 0;
                velocityCommand.angular.z = 0.8;
            }else {
                if (circle_radius > (radiusGoal + sizeTolerance)) {
                    velocityCommand.linear.x= - 0.6;
                }
                else {
                    if (circle_radius < (radiusGoal - sizeTolerance)) {
                        velocityCommand.linear.x = 0.6;
                    }
                }
            }
            ROS_INFO("targetX= %f, targetY = %f", circle_center_x, circle_center_y);
            ROS_INFO("Tracking Command: linear= %f, angular = %f", velocityCommand.linear.x, velocityCommand.angular.z);
            pub.publish(velocityCommand);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}  
