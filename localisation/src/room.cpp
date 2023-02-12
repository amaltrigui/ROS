#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>



using namespace std;
double x,y;
char room_id;

void localize(const geometry_msgs::PoseStamped::ConstPtr& msg){
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    if (x > 5)
    {
        room_id = 'D';
    }
    else
    {
        if (-6.5 > y && y> -13.5)
        {
            room_id = 'C';
        }
        else if (-3.5 > y && y > -6.5)
        {
            room_id = 'B';
        }
        else
        {
            room_id = 'A';
        }       
    }
} 


int main(int argc, char** argv)  
{  
    ros::init(argc, argv, "localisation");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Subscriber sub = n.subscribe("/slam_out_pose", 1, localize);

    while(ros::ok())
    {
        ROS_INFO("x : %f,    y : %f", x, y);
        ROS_INFO("I am in Room: %c", room_id);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
} 


