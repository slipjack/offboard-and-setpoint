/*
 * =====================================================================================
 *
 *       Filename:  velocity_sending_node.cpp
 *
 *    Description:  Send velocity to Ardupilot
 *
 *        Version:  1.0
 *        Created:  Thursday 10 May 2018 05:51:53  IST
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Shailesh Chandra Prusty, shailesh.prusty@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
//#include<iostream>

//using namespace std;

int main(int argv, char **argc)
{
  geometry_msgs::Twist msg;
  
  ros::init(argv,argc,"velocity_sending_node");
  
  ros::NodeHandle n;
  
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped",1000);
  
  ros::Rate loop_rate(10);
  
  msg.linear.x=0;
  msg.linear.y=0;
  msg.linear.z=0;

  msg.angular.x=0;
  msg.angular.y=0;
  msg.angular.z=2;
  
  while(ros::ok()) 
  {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
