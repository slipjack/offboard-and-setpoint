#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
//#include<iostream>

//using namespace std;


//#include <unistd.h>
//void sleep(unsigned long seconds);


int main(int argv, char **argc)
{
  geometry_msgs::Twist msg;
  
  ros::init(argv,argc,"velocity_sending_node");
  
  ros::NodeHandle n;
  
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped",1000);
  
  ros::Rate loop_rate(10);
  
  msg.linear.x=8;
  msg.linear.y=8;
  msg.linear.z=3;

  msg.angular.x=5;
  msg.angular.y=7;
  msg.angular.z=2;
  
  while(ros::ok()) 
  {
    
    pub.publish(msg);
     
    ros::spinOnce();
    loop_rate.sleep();
  }
  
}
