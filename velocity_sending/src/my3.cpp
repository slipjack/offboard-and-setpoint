#include<ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include<geometry_msgs/Twist.h>

#include <mavros_msgs/CommandBool.h>

#include <mavros_msgs/SetMode.h>

#include <mavros_msgs/State.h>


mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


//#include<iostream>

//using namespace std;

int main(int argv, char **argc)
{
  geometry_msgs::Twist msg;
  
  ros::init(argv,argc,"velocity_sending_node");
  
  ros::NodeHandle n;
  
  ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped",1000);
  
   ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

   ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

   ros::Rate rate(20.0);
   //##ros::Rate loop_rate(15);

    // 等待飞控连接mavros，current_state是我们订阅的mavros的状态，连接成功在跳出循环
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //先实例化一个geometry_msgs::PoseStamped类型的对象，并对其赋值，最后将其发布出去
    //geometry_msgs::PoseStamped pose;

    geometry_msgs::Twist pose;
    //##ros::Publisher pub = n.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/  cmd_vel_unstamped",1000);

//###include <geometry_msgs/PoseStamped.h>
//###include<geometry_msgs/Twist.h>


  msg.linear.x=2;
  msg.linear.y=2;
  msg.linear.z=1;

  msg.angular.x=2;
  msg.angular.y=2;
  msg.angular.z=1;


    //建立一个类型为SetMode的服务端offb_set_mode，并将其中的模式mode设为"OFFBOARD"，作用便是用于后面的
    //客户端与服务端之间的通信（服务）  
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";


    //建立一个类型为CommandBool的服务端arm_cmd，并将其中的是否解锁设为"true"，作用便是用于后面的
    //客户端与服务端之间的通信（服务）
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //更新时间
    ros::Time last_request = ros::Time::now();



  while(ros::ok()) 
  {
        //首先判断当前模式是否为offboard模式，如果不是，则客户端set_mode_client向服务端offb_set_mode发起请求call，
        //然后服务端回应response将模式返回，这就打开了offboard模式
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");//打开模式后打印信息
            }
            last_request = ros::Time::now();
        }
        else //else指已经为offboard模式，然后进去判断是否解锁，如果没有解锁，则客户端arming_client向服务端arm_cmd发起请求call
            //然后服务端回应response成功解锁，这就解锁了
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");//解锁后打印信息
                }
                last_request = ros::Time::now();
            }
        }

    pub.publish(msg);//发布位置信息

    ros::spinOnce();
    rate.sleep();
    //##loop_rate.sleep();
  }
    return 0;
}
