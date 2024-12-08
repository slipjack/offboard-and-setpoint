#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>



mavros_msgs::State current_state;

void state_callback(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

const char* msg = R"(
control Your drone!
---------------------------
Moving around:
   q    w    e
   a    s    d  
        x
w/x : increase/decrease linear velocity z
q/e : increase/decrease linear velocity x
a/d : increase/decrease angular velocity z
space key, s : keeping
CTRL-C to quit
)";
char keys =' ';



int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;
  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);
  // Get the current character
  ch = getchar();
  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

void vels(double target_linear_vel_x,double target_linear_vel_z,double target_angular_vel){
    printf("\ncurrently:\nlinear vel_x %lf\n linear vel_z %lf\n angular vel %lf\n", target_linear_vel_x,target_linear_vel_z,target_angular_vel);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_key");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
      ("mavros/state", 10, state_callback);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
      ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
      ("mavros/set_mode");
    ros::Publisher key_pub = nh.advertise<geometry_msgs::Twist>
      ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);	

  ROS_INFO("Initializing...");
  // wait for FCU connection
  while(ros::ok() && !current_state.connected)
   {
    ros::spinOnce();
    //rate.sleep();
   }

  ROS_INFO("Connected.");


    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    
    double target_linear_vel_x = 0.0;
    double target_linear_vel_z = 0.0;
    double target_angular_vel = 0.0;
    double control_linear_vel_x  = 0.0;
    double control_linear_vel_z  = 0.0;
    double control_angular_vel = 0.0;
    printf("%s", msg);
    while(ros::ok()){
        keys = getch();
        printf("%c\n",    keys);
        if(keys == 'w'){
            target_linear_vel_z += LIN_VEL_STEP_SIZE;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        }else if(keys == 'x'){
            target_linear_vel_z -= LIN_VEL_STEP_SIZE;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        }else if(keys == 'a'){
            target_angular_vel += LIN_VEL_STEP_SIZE;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        }else if(keys == 'd'){
            target_angular_vel -= LIN_VEL_STEP_SIZE;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        }else if(keys == 'q'){
            target_linear_vel_x += LIN_VEL_STEP_SIZE;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        }else if(keys == 'e'){
            target_linear_vel_x -= LIN_VEL_STEP_SIZE;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        }else if(keys ==' ' or keys =='s'){
            target_linear_vel_x = 0.0;
            target_linear_vel_z = 0.0;
            target_angular_vel = 0.0;
            control_linear_vel_x  = 0.0;
            control_linear_vel_z = 0.0;
            control_angular_vel = 0.0;
            vels(target_linear_vel_x,target_linear_vel_z,target_angular_vel);
        } else {
            if(keys == '\x03')
                break;
        }
        twist.linear.x = target_linear_vel_x;
        twist.linear.y = 0.0;
        twist.linear.z = target_linear_vel_z;
        twist.angular.x = 0.0;
        twist.angular.y = 0,0;
        twist.angular.z = target_angular_vel;


        key_pub.publish(twist);

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  ros::Time time_start = ros::Time::now();  
  while(ros::ok())
   {
     if ( current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
       {
         if( set_mode_client.call(offb_set_mode) &&
	    offb_set_mode.response.mode_sent)
           {
	    ROS_INFO("Offboard enabled");
           }
         last_request = ros::Time::now();
        } 
     else 
        {
          if( !current_state.armed &&
	    (ros::Time::now() - last_request > ros::Duration(5.0)))
           {
	      if( arming_client.call(arm_cmd) &&
	        arm_cmd.response.success)
                {
	         ROS_INFO("Vehicle armed");










	        }
	last_request = ros::Time::now();
      }
  }

        ros::spinOnce();
    }
}
    return 0;
}
