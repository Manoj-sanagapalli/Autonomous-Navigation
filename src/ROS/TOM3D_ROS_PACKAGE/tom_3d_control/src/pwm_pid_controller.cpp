#include <ros/ros.h>
#include <tom_3d_control/PidData.h>
#include <std_msgs/Float64.h>
#include <math.h>

ros::Time current_time, last_time;

double dt;

double errSum, lastErr;
double kp, ki, kd;
double upper_limit, lower_limit;

std::string set_cur_topic_; 

std_msgs::Float64 pwm_pid_msg;

void pwmCallback(const tom_3d_control::PidData::ConstPtr& pwm_msg)
{
 double Setpoint = pwm_msg->setpoint_data;
 double Input = pwm_msg->measured_data;
 double Output;

 //ros::Time current_time, last_time;

 //current_time = ros::Time::now();
 
 //double dt = (current_time - last_time).toSec();

 double error = Setpoint - Input;

 errSum += (error * dt);
 
 double dErr = (error - lastErr) / dt;

 /*Compute PID Output*/
  Output = kp * error + ki * errSum + kd * dErr;
 
  if (Output >= upper_limit)
  {
   Output = upper_limit;
  }
   else if (Output <= lower_limit)
   {
     Output = lower_limit;
   }
   else if (Setpoint == 0)
   {
     Output = 0;
   }

  pwm_pid_msg.data = Output;

 lastErr = error;
}
 

int main(int argc, char **argv)
 {
  ros::init(argc, argv, "pid_controller");
 
  ros::NodeHandle nh;

  nh.param("kp", kp, 0.5);
  nh.param("ki", ki, 0.5);
  nh.param("kd", kd, 0.0);
  nh.param("upper_limit", upper_limit, 60.0);
  nh.param("lower_limit", lower_limit, -60.0);

  ros::Rate loop_rate(30); 

  //nh.param<std::string>("set_cur_topic", set_cur_topic_, "/pwm_values");
  
  ros::Subscriber sub = nh.subscribe("/rpm_values", 1000, pwmCallback);

  ros::Publisher pwm_pub = nh.advertise<std_msgs::Float64>("/rpm_control_values", 1000);
   
  current_time = ros::Time::now();
  last_time = ros::Time::now();
 
  while(ros::ok())
  { 
   
   ros::spinOnce();

   ros::Time current_time = ros::Time::now();

   dt = (current_time - last_time).toSec();

   pwm_pub.publish(pwm_pid_msg);

   last_time = current_time;

   loop_rate.sleep();
   }
 }
 
