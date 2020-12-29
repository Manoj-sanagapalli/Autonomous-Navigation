#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tom_3d_control/BaseVelocities.h>
#include <sensor_msgs/Imu.h>
#include <math.h>

double vel_x = 0.0;
double vel_y = 0.0;
double ang_z = 0.0;
double imu_z = 0.0;

/*void velCallback( const geometry_msgs::Twist& vel) {
    ros::Time current_time = ros::Time::now();

    vel_x = vel.linear.x;
    vel_y = 0.0;
    ang_z = vel.angular.z;
    //ROS_INFO("ang_z : [&lf]", ang_z);
}*/

void velCallback( const tom_3d_control::BaseVelocities& vel) {
    ros::Time current_time = ros::Time::now();

    vel_x = vel.linear_vel_x;
    vel_y = 0.0;
    ang_z = vel.angular_vel_z;
    //ROS_INFO("ang_z : [&lf]", ang_z);
}

void IMUCallback( const sensor_msgs::Imu& imu){
    //callback every time the robot's angular velocity is received
    ros::Time current_time = ros::Time::now();
    //this block is to filter out imu noise
    if(imu.angular_velocity.z > -0.03 && imu.angular_velocity.z < 0.03)
    {
        imu_z = 0.00;
    }
    else
    {
        imu_z = imu.angular_velocity.z;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "base_controller");
    ros::NodeHandle n;
    ros::NodeHandle nh_private_("~");
    //ros::Subscriber sub = n.subscribe("/cmd_vel", 50, velCallback);
    ros::Subscriber sub = n.subscribe("/base_velocities", 50, velCallback);
    ros::Subscriber imu_sub = n.subscribe("/imu_data", 50, IMUCallback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    double rate = 20.0;
    double x_pos = 0.0;
    double y_pos = 0.0;
    double theta = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(rate);
    while(n.ok()){
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();

        double linear_velocity_x = vel_x;
        double linear_velocity_y = 0.0;
        double angular_velocity_z =  ang_z;

	//ROS_INFO("angular_velocity_z : [&lf]", angular_velocity_z);
	//ROS_INFO("linear_velocity: [%f]", linear_velocity_x);

        double dt = (current_time - last_time).toSec();

        //calculate angular displacement  θ = ω * t
        double delta_theta = angular_velocity_z * dt; //radians
        double delta_x = (linear_velocity_x * cos(theta) - linear_velocity_y * sin(theta)) * dt; //m
        double delta_y = (linear_velocity_x * sin(theta) + linear_velocity_y * cos(theta)) * dt; //m

        //calculate current position of the robot
        x_pos += delta_x;
        y_pos += delta_y;
        theta += delta_theta;

        //calculate robot's heading in quarternion angle
        //ROS has a function to calculate yaw in quaternion angle
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        //robot's position in x,y, and z
        odom_trans.transform.translation.x = x_pos;
        odom_trans.transform.translation.y = y_pos;
        odom_trans.transform.translation.z = 0.0;
        //robot's heading in quaternion
        odom_trans.transform.rotation = odom_quat;
        //publish robot's tf using odom_trans object
        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        //robot's position in x,y, and z
        odom.pose.pose.position.x = x_pos;
        odom.pose.pose.position.y = y_pos;
        odom.pose.pose.position.z = 0.0;
        //robot's heading in quaternion
        odom.pose.pose.orientation = odom_quat;
        

        odom.child_frame_id = "base_link";
        //linear speed from encoders
        odom.twist.twist.linear.x = linear_velocity_x;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;

        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        //angular speed from IMU or encoders
        odom.twist.twist.angular.z = angular_velocity_z;

        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }
}
