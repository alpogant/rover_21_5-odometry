//  @author Alperen Demirkol
//  @date 05.05.2022
//  21.5 itu-rover wheel odometry ros_publisher

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <exception>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#define CIRCUM(r) (2*M_PI*r)

using namespace std;

ros::Publisher pub_odom;
sensor_msgs::Imu imu;
std_msgs::Float64MultiArray hall;
ros::Time last_time;

double dx;     // displacament of rover
double Xx, Xy;  //  displacament vectors
double v;     // linear velocity of rover
double Vx, Vy;  //  velocity vectors
double dt;    // change in time (in seconds)
double init_yaw; // init yaw 
double yaw;   // change in angle according to initial position (in degrees)
double yaw1;  // change in angle according to magnetic east (in radians)
double dyaw;   // change in angle according to initial position (in radians)
double w;     // angular velocity
float err;    //  experimental

double av_rpm;
double av_right_rpm;
double av_left_rpm;

bool flag = false;  //for imu init value
int dir;   // 0 straight, 1 right, 2 left

namespace calc
{
    double displacament(float r, double rpm, double angle)
    {
        double circum = CIRCUM(r);
        dt = (ros::Time::now() - last_time).toSec();
        last_time = ros::Time::now();

        dx = dx + circum*rpm/60;
        Xx = dx*cos(angle);
        Xy = dx*sin(angle);

        return dx, Xx, Xy;
    }

    double velocity(float r, double rpm, double angle)
    {
        double ddx = CIRCUM(r)*rpm/60;
        v = ddx/dt;
        Vx = v*cos(angle);

        return v, Vx;
    }

    float error()
    /**
     * @brief calculates error, just for fun. UNUSED
     * TODO: error değeri kullanılarak farklı yüzeylerde yemre katsayısını dinamik olarak düzenleyen bir kod geliştirilebilir...
     */
    {
        w = w + (((av_left_rpm-av_right_rpm)/60)/1.1)*dt;  // W info from wheel
        err = (abs(yaw/dt - w)*100)/yaw;                 // Error percentege (according to w info given by imu)

        return err;
    }
}

namespace node
{
    void odometry_publisher()
    { 
        nav_msgs::Odometry odom;

        odom.header.stamp = last_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = Xx;
        odom.pose.pose.position.y = Xy;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.w = imu.orientation.w;
        odom.pose.pose.orientation.x = imu.orientation.x;
        odom.pose.pose.orientation.y = imu.orientation.y;
        odom.pose.pose.orientation.z = imu.orientation.z;
    
        odom.twist.twist.linear.x = Vx;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;

        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = yaw;

        pub_odom.publish(odom);
    }

    void imu_cb(const sensor_msgs::Imu::ConstPtr& ip)
    {
        imu = *ip;
        tf::Quaternion q(imu.orientation.x, imu.orientation.y,
                        imu.orientation.z, imu.orientation.w);
        tf::Matrix3x3 m(q);
        double roll,pitch,yaw1;
        m.getRPY(roll, pitch, yaw1);

        if (flag == false)  // for calculate beginning yaw
        {
            double yaws;

            for(int i=0; i<5; i++)
            {
                yaws = yaws + yaw1;
            }
            init_yaw = yaws/5 + M_PI/2;

		    // ROS_INFO("%f",init_yaw);
            flag = true;
        }

        else
        {
        	dyaw = (yaw1 - init_yaw) + M_PI/2;    // calculate yaw change and magnetic east to magnetic north
        	yaw = dyaw*(180/M_PI);  // Radians to degrees
            // ROS_INFO("%f",yaw);
        }
    }

    void hall_cb(const std_msgs::Float64MultiArray::ConstPtr& hp)
    {
        hall = *hp;
        av_rpm = (hall.data[0]+hall.data[1]+hall.data[2]+hall.data[3])/4; //    Avereage rpm of wheels
        av_right_rpm = (hall.data[0]+hall.data[1])/2;   //  Avereage rpm of right wheels
        av_left_rpm = (hall.data[2]+hall.data[3])/2;    //  Avereage rpm of left wheels

        if ((av_left_rpm - av_right_rpm) < 5)
        {
            dir = 2;
        }

        else if ((av_right_rpm - av_left_rpm) < 5)
        {
            dir = 1;
        }

        else
        {
            dir = 0;
        }
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "odometrilamasyon");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    ros::Subscriber sub_imu = nh.subscribe("imu1/data", 100, node::imu_cb);
    ros::Subscriber sub_hall = nh.subscribe("/drive_feedback_topic", 100, node::hall_cb);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/drive_system/odometry_info",100);
    last_time = ros::Time::now();

    while(ros::ok())
    {
        try 
        {
            calc::displacament(0.125, av_rpm, dyaw);
            calc::velocity(0.125, av_rpm, dyaw);
            calc::error();
            //ROS_INFO("error percentage is ", "%f", err);
            node::odometry_publisher();
        }

        catch (const exception& e) 
        {
            cerr << "Caught exception: " << e.what() << "\n";
            return 1;
        }
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
