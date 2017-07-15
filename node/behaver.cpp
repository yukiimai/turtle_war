#include "ros/ros.h"
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

//#define MAV 3.14
//#define MLV 2.0
#define MAV 6.28
#define MLV 0.5

#define MAX_VALUE 999

enum command
{
    RIGHT,
    RIGHT_FORWARD,
    FORWARD,
    LEFT,
    LEFT_FORWARD,
    BACK,
    STOP

};

class RoboCtrl
{
public:
    RoboCtrl()
    {
        filtered_angle = -1;
        red_angle = -1;
        yellow_angle = -1;

        filtered_distance = MAX_VALUE;
        red_distance = MAX_VALUE;
        yellow_distance = MAX_VALUE;

        ros::NodeHandle node;
        red_laser_sub = node.subscribe("red_points", 1, &RoboCtrl::redCallback, this);
        yellow_laser_sub = node.subscribe("yellow_points", 1, &RoboCtrl::yellowCallback, this);
        filtered_laser_sub = node.subscribe("filtered_points", 1, &RoboCtrl::filteredCallback, this);

        twist_pub_ = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

    void consider()
    {
        float target_angle = 0;
        //check distans
        if(yellow_distance < red_distance)
        {
            //yellow chase
            ROS_INFO("YELLOW APROCHE");
            target_angle = yellow_angle;
        }
        else if(red_distance < filtered_distance)
        {
            //red_chase
            ROS_INFO("RED APROCHE");
            target_angle = red_angle;
        }
        else
        {
            ROS_INFO("SEEK");
            target_angle = 90;
        }

        if(target_angle == 90)
        {
            moveRobo(RIGHT);
        }
        else if(target_angle < -15)
        {
            moveRobo(LEFT_FORWARD);
        }
        else if(target_angle > 15)
        {
            moveRobo(RIGHT_FORWARD);
        }
        else
        {
            moveRobo(FORWARD);
        }
    }

    void moveRobo(const int command_type)
    {
        geometry_msgs::Twist twist;

        int c;
        c = command_type;
        twist.linear.x = 0;
        twist.angular.z = 0;
        switch(c)
        {
        case LEFT:
            twist.linear.x = MLV/2;
            twist.angular.z = MAV;
            break;
        case LEFT_FORWARD:
            twist.linear.x = MLV;
            twist.angular.z = MAV;
            break;
        case RIGHT:
            twist.linear.x = MLV/2;
            twist.angular.z = -MAV;
            break;
        case RIGHT_FORWARD:
            twist.linear.x = MLV;
            twist.angular.z = -MAV;
            break;
        case FORWARD:
            twist.linear.x = MLV;
            twist.angular.z = 0;
            break;
        case BACK:
            twist.linear.x = -MLV;
            twist.angular.z = 0;

            break;
        case STOP:
            twist.linear.x = 0;
            twist.angular.z = 0;
            break;
        default:
            break;
        }
        twist_pub_.publish(twist);
    }

    void filteredCallback(const sensor_msgs::LaserScan &input)
    {
        float temp_min = MAX_VALUE;
        int data_no = -1;

        for(int i = 0 ; i < input.ranges.size() ; i++)
        {
            if(input.ranges[i] <  temp_min)
            {
                data_no = i;
                temp_min = input.ranges[i];
            }
        }

        if(data_no != -1)
        {
            filtered_distance = temp_min;
            filtered_angle = input.angle_min + data_no * input.angle_increment;
        }
        else
        {
            filtered_distance = MAX_VALUE;
            filtered_angle = -1;
        }
        return;
    }

    void redCallback(const sensor_msgs::LaserScan &input)
    {
        float temp_min = MAX_VALUE;
        int data_no = -1;

        for(int i = 0 ; i < input.ranges.size() ; i++)
        {
            if(input.ranges[i] <  temp_min)
            {
                data_no = i;
                temp_min = input.ranges[i];
            }
        }

        if(data_no != -1)
        {
            red_distance = temp_min;
            red_angle = input.angle_min + data_no * input.angle_increment;
        }
        else
        {
            red_distance = MAX_VALUE;
            red_angle = -1;
        }
        return;
    }

    void yellowCallback(const sensor_msgs::LaserScan &input)
    {
        float temp_min = MAX_VALUE;
        int data_no = -1;

        for(int i = 0 ; i < input.ranges.size() ; i++)
        {
            if(input.ranges[i] <  temp_min)
            {
                data_no = i;
                temp_min = input.ranges[i];
            }
        }

        if(data_no != -1)
        {
            yellow_distance = temp_min;
            yellow_angle = input.angle_min + data_no * input.angle_increment;
        }
        else
        {
            yellow_distance = MAX_VALUE;
            yellow_angle = -1;
        }
        return;
    }

private:
    ros::Subscriber red_laser_sub;
    ros::Subscriber yellow_laser_sub;
    ros::Subscriber filtered_laser_sub;

    ros::Publisher twist_pub_;

    sensor_msgs::LaserScan filtered_input;
    sensor_msgs::LaserScan red_input;
    sensor_msgs::LaserScan yellow_input;

    float filtered_angle;
    float red_angle;
    float yellow_angle;
    float filtered_distance;
    float red_distance;
    float yellow_distance;


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robo_ctrl");
    RoboCtrl robo_ctrl;
    ros::Rate r(100);
    while (ros::ok())
    {
        robo_ctrl.consider();
        ros::spinOnce();
        r.sleep();
    }
}
