/*
Yuki Imai
*/

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

ros::Publisher filtered_points_pub;
ros::Publisher red_points_pub;
ros::Publisher yellow_points_pub;
ros::Publisher origin_pub;

static std::chrono::time_point<std::chrono::system_clock> filter_start, filter_end;
static bool _output_log = false;
static std::ofstream ofs;
static std::string filename;
static std::string POINTS_TOPIC;

static void scan_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZRGB> tmp;
    pcl::PointCloud<pcl::PointXYZRGB> filtered_output;
    pcl::PointCloud<pcl::PointXYZRGB> red_output;
    pcl::PointCloud<pcl::PointXYZRGB> yellow_output;

    pcl::fromROSMsg(*input, tmp);

    filter_start = std::chrono::system_clock::now();
    pcl::PointXYZRGB p;

    for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
    {
        if(item->r > 50 && item->g < 50 && item->b < 50)
        {
            //RED
            p.x = item->x;
            p.y = item->y;
            p.z = item->z;
            p.r = item->r;
            p.g = item->g;
            p.b = item->b;
            red_output.points.push_back(p);
        }
        else if(item->r > 50 && item->g > 50 && item->b < 50)
        {
            //YELLOW
            p.x = item->x;
            p.y = item->y;
            p.z = item->z;
            p.r = item->r;
            p.g = item->g;
            p.b = item->b;
            yellow_output.points.push_back(p);
        }
        else
        {
            p.x = item->x;
            p.y = item->y;
            p.z = item->z;
            p.r = item->r;
            p.g = item->g;
            p.b = item->b;
            filtered_output.points.push_back(p);
        }


    }

    //filtered
    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(filtered_output, filtered_msg);
    filtered_msg.header = input->header;
    filtered_points_pub.publish(filtered_msg);

    //red
    pcl::toROSMsg(red_output, filtered_msg);
    filtered_msg.header = input->header;
    red_points_pub.publish(filtered_msg);

    //yellow
    pcl::toROSMsg(yellow_output, filtered_msg);
    filtered_msg.header = input->header;
    yellow_points_pub.publish(filtered_msg);


    filter_end = std::chrono::system_clock::now();

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_cordinator");

    ros::NodeHandle nh;

    //private_nh.getParam("points_topic", POINTS_TOPIC);
    POINTS_TOPIC = "/camera/depth/points";

    // Publishers
    filtered_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);

    red_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/red_points", 10);
    yellow_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/yellow_points", 10);

    origin_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);

    geometry_msgs::PoseWithCovarianceStamped origin;
    origin.pose.pose.position.x = 90.0041;
    origin.pose.pose.position.y = 118.303;
    origin.pose.pose.position.z = 0.0;
    origin.header.frame_id = "map";
    origin_pub.publish(origin);

    // Subscribers
    ros::Subscriber scan_sub = nh.subscribe(POINTS_TOPIC, 10, scan_callback);

    ros::spin();

    return 0;
}
