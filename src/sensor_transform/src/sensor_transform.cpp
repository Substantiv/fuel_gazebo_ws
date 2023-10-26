#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry odom_;

void rcvOdometryCallbck(const nav_msgs::Odometry& odom)
{
  odom_ = odom;
}

int main(int argc, char *argv[])
{

    // 编码,初始化,NodeHandle
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "tfs");
    ros::NodeHandle nh;

    // 创建订阅对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener sub(buffer);
    ros::Subscriber odom_sub;
    odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

    // 创建发布对象
    ros::Publisher depth_camera_pose_pub;
    depth_camera_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("depth_camera_pose", 10);;
    

    ros::Rate rate(10);
    while (ros::ok())
    {
        try
        {
            // 计算son1和son2的相对关系
            geometry_msgs::TransformStamped sensor2world = buffer.lookupTransform("world", "depth_camera_front_link", ros::Time());

            geometry_msgs::PoseStamped sensor_pose;
            sensor_pose.header = odom_.header;
            sensor_pose.header.frame_id = "world";
            sensor_pose.pose.position.x = sensor2world.transform.translation.x;
            sensor_pose.pose.position.y = sensor2world.transform.translation.y;
            sensor_pose.pose.position.z = sensor2world.transform.translation.z;
            sensor_pose.pose.orientation.w = sensor2world.transform.rotation.w;
            sensor_pose.pose.orientation.x = sensor2world.transform.rotation.x;
            sensor_pose.pose.orientation.y = sensor2world.transform.rotation.y;
            sensor_pose.pose.orientation.z = sensor2world.transform.rotation.z;
            depth_camera_pose_pub.publish(sensor_pose);
        }
        catch(const std::exception& e)
        {
            ROS_INFO(e.what());
        }
        
        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}