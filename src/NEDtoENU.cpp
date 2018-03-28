#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>


sensor_msgs::Imu imu_ENU, imu_NED;
geometry_msgs::PoseWithCovarianceStamped svo_pose;
nav_msgs::Odometry filter_pose;

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
  imu_NED = *msg;
}

void svo_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  svo_pose = *msg;
}

void filter_cb(const nav_msgs::Odometry::ConstPtr& msg){
  filter_pose = *msg;
}

void NEDtoENU(const sensor_msgs::Imu NED, sensor_msgs::Imu *imu_ENU){
  imu_ENU->linear_acceleration.x = NED.linear_acceleration.y;
  imu_ENU->linear_acceleration.y = NED.linear_acceleration.x;
  imu_ENU->linear_acceleration.z = -NED.linear_acceleration.z;

  imu_ENU->angular_velocity.x = NED.angular_velocity.y;
  imu_ENU->angular_velocity.y = NED.angular_velocity.x;
  imu_ENU->angular_velocity.z = -NED.angular_velocity.z;

  imu_ENU->orientation.x = NED.orientation.y;
  imu_ENU->orientation.y = NED.orientation.x;
  imu_ENU->orientation.z = -NED.orientation.z;
  imu_ENU->orientation.w = NED.orientation.w;

  imu_ENU->orientation_covariance = imu_NED.orientation_covariance;
  imu_ENU->angular_velocity_covariance = imu_NED.angular_velocity_covariance;
  imu_ENU->linear_acceleration_covariance = imu_NED.linear_acceleration_covariance;



}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "NEDtoENU");
  ros::NodeHandle nh;

  /* The topic which is needed to be subscribed depends on what imu you use, here we use the imu on Pixhawk. */

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, imu_cb);
  ros::Subscriber svo_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/svo/pose_imu", 10, svo_cb);
  ros::Subscriber filter_sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, filter_cb);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/enu",10);

  ros::Rate rate(50);

  imu_ENU.header.frame_id = "base_link";


  while (ros::ok()) {
   imu_ENU.header.stamp = ros::Time::now();
   NEDtoENU(imu_NED, &imu_ENU);
   imu_pub.publish(imu_ENU);
   ROS_INFO("SVO:    x = %f, y = %f, z = %f",svo_pose.pose.pose.position.x, svo_pose.pose.pose.position.y, svo_pose.pose.pose.position.z);
   ROS_INFO("Filter: x = %f, y = %f, z = %f", filter_pose.pose.pose.position.x, filter_pose.pose.pose.position.y, filter_pose.pose.pose.position.z);
   ros::spinOnce();
   rate.sleep();
  }
  return 0;


}
