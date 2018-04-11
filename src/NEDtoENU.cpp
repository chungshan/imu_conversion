#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>

sensor_msgs::Imu imu_ENU, imu_NED;
geometry_msgs::PoseWithCovarianceStamped svo_pose;
nav_msgs::Odometry filter_pose;
geometry_msgs::TwistWithCovarianceStamped svo_twist;

typedef struct pose_old{
  float x;
  float y;
  float z;
}pose_old;
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
pose_old svo_pose_old;
void svo_poseTotwist(const geometry_msgs::PoseWithCovarianceStamped pose, geometry_msgs::TwistWithCovarianceStamped *twist, const sensor_msgs::Imu imu){

  float dt = 0.02;
  float vx,vy,vz;
  tf::Quaternion quat(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
  twist->twist.covariance = pose.pose.covariance;


  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  geometry_msgs::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;
  //ROS_INFO("roll = %f",roll);
  Eigen::Matrix3f Rx, Ry, Rz;

  Rx(0,0) = 1;
  Rx(1,0) = 0;
  Rx(2,0) = 0;
  Rx(0,1) = 0;
  Rx(1,1) = cos(roll);
  Rx(1,2) = -sin(roll);
  Rx(2,0) = 0;
  Rx(2,1) = sin(roll);
  Rx(2,2) = cos(roll);

  Ry(0,0) = cos(pitch);
  Ry(1,0) = 0;
  Ry(2,0) = sin(pitch);
  Ry(0,1) = 0;
  Ry(1,1) = 1;
  Ry(1,2) = 0;
  Ry(2,0) = -sin(pitch);
  Ry(2,1) = 0;
  Ry(2,2) = cos(pitch);

  Rz(0,0) = cos(yaw);
  Rz(1,0) = -sin(yaw);
  Rz(2,0) = 0;
  Rz(0,1) = sin(yaw);
  Rz(1,1) = cos(yaw);
  Rz(1,2) = 0;
  Rz(2,0) = 0;
  Rz(2,1) = 0;
  Rz(2,2) = 1;

  vx = (pose.pose.pose.position.x - svo_pose_old.x)/dt;
  vy = (pose.pose.pose.position.y - svo_pose_old.y)/dt;
  vz = (pose.pose.pose.position.z - svo_pose_old.z)/dt;
  svo_pose_old.x = pose.pose.pose.position.x;
  svo_pose_old.y = pose.pose.pose.position.y;
  svo_pose_old.z = pose.pose.pose.position.z;

  Eigen::Vector3f v, v_after;

  v(0) = vx;
  v(1) = vy;
  v(2) = vz;

  v_after = Rx*Ry*Rz*v;
  twist->twist.twist.linear.x = v_after(0);
  twist->twist.twist.linear.y = v_after(1);
  twist->twist.twist.linear.z = v_after(2);
  ROS_INFO("SVO_Vx:x = %f, y = %f, z = %f.",twist->twist.twist.linear.x,twist->twist.twist.linear.y, twist->twist.twist.linear.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "NEDtoENU");
  ros::NodeHandle nh;

  /* The topic which is needed to be subscribed depends on what imu you use, here we use the imu on Pixhawk. */

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/drone2/mavros/imu/data", 10, imu_cb);
  ros::Subscriber svo_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/svo/pose_imu", 10, svo_cb);
  ros::Subscriber filter_sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, filter_cb);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/enu",10);
  ros::Publisher svo_twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/svo/twist", 10);
  ros::Publisher svo_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/svo/pose_imu2", 10);
  ros::Rate rate(50);
  svo_pose_old.x = 0;
  svo_pose_old.y = 0;
  svo_pose_old.z = 0;

  imu_ENU.header.frame_id = "base_link";
  svo_twist.header.frame_id = "base_link";
  svo_pose.header.frame_id = "odom";




  while (ros::ok()) {
   imu_ENU.header.stamp = ros::Time::now();
   svo_twist.header.stamp = ros::Time::now();
   NEDtoENU(imu_NED, &imu_ENU);
   imu_pub.publish(imu_ENU);
   svo_poseTotwist(svo_pose, &svo_twist, imu_NED);
   svo_twist_pub.publish(svo_twist);
   svo_pub.publish(svo_pose);
   ROS_INFO("SVO:    x = %f, y = %f, z = %f",svo_pose.pose.pose.position.x, svo_pose.pose.pose.position.y, svo_pose.pose.pose.position.z);
   ROS_INFO("Filter: x = %f, y = %f, z = %f", filter_pose.pose.pose.position.x, filter_pose.pose.pose.position.y, filter_pose.pose.pose.position.z);
   ros::spinOnce();
   rate.sleep();

  }
  return 0;


}
