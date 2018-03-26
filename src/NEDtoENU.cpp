#include <ros/ros.h>
#include <sensor_msgs/Imu.h>



sensor_msgs::Imu imu_ENU, imu_NED;


void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
  imu_NED = *msg;
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



}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "NEDtoENU");
  ros::NodeHandle nh;

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, imu_cb);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/enu",10);

  ros::Rate rate(50);

  imu_ENU.header.frame_id = "base_link";
  imu_ENU.orientation_covariance = {1 , 0 , 0 , 0 ,1 , 0 , 0 ,0 , 1};

  while (ros::ok()) {
   imu_ENU.header.stamp = ros::Time::now();
   NEDtoENU(imu_NED, &imu_ENU);
   imu_pub.publish(imu_ENU);
   ros::spinOnce();
   rate.sleep();
  }
  return 0;


}
