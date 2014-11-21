/*
 * 3dm_gx4_node.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: mahisorn
 */

#include <ros/ros.h>
#include <hg_3dm_gx4/3dm_gx4.h>
#include <sensor_msgs/Imu.h>

using namespace std;

ros::Publisher g_pub_imu;
ros::Publisher g_pub_ef_imu;

void publishIMUData(const hg_3dm_gx4::IMUData& data)
{
  //cout << __FUNCTION__ << endl;
  sensor_msgs::Imu imu;

  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = "imu";

  imu.linear_acceleration.x = data.scaled_accelerometer[0];
  imu.linear_acceleration.y = data.scaled_accelerometer[1];
  imu.linear_acceleration.z = data.scaled_accelerometer[2];
  imu.angular_velocity.x = data.scaled_gyro[0];
  imu.angular_velocity.y = data.scaled_gyro[1];
  imu.angular_velocity.z = data.scaled_gyro[2];

  g_pub_imu.publish(imu);
}

void publishEFData(const hg_3dm_gx4::EFData& data)
{
  cout << __FUNCTION__ << endl;
  sensor_msgs::Imu imu;

  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = "imu";

  imu.linear_acceleration.x = data.compensated_acceleration[0];
  imu.linear_acceleration.y = data.compensated_acceleration[1];
  imu.linear_acceleration.z = data.compensated_acceleration[2];
  imu.angular_velocity.x = data.compensated_angular_rate[0];
  imu.angular_velocity.y = data.compensated_angular_rate[1];
  imu.angular_velocity.z = data.compensated_angular_rate[2];

  g_pub_imu.publish(imu);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hg_3dm_gx4");
  ros::NodeHandle nh("~");

  hg_3dm_gx4::Hg3dmGx4 imu;




  if(!imu.openPort("/dev/ttyACM0", 115200))
  {
    ROS_ERROR("Cannot open device");
    return -1;
  }

  imu.ping();

  imu.idle();

  imu.selectBaudRate(460800);

  static const int decimation = (500/500);

  imu.setIMUDataRate(decimation, //(500 / 1) for 3DM-GX4-45
                     hg_3dm_gx4::IMUData::SCALED_ACCELEROMETER |
                     hg_3dm_gx4::IMUData::SCALED_GYRO |
                     hg_3dm_gx4::IMUData::SCALED_MAGNETO |
                     0);


  imu.setEFDataRate(decimation,
                    //hg_3dm_gx4::EFData::ORIENTATION_EULER |
                    hg_3dm_gx4::EFData::GRAVITY_VECTOR |
                    //hg_3dm_gx4::EFData::FILTER_STATUS |
                    //hg_3dm_gx4::EFData::COMPENSATED_ACCELERATION |
                    hg_3dm_gx4::EFData::COMPENSATED_ANGULAR_RATE |
                    0);

  imu.setGPSDataRate(1,
                     hg_3dm_gx4::GPSData::FIX_INFORMATION |
                     0);

  imu.selectDataStream(
                       //hg_3dm_gx4::DataStream::IMU_DATA |
                       hg_3dm_gx4::DataStream::EF_DATA |
                       //hg_3dm_gx4::DataStream::GPS_DATA |
                       0);

  imu.resume();

  //imu.initializeFilterWithMagneto();

  imu.setInitialAttitude(0, 0, 0);

  //imu.setIMUDataCallback(publishIMUData);
  imu.setEFDataCallback(publishEFData);


  g_pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 1);




  while(ros::ok())
  {
    imu.receiveDataStream();
  }



}



