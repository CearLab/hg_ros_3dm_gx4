/*
 * 3dm_gx4_node.cpp
 *
 *  Created on: Nov 21, 2014
 *      Author: mahisorn
 */
#include <limits>
#include <time.h>

#include <ros/ros.h>
#include <hg_ros_3dm_gx4/3dm_gx4.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

using namespace std;

ros::Publisher g_pub_raw_imu;
ros::Publisher g_pub_raw_mag;
ros::Publisher g_pub_navsat_fix;
ros::Publisher g_pub_navsat_vel;
ros::Publisher g_pub_navsat_tf;
ros::Publisher g_pub_filtered_imu;
ros::Publisher g_pub_filtered_fix;
ros::Publisher g_pub_filtered_vel;

std::string gps_frame_id;
std::string imu_frame_id;

void publishIMUData(const hg_3dm_gx4::IMUData& data)
{
  sensor_msgs::Imu g_raw_imu;
  sensor_msgs::MagneticField g_raw_mag;

  g_raw_imu.header.stamp = ros::Time::now();
  g_raw_imu.header.frame_id = imu_frame_id;
  g_raw_mag.header.stamp = g_raw_imu.header.stamp;
  g_raw_mag.header.frame_id = imu_frame_id;

  // In world frame.
  g_raw_imu.orientation.w =  data.orientation_quaternion[3];
  g_raw_imu.orientation.x =  data.orientation_quaternion[1];
  g_raw_imu.orientation.y =  data.orientation_quaternion[0];
  g_raw_imu.orientation.z = -data.orientation_quaternion[2];

  g_raw_mag.magnetic_field.x =  data.scaled_magneto[1] * hg_3dm_gx4::Hg3dmGx4::GAUSS_TO_TESLA;
  g_raw_mag.magnetic_field.y =  data.scaled_magneto[0] * hg_3dm_gx4::Hg3dmGx4::GAUSS_TO_TESLA;
  g_raw_mag.magnetic_field.z = -data.scaled_magneto[2] * hg_3dm_gx4::Hg3dmGx4::GAUSS_TO_TESLA;

  // In body-fixed frame.
  g_raw_imu.linear_acceleration.x =  data.scaled_accelerometer[0] * hg_3dm_gx4::Hg3dmGx4::G_TO_ACCELERATION;
  g_raw_imu.linear_acceleration.y = -data.scaled_accelerometer[1] * hg_3dm_gx4::Hg3dmGx4::G_TO_ACCELERATION;
  g_raw_imu.linear_acceleration.z = -data.scaled_accelerometer[2] * hg_3dm_gx4::Hg3dmGx4::G_TO_ACCELERATION;
  g_raw_imu.angular_velocity.x =  data.scaled_gyro[0];
  g_raw_imu.angular_velocity.y = -data.scaled_gyro[1];
  g_raw_imu.angular_velocity.z = -data.scaled_gyro[2];

  g_pub_raw_imu.publish(g_raw_imu);
  g_pub_raw_mag.publish(g_raw_mag);
}

void publishEFData(const hg_3dm_gx4::EFData& data)
{

  sensor_msgs::Imu g_filtered_imu;
  sensor_msgs::NavSatFix g_filtered_fix;
  geometry_msgs::TwistWithCovarianceStamped g_filtered_vel;

  g_filtered_imu.header.frame_id = imu_frame_id;
  g_filtered_fix.header.frame_id = gps_frame_id;
  g_filtered_vel.header.frame_id = gps_frame_id;

  if (data.status < 0x02)
  {
    ROS_WARN_THROTTLE(5, "Filter not running with status flag: %x04.", data.status_flag);
  }
  else
  {
    g_filtered_imu.header.stamp = ros::Time::now();

    g_filtered_imu.orientation.w =  data.orientation_quaternion[3];
    g_filtered_imu.orientation.x =  data.orientation_quaternion[1];
    g_filtered_imu.orientation.y =  data.orientation_quaternion[0];
    g_filtered_imu.orientation.z = -data.orientation_quaternion[2];
    g_filtered_imu.linear_acceleration.x =  data.compensated_acceleration[0];
    g_filtered_imu.linear_acceleration.y = -data.compensated_acceleration[1];
    g_filtered_imu.linear_acceleration.z = -data.compensated_acceleration[2];
    g_filtered_imu.angular_velocity.x =  data.compensated_angular_rate[0];
    g_filtered_imu.angular_velocity.y = -data.compensated_angular_rate[1];
    g_filtered_imu.angular_velocity.z = -data.compensated_angular_rate[2];


    tf::Quaternion q(data.orientation_uncertainty[1],
                   data.orientation_uncertainty[2],
                   data.orientation_uncertainty[3],
                   data.orientation_uncertainty[0]);

    tf::Matrix3x3 m(q);
    m.getRPY(g_filtered_imu.orientation_covariance[4],
      g_filtered_imu.orientation_covariance[0], g_filtered_imu.orientation_covariance[8]);

    g_filtered_imu.orientation_covariance[8] *= -1;

    g_filtered_imu.linear_acceleration_covariance[0] =
      data.uncertainty_acceleration[0] * data.uncertainty_acceleration[0];
    g_filtered_imu.linear_acceleration_covariance[4] =
      data.uncertainty_acceleration[1] * data.uncertainty_acceleration[1];
    g_filtered_imu.linear_acceleration_covariance[8] =
      data.uncertainty_acceleration[2] * data.uncertainty_acceleration[2];

    g_filtered_imu.angular_velocity_covariance[0] =
      data.uncertainty_angular_rate[0] * data.uncertainty_angular_rate[0];
    g_filtered_imu.angular_velocity_covariance[4] =
      data.uncertainty_angular_rate[1] * data.uncertainty_angular_rate[1];
    g_filtered_imu.angular_velocity_covariance[8] =
      data.uncertainty_angular_rate[2] * data.uncertainty_angular_rate[2];

    g_filtered_fix.header.stamp = g_filtered_imu.header.stamp;
    g_filtered_fix.latitude = data.llh[0];
    g_filtered_fix.longitude = data.llh[1];
    g_filtered_fix.altitude = data.llh[2];
    g_filtered_fix.position_covariance[0] = data.llh_uncertainty[0] * data.llh_uncertainty[0];
    g_filtered_fix.position_covariance[4] = data.llh_uncertainty[1] * data.llh_uncertainty[1];
    g_filtered_fix.position_covariance[8] = data.llh_uncertainty[2] * data.llh_uncertainty[2];
    g_filtered_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;


    g_filtered_vel.header.stamp = g_filtered_imu.header.stamp;
    g_filtered_vel.twist.twist.linear.x = data.vel[0];
    g_filtered_vel.twist.twist.linear.y = data.vel[1];
    g_filtered_vel.twist.twist.linear.z = data.vel[2];
    g_filtered_vel.twist.covariance[0] = data.vel_uncertainty[0];
    g_filtered_vel.twist.covariance[4] = data.vel_uncertainty[1];
    g_filtered_vel.twist.covariance[8] = data.vel_uncertainty[3];

    g_pub_filtered_imu.publish(g_filtered_imu);
    g_pub_filtered_fix.publish(g_filtered_fix);
    g_pub_filtered_vel.publish(g_filtered_vel);

    if (data.status == 0x03)
    {
      ROS_WARN_THROTTLE(5, "Filter running but solution is not valid with status flag %x04.", data.status_flag);
    }
    else
    {
      ROS_DEBUG("Filter running with valid solution");
    }
  }
}

void publishGPSData(const hg_3dm_gx4::GPSData& data)
{
  sensor_msgs::NavSatFix g_navsat_fix;
  geometry_msgs::TwistStamped g_navsat_vel;
  sensor_msgs::TimeReference g_navsat_tf;

  g_navsat_fix.header.stamp = ros::Time::now();
  g_navsat_vel.header.stamp = g_navsat_fix.header.stamp;
  g_navsat_tf.header.stamp = g_navsat_fix.header.stamp;

  g_navsat_fix.header.frame_id = gps_frame_id;
  g_navsat_vel.header.frame_id = gps_frame_id;

  if (data.status == 0x04)  // Invalid
  {
    ROS_WARN_THROTTLE(30, "Invalid fix.");
    return;
  }
  else if (data.status == 0x03)  // None
  {
    g_navsat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
  }
  else if (data.status == 0x02)  // Time only
  {
    g_navsat_tf.time_ref.sec = data.posix_time;
  }
  else
  {
    if (data.status_flag == 0x0001)
    {
      g_navsat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
    }
    else if (data.status_flag == 0x0002)
    {
      g_navsat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    }
    else
    {
      g_navsat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    }

    g_navsat_tf.time_ref.sec = data.posix_time;

    g_navsat_fix.latitude = data.llh[0];
    g_navsat_fix.longitude = data.llh[1];
    g_navsat_fix.position_covariance[0] = data.llh_accuracy[0];
    g_navsat_fix.position_covariance[4] = data.llh_accuracy[1];

    if (data.status == 0x00)  // 3D fix
    {
      g_navsat_fix.altitude = data.llh[2];
      g_navsat_fix.position_covariance[8] = -1; // Fix this.
    }
    else  // 2D fix
    {
      g_navsat_fix.altitude =  std::numeric_limits<double>::quiet_NaN();
      g_navsat_fix.position_covariance[8] =  std::numeric_limits<double>::quiet_NaN();
    }
    g_navsat_fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    g_navsat_vel.twist.linear.x = data.vel[0];
    g_navsat_vel.twist.linear.y = data.vel[1];
    g_navsat_vel.twist.linear.z = data.vel[2];
  }


  if (data.status < 0x04)
  {
    g_pub_navsat_fix.publish(g_navsat_fix);
  }
  if (data.status < 0x03)
  {
    g_pub_navsat_tf.publish(g_navsat_tf);
  }
  if (data.status < 0x02)
  {
    g_pub_navsat_vel.publish(g_navsat_vel);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hg_3dm_gx4");
  ros::NodeHandle nh("~");

  hg_3dm_gx4::Hg3dmGx4 imu;

  std::string imu_prefix;
  std::string gps_prefix;
  std::string filter_prefix;
  std::string device;
  int baudrate;
  int imu_rate;
  int gps_rate;

  nh.param<std::string>("imu_prefix", imu_prefix, "imu");
  nh.param<std::string>("gps_prefix", gps_prefix, "navsat");
  nh.param<std::string>("filter_prefix", filter_prefix, "filter");
  nh.param<std::string>("device", device,"/dev/ttyACM0");
  nh.param<std::string>("imu_frame_id", imu_frame_id, "imu_link");
  nh.param<std::string>("gps_frame_id", gps_frame_id, "gps_link");
  nh.param<int>("baudrate", baudrate, 460800);
  nh.param<int>("imu_rate", imu_rate, 100);
  nh.param<int>("gps_rate", gps_rate, 4);

  if(!imu.openPort(device, baudrate))
  {
    ROS_ERROR_STREAM("Cannot open : " << device);
    return -1;
  }

  imu.ping();
  imu.idle();
  imu.selectBaudRate(baudrate);

  if(imu_rate > 500)
  {
    ROS_WARN("Max frequency is 500!");
    imu_rate = 500;
  }
  else if(imu_rate <= 0)
  {
    ROS_WARN("Min frequency should be 1! 100 Hz will be set");
    imu_rate = 100;
  }

  if(gps_rate > 4)
  {
    ROS_WARN("Max frequency is 4!");
    gps_rate = 4;
  }
  else if(gps_rate <= 0)
  {
    ROS_WARN("Min frequency should be 1! 1 Hz will be set");
    gps_rate = 1;
  }

  static const int imu_decimation = (500/imu_rate);
  static const int gps_decimation = (4/gps_rate);

  imu.setIMUDataRate(imu_decimation, //(500 / 1) for 3DM-GX4-45
                     hg_3dm_gx4::IMUData::SCALED_ACCELEROMETER |
                     hg_3dm_gx4::IMUData::SCALED_GYRO |
                     hg_3dm_gx4::IMUData::SCALED_MAGNETO |
                     hg_3dm_gx4::IMUData::CF_QUATERNION |
                     0);


  imu.setEFDataRate(imu_decimation,
                    hg_3dm_gx4::EFData::FILTER_STATUS |
                    hg_3dm_gx4::EFData::LLH_POSITION |
                    hg_3dm_gx4::EFData::LLH_POSITION_UNCERTAINTY |
                    hg_3dm_gx4::EFData::NED_VELOCITY |
                    hg_3dm_gx4::EFData::NED_VELOCITY_UNCERTAINTY |
                    hg_3dm_gx4::EFData::ORIENTATION_QUATERNION |
                    hg_3dm_gx4::EFData::COMPENSATED_ACCELERATION |
                    hg_3dm_gx4::EFData::COMPENSATED_ANGULAR_RATE |
                    hg_3dm_gx4::EFData::GYRO_BIAS_UNCERTAINTY |
                    hg_3dm_gx4::EFData::ACCEL_BIAS_UNCERTAINTY |
                    hg_3dm_gx4::EFData::ALTITUDE_UNCERTAINTY_QUATERNION_ELEMENT |
                    0);

  imu.setGPSDataRate(gps_decimation,
                     hg_3dm_gx4::GPSData::FIX_INFORMATION |
                     hg_3dm_gx4::GPSData::LLH_POSITION |
                     hg_3dm_gx4::GPSData::NED_VELOCITY |
                     hg_3dm_gx4::GPSData::UTC_TIME |
                     0);

  imu.selectDataStream(
                       hg_3dm_gx4::DataStream::IMU_DATA |
                       hg_3dm_gx4::DataStream::EF_DATA |
                       hg_3dm_gx4::DataStream::GPS_DATA |
                       0);

  imu.resume();

  imu.initializeFilterWithMagneto();

  imu.setInitialAttitude(0, 0, 0);

  g_pub_raw_imu = nh.advertise<sensor_msgs::Imu>(imu_prefix + "/data", 1);
  g_pub_raw_mag = nh.advertise<sensor_msgs::MagneticField>(imu_prefix + "/mag", 1);
  g_pub_navsat_fix = nh.advertise<sensor_msgs::NavSatFix>(gps_prefix + "/fix", 1);
  g_pub_navsat_vel = nh.advertise<geometry_msgs::TwistStamped>(gps_prefix + "/vel", 1);
  g_pub_navsat_tf = nh.advertise<sensor_msgs::TimeReference>(gps_prefix + "/time_reference", 1);
  g_pub_filtered_imu = nh.advertise<sensor_msgs::Imu>(filter_prefix + "/imu/data", 1);
  g_pub_filtered_fix = nh.advertise<sensor_msgs::NavSatFix>(filter_prefix + "/fix", 1);
  g_pub_filtered_vel = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(filter_prefix + "/vel", 1);

  imu.setIMUDataCallback(publishIMUData);
  imu.setEFDataCallback(publishEFData);
  imu.setGPSDataCallback(publishGPSData);

  while(ros::ok())
  {
    imu.receiveDataStream();
  }

}
