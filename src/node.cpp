/*
 * KVH 1750 IMU
 * Eric L. Hahn <erichahn@vt.edu>
 * 12/5/2014
 * Copyright 2014. All Rights Reserved.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include <ros/ros.h>
#pragma GCC diagnostic pop

#include "kvh1750/imu.h"
#include <trooper_mlc_msgs/CachedRawIMUData.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Imu.h>

namespace
{
  const std::string DefaultImuLink = "torso";
  const std::string DefaultAddress = "/dev/ttyS4";
  const size_t ImuCacheSize = 15;
}

/**
 * Converts KVH1750Message into the standard ROS messages corresponding
 * to the same set of data, namely an Imu and Temperature message.
 */
void to_ros(const kvh::Message& msg, sensor_msgs::Imu& imu,
  sensor_msgs::Temperature& temp)
{
  msg.time(imu.header.stamp.sec, imu.header.stamp.nsec);

  imu.angular_velocity.x = msg.gyro_x();
  imu.angular_velocity.y = msg.gyro_y();
  imu.angular_velocity.z = msg.gyro_z();
  imu.linear_acceleration.x = msg.accel_x();
  imu.linear_acceleration.y = msg.accel_y();
  imu.linear_acceleration.z = msg.accel_z();

  temp.header.stamp = imu.header.stamp;
  temp.temperature = msg.temp();
}

/**
 * Adds a single IMU reading to the cached value. If the cache is full, this
 * resets the counter and returns true.
 */
bool cache_imu(const kvh::Message& msg, trooper_mlc_msgs::CachedRawIMUData& cache,
  size_t& counter)
{
  if(counter >= ImuCacheSize)
  {
    counter = 0;
    return false;
  }

  trooper_mlc_msgs::RawIMUData& imu = cache.data[counter];
  uint32_t secs = 0;
  uint32_t nsecs = 0;
  msg.time(secs, nsecs);
  imu.imu_timestamp = static_cast<uint64_t>(secs * 1.0E-6) +
  static_cast<uint64_t>(nsecs + 1.0E3);
  imu.packet_count = msg.sequence_number();
  imu.dax = msg.gyro_x();
  imu.day = msg.gyro_y();
  imu.daz = msg.gyro_z();
  imu.ddx = msg.accel_x();
  imu.ddy = msg.accel_y();
  imu.ddz = msg.accel_z();

  //if the pre-increment sets it to 15, will be set to 0 and return true
  return (++counter % ImuCacheSize) == 0;
}

int main(int argc, char **argv)
{
  //Name of node
  ros::init(argc, argv, "kvh_1750_imu");
  //Node handle
  ros::NodeHandle nh("~");
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1);
  ros::Publisher temp_pub = nh.advertise<sensor_msgs::Temperature>("temp", 1);
  ros::Publisher cache_pub = nh.advertise<trooper_mlc_msgs::CachedRawIMUData>("cached", 1);
  std::string imu_link_name = DefaultImuLink;
  nh.getParam("imu_link_name", imu_link_name);

  size_t cache_counter = 0;
  trooper_mlc_msgs::CachedRawIMUData cached_imu;
  sensor_msgs::Imu current_imu;
  sensor_msgs::Temperature current_temp;

  //Static Settings
  current_imu.angular_velocity_covariance[0] = -1;
  current_imu.angular_velocity_covariance[4] = -1;
  current_imu.angular_velocity_covariance[8] = -1;
  current_imu.linear_acceleration_covariance[0] = -1;
  current_imu.linear_acceleration_covariance[4] = -1;
  current_imu.linear_acceleration_covariance[8] = -1;
  //IMU link locations
  current_temp.header.frame_id = imu_link_name;
  current_imu.header.frame_id = imu_link_name;
  cached_imu.header.frame_id = imu_link_name;

  std::string addr = DefaultAddress;
  nh.getParam("imu_address", addr);
  std::shared_ptr<trec::KVHBase> kvh = std::shared_ptr<trec::KVHBase>(new trec::KVH1750(addr));

  bool keep_reading = true;
  while(ros::ok() && keep_reading)
  {
    kvh::Message msg;
    switch(kvh->read(msg))
    {
      case trec::VALID:
        to_ros(msg, current_imu, current_temp);
        if(cache_imu(msg, cached_imu, cache_counter))
        {
          cache_pub.publish(cached_imu);
        }
        imu_pub.publish(current_imu);
        temp_pub.publish(current_temp);
        break;
      case trec::BAD_SIZE:
      case trec::BAD_READ:
      case trec::BAD_CRC:
        ROS_ERROR("Bad data from KVH, ignoring.");
        break;
      case trec::FATAL_ERROR:
        ROS_FATAL("Lost connection to IMU!"); //should reconnect
        keep_reading = false;
        break;
      case trec::OVER_TEMP:
        ROS_FATAL("IMU is overheating!");
        keep_reading = false;
        break;
      case trec::PARTIAL_READ:
      default:
      break;
    }
    ros::spinOnce();
  }

  return 0;
}