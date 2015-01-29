/*
 * KVH 1750 IMU
 * Eric L. Hahn <erichahn@vt.edu>
 * 12/5/2014
 * Copyright 2014. All Rights Reserved.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Imu.h>

#pragma GCC diagnostic pop

#include <fstream>
#include <algorithm>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <byteswap.h>

namespace
{
  const size_t HeaderSize = 4;
  const char Header[] = {0xFE, 0x81, 0xFF, 0x55};
  const int TempWarn = 75;
}

struct KVH1750Message
{
  char Header[4];
  int32_t rots[3];
  int32_t accels[3];
  uint8_t status;
  uint8_t seq;
  uint16_t temp;
  uint32_t crc;
};

/**
 * Extracts values from byte array to arrays of floats. Does not modify
 * the input buffer, and performs full conversion of values.
 */
bool extractValues(const std::vector<char>& buffer,
  double** ang_vel, double** lin_accel, double& temp)
{
  const size_t Num_Values_Per_Array = 3;

  const KVH1750Message* message = reinterpret_cast<const KVH1750Message*>(&buffer.front());
  for(size_t ii = 0; ii < Num_Values_Per_Array; ++ii)
  {
    const double Gravity = 9.80665;
    int32_t ang = bswap_32(message->rots[ii]);
    int32_t lin = bswap_32(message->accels[ii]);
    *ang_vel[ii] = *reinterpret_cast<float*>(&ang);
    *lin_accel[ii] = Gravity * *reinterpret_cast<float*>(&lin);
  }

  //TODO: add support for above 100c
  temp =  bswap_16(message->temp);

  if(temp > TempWarn)
  {
    ROS_FATAL("!!!WARNING WARNING WARNING!!! IMU TOO HOT, TURN OFF ROBOT NOW!");
  }

  return true;
}

int main(int argc, char **argv)
{
  //Name of node
  ros::init(argc, argv, "kvh_1750_imu");
  //Node handle
  ros::NodeHandle nh("~");
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 1);
  ros::Publisher temp_pub = nh.advertise<sensor_msgs::Temperature>("temp", 1);
  std::string imu_link_name = "torso";
  nh.getParam("imu_link_name", imu_link_name);

  const double DefaultRate_Hz = 1000.0;
  double rate_hz = DefaultRate_Hz;

  nh.getParam("rate_hz", rate_hz);

  if (rate_hz <= 0.0)
  {
    ROS_WARN("Invalid cycle rate. Defaulting to %f", DefaultRate_Hz);
    rate_hz = DefaultRate_Hz;
  }

  sensor_msgs::Imu current_imu;
  sensor_msgs::Temperature kvh_temp;

  ros::Rate loop_rate(rate_hz);
  // ROS_INFO("OUTPUT 1");
  int fd = open("/dev/ttyS4", O_RDONLY);
  //Static Settings
  current_imu.angular_velocity_covariance[0] = -1;
  current_imu.angular_velocity_covariance[4] = -1;
  current_imu.angular_velocity_covariance[8] = -1;
  current_imu.linear_acceleration_covariance[0] = -1;
  current_imu.linear_acceleration_covariance[4] = -1;
  current_imu.linear_acceleration_covariance[8] = -1;
  kvh_temp.header.frame_id = imu_link_name;
  current_imu.header.frame_id = imu_link_name;

  const size_t MessageSize = 36;
  std::vector<char> buffer(MessageSize, 0);
  size_t bytes_used = 0;
  while(ros::ok())
  {
    bool read_msg = false;
    while(ros::ok() && !read_msg)
    {
      ros::Time read_tm = ros::Time::now();
      int rc = read(fd, &buffer[bytes_used], MessageSize - bytes_used);
      if(rc < 0)
      {
        ROS_ERROR("Error reading file. %s", strerror(errno));
      }
      bytes_used += rc;
      std::vector<char>::iterator buffer_end = buffer.begin() + bytes_used;
      std::vector<char>::iterator match = std::search(buffer.begin(),
        buffer_end, Header, Header + HeaderSize);

      if(match == buffer.begin()) //process complete message
      {
        double* accels[] = {&current_imu.linear_acceleration.x,
          &current_imu.linear_acceleration.y, &current_imu.linear_acceleration.z};
        double* ang_vels[] = {&current_imu.angular_velocity.x,
          &current_imu.angular_velocity.y, &current_imu.angular_velocity.z};
        extractValues(buffer, ang_vels, accels, kvh_temp.temperature);

        kvh_temp.header.stamp = read_tm;
        current_imu.header.stamp = read_tm;
        read_msg = true;
        buffer.clear();
        buffer.resize(MessageSize, 0);
      }
      else
      {
        std::vector<char> new_buffer;
        //start at either the match, or the longest missable substring
        std::vector<char>::iterator start = std::min(match, std::max(buffer.begin(),
          buffer_end - (HeaderSize - 1)));
        new_buffer.insert(new_buffer.begin(), start, buffer_end);
        new_buffer.resize(MessageSize, 0);
        bytes_used = buffer_end - start;
        buffer.swap(new_buffer);
      }
    }

    imu_pub.publish(current_imu);
    temp_pub.publish(kvh_temp);
    ros::spinOnce();
    loop_rate.sleep();
  }

  close(fd);
}
