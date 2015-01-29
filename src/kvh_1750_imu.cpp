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

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <fstream>
#include <algorithm>
#include <string.h>

void bytesToFloat(char *inputBuffer, float *rotationOutput, float *accelerationOutput)
{
  for (int quadByteCounter = 0; quadByteCounter < 21; quadByteCounter += 4)
  {
    std::reverse(&inputBuffer[quadByteCounter], &inputBuffer[quadByteCounter + 4]);
  }

  memcpy(&rotationOutput[0], &inputBuffer[0], 12);
  memcpy(&accelerationOutput[0], &inputBuffer[12], 12);

}

int main(int argc, char **argv)
{
  //Name of node
  ros::init(argc, argv, "kvh_1750_imu");
  //Node handle
  ros::NodeHandle kvh1750imuNodeHandle;
  ros::Publisher kvh1750_IMU = kvh1750imuNodeHandle.advertise<sensor_msgs::Imu>("kvh1750imu_message", 1);
  ros::Publisher kvh1750_TEMP = kvh1750imuNodeHandle.advertise<sensor_msgs::Temperature>("kvh1750temp_message", 1);

  sensor_msgs::Imu KVHImu;
  sensor_msgs::Temperature KVHTemp;

  char byte[36];
  char header;
  float accel[3];
  float rotat[3];

  const float GRAVITY_CONSTANT = 9.80665;
  const char HEADER_1 = 0xFE;
  const char HEADER_2 = 0x81;
  const char HEADER_3 = 0xFF;
  const char HEADER_4 = 0x55;
  const int TEMP_WARN = 75;

  double rate_hz = 1000.0;

  kvh1750imuNodeHandle.getParam("rate_hz", rate_hz);

  if (rate_hz <= 0.0)
  {
    ROS_WARN("Invalid cycle rate. Defaulting to 10.0");
    rate_hz = 10.0;
  }

  ros::Rate loop_rate(rate_hz);
  // ROS_INFO("OUTPUT 1");
  int fd = open("/dev/ttyS4", O_RDWR);
  //Static Settings
  KVHImu.angular_velocity_covariance[0] = -1;
  KVHImu.angular_velocity_covariance[4] = -1;
  KVHImu.angular_velocity_covariance[8] = -1;
  KVHImu.linear_acceleration_covariance[0] = -1;
  KVHImu.linear_acceleration_covariance[4] = -1;
  KVHImu.linear_acceleration_covariance[8] = -1;
  KVHTemp.header.frame_id = "torso";
  KVHImu.header.frame_id = "torso";


  while (ros::ok())
  {
    ros::Time read_tm = ros::Time::now();
    read(fd, &header, 1);
    while (header != HEADER_1)
    {
      read(fd, &header, 1);
    }
    if (header == HEADER_1)
    {
      read(fd, &header, 1);
      if (header == HEADER_2)
      {
        read(fd, &header, 1);
        if (header == HEADER_3)
        {
          read(fd, &header, 1);
          if (header == HEADER_4)
          {
            read(fd, &byte, 32);

            bytesToFloat(byte, rotat, accel);
            accel[0] *= GRAVITY_CONSTANT;
            accel[1] *= GRAVITY_CONSTANT;
            accel[2] *= GRAVITY_CONSTANT;

            KVHImu.linear_acceleration.x = accel[0];
            KVHImu.linear_acceleration.y = accel[1];
            KVHImu.linear_acceleration.z = accel[2];

            KVHImu.angular_velocity.x = rotat[0];
            KVHImu.angular_velocity.y = rotat[1];
            KVHImu.angular_velocity.z = rotat[2];

            //TODO: add support for above 100c
            KVHTemp.temperature =  byte[27] ;
            if (byte[27] > TEMP_WARN)
            {
              ROS_FATAL("\n  !!!WARNING WARNING WARNING!!! \n IMU TOO HOT, TURN OFF ROBOT NOW!");
            }
            KVHTemp.header.stamp = read_tm;
            KVHImu.header.stamp = read_tm;
          }
        }
      }
    }

    kvh1750_IMU.publish(KVHImu);
    kvh1750_TEMP.publish(KVHTemp);
    ros::spinOnce();
    loop_rate.sleep();
  }

}
