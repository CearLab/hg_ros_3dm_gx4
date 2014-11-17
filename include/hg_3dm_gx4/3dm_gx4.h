/*
 * 3dm_gx4.h
 *
 *  Created on: Nov 17, 2014
 *      Author: mahisorn
 */

#ifndef SOURCE_DIRECTORY__HG_3DM_GX4_INCLUDE_HG_3DM_GX4_3DM_GX4_H_
#define SOURCE_DIRECTORY__HG_3DM_GX4_INCLUDE_HG_3DM_GX4_3DM_GX4_H_

#include <hg_ros_serial/serial.h>

namespace hg_3dm_gx4
{

class Hg3dmGx4 : public hg_ros_serial::Serial
{
public:

  //Code from imu_3dm_gx4 package
  static const uint8_t COMMAND_CLASS_BASE = 0x01;
  static const uint8_t COMMAND_CLASS_3DM = 0x0C;
  static const uint8_t COMMAND_CLASS_FILTER = 0x0D;
  static const uint8_t COMMAND_CLASS_SYSTEM = 0x7F;

  static const uint8_t DATA_CLASS_IMU = 0x80;
  static const uint8_t DATA_CLASS_GPS = 0x81;
  static const uint8_t DATA_CLASS_FILTER = 0x82;

  //  base commands
  static const uint8_t DEVICE_PING = 0x01;
  static const uint8_t DEVICE_IDLE = 0x02;
  static const uint8_t DEVICE_RESUME = 0x06;


  static const uint8_t FIELD_ACK_OR_NACK = 0xF1;

  Hg3dmGx4() { }
  virtual ~Hg3dmGx4() { }

//Base commands
  void ping();
  void idle();
  void resume();
  void updateGpsTime();
  void getDeviceInformation();
  void getDeviceDescriptorSets();
  bool runDeviceBuildInTest();
  void reset();

protected:


};


}


#endif /* SOURCE_DIRECTORY__HG_3DM_GX4_INCLUDE_HG_3DM_GX4_3DM_GX4_H_ */
