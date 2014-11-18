/*
 * 3dm_gx4.h
 *
 *  Created on: Nov 17, 2014
 *      Author: mahisorn
 */

#ifndef SOURCE_DIRECTORY__HG_3DM_GX4_INCLUDE_HG_3DM_GX4_3DM_GX4_H_
#define SOURCE_DIRECTORY__HG_3DM_GX4_INCLUDE_HG_3DM_GX4_3DM_GX4_H_

#include <hg_ros_serial/serial.h>
#include <hg_3dm_gx4/mip.h>

namespace hg_3dm_gx4
{

class Hg3dmGx4 : public hg_ros_serial::Serial
{
public:
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

  void selectBaudRate(unsigned int baud);

protected:
  void sendPacket(const MIP& p, int timeout);
  void sendAndReceivePacket(const MIP& p);
  void processPacket();
  void processIMUPacket();
  void processGPSPacket();
  void processEFPacket();
  void processRespondPacket();


  MIP received_packet_;

};


}


#endif /* SOURCE_DIRECTORY__HG_3DM_GX4_INCLUDE_HG_3DM_GX4_3DM_GX4_H_ */
