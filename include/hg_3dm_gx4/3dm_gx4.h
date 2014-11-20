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
#include <bitset>

namespace hg_3dm_gx4
{

static const unsigned int COMMAND_RW_TIMEOUT = 2000;

class Hg3dmGx4 : public hg_ros_serial::Serial
{
public:
  Hg3dmGx4()
    : is_running_(false)
  {

  }

  virtual ~Hg3dmGx4()
  {

  }

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

  void setIMUDataRate(unsigned int decimation, const std::bitset<IMUData::NUM_IMU_DATA>& sources);
  void setGPSDataRate(unsigned int decimation, const std::bitset<GPSData::NUM_GPS_DATA>& sources);
  void setEFDataRate(unsigned int decimation, const std::bitset<EFData::NUM_EF_DATA>& sources);

  void selectDataStream(const std::bitset<3>& streams);

  void initializeFilterWithMagneto();
  void setInitialAttitude(float roll, float pitch, float yaw);
  void setInitialHeading(float heading);

  //const MIP& receivedPacket() { return received_packet_; }

  void receiveDataStream();

protected:
  void sendPacket(const MIP& p, int timeout);
  void sendAndReceivePacket(const MIP& p);



  void processPacket();
  void processIMUPacket();
  void processGPSPacket();
  void processEFPacket();
  void processRespondPacket();




  MIP received_packet_;
  bool is_running_;

};


}


#endif /* SOURCE_DIRECTORY__HG_3DM_GX4_INCLUDE_HG_3DM_GX4_3DM_GX4_H_ */
