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

static const unsigned int COMMAND_RW_TIMEOUT = 100;

class CommandError : public std::runtime_error
{
public:
  CommandError(const MIP &p, uint8_t code)
    : std::runtime_error(generateString(p, code))
  {

  }

private:
  std::string generateString(const MIP &p, uint8_t code)
  {
    std::stringstream ss;
    ss << "Received NACK with error code " << std::hex << static_cast<int>(code);
    ss << ". Command Packet:\n" << p.toString();
    return ss.str();
  }
};

struct IMUData
{
  enum
  {
    SCALED_ACCELEROMETER = (1 << 0),
    SCALED_GYRO = (1 << 1),
    SCALED_MAGNETO = (1 << 2),
    SCALED_PRESSURE = (1 << 3),
    DELTA_THETA = (1 << 4),
    DELTA_VELOCITY = (1 << 5),
    GPS_CORRELATION_TIMESTAMP = (1 << 6),
    NUM_IMU_DATA = 7
  };
};

struct EFData
{
  enum
  {
    FILTER_STATUS = (1 << 0),
    GPS_TIMESTAMP = (1 << 1),
    LLH_POSITION = (1 << 2),
    NED_VELOCITY = (1 << 3),
    ORIENTATION_QUATERNION = (1 << 4),

    ORIENTATION_MATRIX = (1 << 5),
    EF_ORIENTATION_EULER = (1 << 6),
    GYRO_BIAS = (1 << 7),
    ACCEL_BIAS = (1 << 8),
    LLH_POSITION_UNCERTAINTY = (1 << 9),

    NED_VELOCITY_UNCERTAINTY = (1 << 10),
    ALTITUDE_UNCERTAINTY = (1 << 11),
    GYRO_BIAS_UNCERTAINTY = (1 << 12),
    ACCEL_BIAS_UNCERTAINTY = (1 << 13),
    LINEAER_ACCELERATION = (1 << 14),

    COMPENSATED_ACCELERATION = (1 << 15),
    COMPENSATED_ANGULAR_RATE = (1 << 16),
    WGS84_LOCAL_GRAVITY_MAGNITUDE = (1 << 17),
    ALTITUDE_UNCERTAINTY_QUATERNION_ELEMENT = (1 << 18),
    GRAVITY_VECTOR = (1 << 19),

    HEADING_UPDATE_SOURCE_STATE = (1 << 20),
    MAGNETIC_MODEL_SOLUTION = (1 << 21),
    GYRO_SCALE_FACTOR = (1 << 22),
    ACCEL_SCALE_FACTOR = (1 << 23),
    GYRO_SCALE_FACTOR_UNCERTAINTY = (1 << 24),

    ACCEL_SCALE_FACTOR_UNCERTAINTY = (1 << 25),
    STANDARD_ATMOSPHERE_MODEL = (1 << 26),
    PRESSURE_ALTITUDE = (1 << 27),
    GPS_ANTENNA_OFFSET_CORRECTION = (1 << 28),
    GPS_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY = (1 << 29)
  };
};

struct DataStream
{
  enum
  {
    IMU_DATA = (1 << 0),
    GPS_DATA = (1 << 1),
    EF_DATA = (1 << 2)
  };
};

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
  void setGPSDataRate(unsigned int decimation, const std::bitset<13>& sources);
  void setEFDataRate(unsigned int decimation, const std::bitset<7>& sources);

  void selectDataStream(const std::bitset<3>& streams);

  const MIP& receivedPacket() { return received_packet_; }

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
