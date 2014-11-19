/*
 * mip.h
 *
 *  Created on: Nov 18, 2014
 *      Author: mahisorn
 */

#ifndef SOURCE_DIRECTORY__HG_3DM_GX4_INCLUDE_HG_3DM_GX4_MIP_H_
#define SOURCE_DIRECTORY__HG_3DM_GX4_INCLUDE_HG_3DM_GX4_MIP_H_

namespace hg_3dm_gx4
{
  //Command and data class
  static const uint8_t CMD_CLASS_BASE = 0x01;
  static const uint8_t CMD_CLASS_3DM = 0x0C;
  static const uint8_t CMD_CLASS_EF = 0x0D;
  static const uint8_t CMD_CLASS_SYSTEM = 0x7F;

  static const uint8_t DATA_CLASS_IMU = 0x80;
  static const uint8_t DATA_CLASS_GPS = 0x81;
  static const uint8_t DATA_CLASS_EF = 0x82;

  static const uint8_t SELECT_IMU = 0x01;
  static const uint8_t SELECT_GPS = 0x02;
  static const uint8_t SELECT_EF = 0x03;

  static const uint8_t FUNCTION_APPLY = 0x01;
  static const uint8_t FIELD_ACK_NACK = 0xF1;

  static const uint8_t DATA_STREAM_ENABLE = 0x01;
  static const uint8_t DATA_STREAM_DISABLE = 0x00;


  //Base commands
  static const uint8_t CMD_PING = 0x01;
  static const uint8_t CMD_IDLE = 0x02;
  static const uint8_t CMD_RESUME = 0x06;
  static const uint8_t CMD_GPS_TIME_UPDATE = 0x72;
  static const uint8_t CMD_GET_DEVICE_INFORMATION = 0x03;
  static const uint8_t CMD_GET_DEVICE_DESCRIPTION = 0x04;
  static const uint8_t CMD_BUILD_IN_TEST = 0x05;
  static const uint8_t CMD_RESET = 0x7E;

  //3DM commands
  static const uint8_t CMD_IMU_MESSAGE_FORMAT = 0x08;
  static const uint8_t CMD_UART_BAUD_RATE = 0x40;
  static const uint8_t CMD_ENABLE_DATA_STREAM = 0x11;

  //IMU Data field
  static const uint8_t FILED_IMU_SCALED_ACCELEROMETER = 0x04;
  static const uint8_t FILED_IMU_SCALED_GYRO = 0x05;
  static const uint8_t FILED_IMU_SCALED_MAGNETO = 0x06;
  static const uint8_t FILED_IMU_SCALED_PRESSURE = 0x17;
  static const uint8_t FILED_IMU_DELTA_THETA = 0x07;

  static const uint8_t FILED_IMU_DELTA_VELOCITY = 0x08;
  static const uint8_t FILED_IMU_GPS_CORRELATION_TIMESTAMP = 0x12;

  //GPS Data field
  static const uint8_t FILED_GPS_LLH_POSITION = 0x03;
  static const uint8_t FILED_GPS_ECEF_POSITION = 0x04;
  static const uint8_t FILED_GPS_NED_VELOCITY = 0x05;
  static const uint8_t FILED_GPS_ECEF_VELOCITY = 0x06;
  static const uint8_t FILED_GPS_DOP_DATA = 0x07;

  static const uint8_t FILED_GPS_UTC_TIME = 0x08;
  static const uint8_t FILED_GPS_TIME = 0x09;
  static const uint8_t FILED_GPS_CLOCK_INFORMATION = 0x0A;
  static const uint8_t FILED_GPS_FIX_INFORMATION = 0x0B;
  static const uint8_t FILED_GPS_SPACE_VEHICLE_INFORMATION = 0x0C;

  static const uint8_t FILED_GPS_HARDWARE_STATUS = 0x0D;
  static const uint8_t FILED_DGPS_INFORMATION = 0x0E;
  static const uint8_t FILED_DGPS_CHANNEL_STATUS = 0x0E;

  //EF Data field
  static const uint8_t FIELD_EF_FILTER_STATUS = 0x10;
  static const uint8_t FIELD_EF_GPS_TIMESTAMP = 0x11;
  static const uint8_t FIELD_EF_LLH_POSITION = 0x01;
  static const uint8_t FIELD_EF_NED_VELOCITY = 0x02;
  static const uint8_t FIELD_EF_ORIENTATION_QUATERNION = 0x03;

  static const uint8_t FIELD_EF_ORIENTATION_MATRIX = 0x04;
  static const uint8_t FIELD_EF_EF_ORIENTATION_EULER = 0x05;
  static const uint8_t FIELD_EF_GYRO_BIAS = 0x06;
  static const uint8_t FIELD_EF_ACCEL_BIAS = 0x07;
  static const uint8_t FIELD_EF_LLH_POSITION_UNCERTAINTY = 0x08;

  static const uint8_t FIELD_EF_NED_VELOCITY_UNCERTAINTY = 0x09;
  static const uint8_t FIELD_EF_ALTITUDE_UNCERTAINTY = 0x0A;
  static const uint8_t FIELD_EF_GYRO_BIAS_UNCERTAINTY = 0x0B;
  static const uint8_t FIELD_EF_ACCEL_BIAS_UNCERTAINTY = 0x0C;
  static const uint8_t FIELD_EF_LINEAER_ACCELERATION = 0x0D;

  static const uint8_t FIELD_EF_COMPENSATED_ACCELERATION = 0x1C;
  static const uint8_t FIELD_EF_COMPENSATED_ANGULAR_RATE = 0x0E;
  static const uint8_t FIELD_EF_WGS84_LOCAL_GRAVITY_MAGNITUDE = 0x0F;
  static const uint8_t FIELD_EF_ALTITUDE_UNCERTAINTY_QUATERNION_ELEMENT = 0x12;
  static const uint8_t FIELD_EF_GRAVITY_VECTOR = 0x13;

  static const uint8_t FIELD_EF_HEADING_UPDATE_SOURCE_STATE = 0x14;
  static const uint8_t FIELD_EF_MAGNETIC_MODEL_SOLUTION = 0x15;
  static const uint8_t FIELD_EF_GYRO_SCALE_FACTOR = 0x16;
  static const uint8_t FIELD_EF_ACCEL_SCALE_FACTOR = 0x17;
  static const uint8_t FIELD_EF_GYRO_SCALE_FACTOR_UNCERTAINTY = 0x18;

  static const uint8_t FIELD_EF_ACCEL_SCALE_FACTOR_UNCERTAINTY = 0x19;
  static const uint8_t FIELD_EF_STANDARD_ATMOSPHERE_MODEL = 0x20;
  static const uint8_t FIELD_EF_PRESSURE_ALTITUDE = 0x21;
  static const uint8_t FIELD_EF_GPS_ANTENNA_OFFSET_CORRECTION = 0x30;
  static const uint8_t FIELD_EF_GPS_ANTENNA_OFFSET_CORRECTION_UNCERTAINTY = 0x31;




struct MIP
{
  static const uint8_t HEADER_LENGTH = 4;
  static const uint8_t SYNC1 = 0x75;
  static const uint8_t SYNC2 = 0x65;
  static const uint8_t MAX_PAYLOAD = 255;


  union
  {
    struct
    {
      uint8_t sync1;
      uint8_t sync2;
    };
    uint16_t sync;
  };

  uint8_t descriptor;
  uint8_t length;
  std::vector<uint8_t> payload;

  union
  {
    struct
    {
      uint8_t crc1;
      uint8_t crc2;
    };
    uint16_t crc;
  };

  MIP(uint8_t desc = 0)
    : sync1(SYNC1), sync2(SYNC2), descriptor(desc), length(0), crc(0), payload(MAX_PAYLOAD, 0), field_length_position_(0), is_encoding_(false)
  {

  }

  void reset()
  {
    field_length_position_ = 0;
    is_encoding_ = false;
  }

  void beginField(uint8_t description)
  {
    assert(!is_encoding_);
    field_length_position_ = length;
    payload[field_length_position_ + 1] = description;
    length += 2;
    is_encoding_ = true;
  }

  template<typename T>
  void append(const T& t)
  {
    assert(is_encoding_);

    if(sizeof(t) == 1)
    {
      payload[length] = t;
      length++;
      return;
    }

    uint8_t* p = (uint8_t*)(&t);
#ifdef HOST_LITTLE_ENDIAN
    p += (sizeof(t) - 1);
#endif
    for(int i = 0; i < sizeof(t); i++)
    {
#ifdef HOST_LITTLE_ENDIAN
      payload[length + i] = *(p-i);
#else
      payload[length + i] = *(p+i);
#endif
    }
    length += sizeof(t);
  }

  void endField()
  {
    assert(is_encoding_);
    payload[field_length_position_] = length - field_length_position_;
    is_encoding_ = false;
  }

  int getFieldDescriptor() const
  {
    if (field_length_position_ > (payload.size() - 2))
    {
      return -1;
    }
    if (payload[field_length_position_] == 0)
    {
      return -1;  //  no field
    }
    return payload[field_length_position_ + 1]; //  descriptor after length
  }

  bool isFieldAckOrNack() const
  {
    const int desc = getFieldDescriptor();
    if (desc == static_cast<int>(FIELD_ACK_NACK))
    {
      return true;
    }
    return false;
  }

  int currentfieldLength() const
  {
    assert(field_length_position_ < payload.size());
    return payload[field_length_position_];
  }

  bool gotoField(uint8_t field)
  {
    for (int d; (d = getFieldDescriptor()) > 0; nextField())
    {
      if (d == static_cast<int>(field))
      {
        return true;
      }
    }
    return false;
  }

  void nextField()
  {
    field_length_position_ += payload[field_length_position_];
  }

  template <typename T>
  void extract(size_t count, T* output)
  {
    const size_t sz = sizeof(T)*count;
    assert(field_length_position_+2+sz <= payload.size());

    for(int i = 0; i < count; i++)
    {
      if(sizeof(T) == 1)
      {
        output[i] = payload[i];
      }
      else
      {
          uint8_t* p = (uint8_t*)(&output[i]);
#ifdef HOST_LITTLE_ENDIAN
          p += (sizeof(T) - 1);
#endif
          for (int j = 0; j < sizeof(T); j++)
          {
#ifdef HOST_LITTLE_ENDIAN
            *(p - j) = payload[(field_length_position_+2) + (i*sizeof(T)) + j];
#else
            *(p + j) = payload[(field_length_position_+2) + (i*sizeof(T)) + j];
#endif
          }
      }
    }
  }

  void updateCheckSum()
  {
    uint8_t byte1 = 0, byte2 = 0;

#define add_byte(x) \
    byte1 += (x); \
    byte2 += byte1; \

    add_byte(sync1);
    add_byte(sync2);
    add_byte(descriptor);
    add_byte(length);

    for(int i = 0; i < length; i++)
    {
      add_byte(payload[i]);
    }
#undef add_byte

    crc = (static_cast<uint16_t>(byte1 << 8) + static_cast<uint16_t>(byte2));

#ifdef HOST_LITTLE_ENDIAN
    uint8_t temp = crc1;
    crc1 = crc2;
    crc2 = temp;
#endif
  }

std::string toString() const {
    std::stringstream ss;
    ss << std::hex;
    ss << "\n======= Packet =======\n";
    ss << "Sync MSB: " << static_cast<int>(sync1) << "\n";
    ss << "Sync LSB: " << static_cast<int>(sync2) << "\n";
    ss << "Descriptor: " << static_cast<int>(descriptor) << "\n";
    ss << "Length: " << static_cast<int>(length) << "\n";
    ss << "Payload: ";
    for (size_t s=0; s < length; s++) {
      ss << static_cast<int>(payload[s]) << " ";
    }
    ss << "\nCRC MSB: " << static_cast<int>(crc1) << "\n";
    ss << "CRC LSB: " << static_cast<int>(crc2) << "\n";
    ss << "======================";
    return ss.str();
  }

private:
  uint8_t field_length_position_;
  bool is_encoding_;
};

};




#endif /* SOURCE_DIRECTORY__HG_3DM_GX4_INCLUDE_HG_3DM_GX4_MIP_H_ */
