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

  static const uint8_t FUNCTION_APPLY = 0x01;

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
  static const uint8_t CMD_UART_BAUD_RATE = 0x40;

  static const uint8_t FIELD_ACK_NACK = 0xF1;

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
    : sync1(SYNC1), sync2(SYNC2), descriptor(desc), length(0), crc(0), payload(MAX_PAYLOAD, 0)
  {

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
    ss << "Sync MSB: " << static_cast<int>(sync1) << "\n";
    ss << "Sync LSB: " << static_cast<int>(sync2) << "\n";
    ss << "Descriptor: " << static_cast<int>(descriptor) << "\n";
    ss << "Length: " << static_cast<int>(length) << "\n";
    ss << "Payload: ";
    for (size_t s=0; s < length; s++) {
      ss << static_cast<int>(payload[s]) << " ";
    }
    ss << "\nCRC MSB: " << static_cast<int>(crc1) << "\n";
    ss << "CRC LSB: " << static_cast<int>(crc2);
    return ss.str();
  }
};




};




#endif /* SOURCE_DIRECTORY__HG_3DM_GX4_INCLUDE_HG_3DM_GX4_MIP_H_ */
