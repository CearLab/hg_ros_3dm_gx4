/*
 * 3dm_gx4.cpp
 *
 *  Created on: Nov 17, 2014
 *      Author: mahisorn
 */
#include <hg_3dm_gx4/3dm_gx4.h>
#include <boost/asio.hpp>
#include <boost/chrono.hpp>

#define u8(x) static_cast<uint8_t>((x))
#define u16(x) static_cast<uint16_t>((x))

using namespace hg_3dm_gx4;

void Hg3dmGx4::ping()
{
  MIP p(CMD_CLASS_BASE);
  p.length = 2;
  p.payload[0] = 2;
  p.payload[1] = CMD_PING;
  p.updateCheckSum();

  sendAndReceivePacket(p);
}

void Hg3dmGx4::idle()
{
  MIP p(CMD_CLASS_BASE);
  p.length = 2;
  p.payload[0] = 2;
  p.payload[1] = CMD_IDLE;
  p.updateCheckSum();

  sendAndReceivePacket(p);
}

void Hg3dmGx4::resume()
{
  MIP p(CMD_CLASS_BASE);
  p.length = 2;
  p.payload[0] = 2;
  p.payload[1] = CMD_RESUME;
  p.updateCheckSum();

  sendAndReceivePacket(p);
}

void Hg3dmGx4::reset()
{
  MIP p(CMD_CLASS_BASE);
  p.length = 2;
  p.payload[0] = 2;
  p.payload[1] = CMD_RESET;
  p.updateCheckSum();

  sendAndReceivePacket(p);
}

void Hg3dmGx4::selectBaudRate(unsigned int baud)
{
  //search for device
  static const size_t num_rates = 6;
  static const unsigned int rates[num_rates] = {9600, 19200, 115200, 230400, 460800, 921600};

  bool found_rate = false;
  int i;
  for(i = 0; i < num_rates; i++)
  {
    std::cout << "Try: " << rates[i] << std::endl;
    changeBaudRate(rates[i]);
    try
    {
      ping();
    }
    catch (std::exception& e)
    {
      std::cout << e.what() << std::endl;
      continue;
    }
    break;
  }

  //Change baudrate

  MIP p(CMD_CLASS_3DM);
  p.length = 7;
  p.payload[0] = 7;
  p.payload[1] = CMD_UART_BAUD_RATE;
  p.payload[2] = FUNCTION_APPLY;
  p.payload[3] = (baud >> 24) & 0xff;
  p.payload[4] = (baud >> 16) & 0xff;
  p.payload[5] = (baud >> 8) & 0xff;
  p.payload[6] = baud & 0xff ;
  p.updateCheckSum();

  std::cout << p.toString() << std::endl;

  try
  {
    sendAndReceivePacket(p);
    sleep(1);
    ping();
  }
  catch (std::exception& e)
  {
    std::cout << e.what() << std::endl;
  }
}

void Hg3dmGx4::setIMUDataRate(unsigned int decimation, const std::bitset<7>& sources)
{
  static const uint8_t data_fields[] =
  {
    FILED_IMU_SCALED_ACCELEROMETER,
    FILED_IMU_SCALED_GYRO,
    FILED_IMU_SCALED_MAGNETO,
    FILED_IMU_SCALED_PRESSURE,
    FILED_IMU_DELTA_THETA,
    FILED_IMU_DELTA_VELOCITY,
    FILED_IMU_GPS_CORRELATION_TIMESTAMP
  };

  assert(sizeof(data_fields) == sources.size());

  std::vector<uint8_t> fields;

  for(int i = 0; i < sources.size(); i++)
  {
    if(sources[i])
    {
      fields.push_back(data_fields[i]);
    }
  }

  MIP p(CMD_CLASS_3DM);
  p.beginField(CMD_IMU_MESSAGE_FORMAT);
  p.append(FUNCTION_APPLY);
  p.append(u8(fields.size()));

  for(int i = 0; i < fields.size(); i++)
  {
    p.append(fields[i]);
    p.append(u16(decimation));
  }

  p.endField();
  p.updateCheckSum();

  std::cout << p.toString() << std::endl;

  sendAndReceivePacket(p);
}

void Hg3dmGx4::setEFDataRate(unsigned int decimation, const std::bitset<7>& sources)
{
  static const uint8_t data_fields[] =
  {
    FILED_IMU_SCALED_ACCELEROMETER,
    FILED_IMU_SCALED_GYRO,
    FILED_IMU_SCALED_MAGNETO,
    FILED_IMU_SCALED_PRESSURE,
    FILED_IMU_DELTA_THETA,
    FILED_IMU_DELTA_VELOCITY,
    FILED_IMU_GPS_CORRELATION_TIMESTAMP
  };

  assert(sizeof(data_fields) == sources.size());

  std::vector<uint8_t> fields;

  for(int i = 0; i < sources.size(); i++)
  {
    if(sources[i])
    {
      fields.push_back(data_fields[i]);
    }
  }

  MIP p(CMD_CLASS_3DM);
  p.beginField(CMD_IMU_MESSAGE_FORMAT);
  p.append(FUNCTION_APPLY);
  p.append(u8(fields.size()));

  for(int i = 0; i < fields.size(); i++)
  {
    p.append(fields[i]);
    p.append(u16(decimation));
  }

  p.endField();
  p.updateCheckSum();

  std::cout << p.toString() << std::endl;

  sendAndReceivePacket(p);
}

void Hg3dmGx4::selectDataStream(const std::bitset<3>& streams)
{
  static const uint8_t selection[] =
  {
    SELECT_IMU,
    SELECT_GPS,
    SELECT_EF,
  };

  assert(sizeof(selection) == streams.size());

  MIP p(CMD_CLASS_3DM);

  for(int i = 0; i < streams.size(); i++)
  {
    p.beginField(CMD_ENABLE_DATA_STREAM);
    p.append(FUNCTION_APPLY);
    p.append(selection[i]);
    p.append(streams[i]);
    p.endField();
  }


  p.updateCheckSum();
  std::cout << p.toString() << std::endl;

  sendAndReceivePacket(p);
  std::cout << std::dec;
  std::cout << received_packet_.length << std::endl;
  std::cout << std::hex;
  std::cout << received_packet_.getFieldDescriptor() << std::endl;
  p.nextField();
  std::cout << received_packet_.getFieldDescriptor() << std::endl;
  p.nextField();
  std::cout << received_packet_.getFieldDescriptor() << std::endl;


}

void Hg3dmGx4::sendPacket(const MIP& p, int timeout)
{
  using namespace boost::chrono;

  Serial::Data buffer;
  buffer.reserve(MIP::HEADER_LENGTH + MIP::MAX_PAYLOAD + 2);

  buffer.push_back(p.sync1);
  buffer.push_back(p.sync2);
  buffer.push_back(p.descriptor);
  buffer.push_back(p.length);
  for (size_t i = 0; i < p.length; i++) {
    buffer.push_back(p.payload[i]);
  }
  buffer.push_back(p.crc1);
  buffer.push_back(p.crc2);

  try
  {
    writeData(buffer, timeout);
  }
  catch (std::exception &e)
  {
    std::cout << e.what() << std::endl;
  }
}



void Hg3dmGx4::sendAndReceivePacket(const MIP& p)
{
  using namespace boost::chrono;

  sendPacket(p, COMMAND_RW_TIMEOUT);

  static Serial::Data buffer(MIP::HEADER_LENGTH + MIP::MAX_PAYLOAD + 2);

  high_resolution_clock::time_point tstart = high_resolution_clock::now();
  high_resolution_clock::time_point tstop = tstart + milliseconds(300);

  while(true)
  {
    if (tstop < high_resolution_clock::now())
    {
      throw std::runtime_error("Got no respond");
    }

    try
    {
      //Get header
      asyncReadBlockOfData(buffer, 1, COMMAND_RW_TIMEOUT);
      if(buffer[0] != MIP::SYNC1)
        continue;

      asyncReadBlockOfData(buffer, 1, COMMAND_RW_TIMEOUT);
      if (buffer[0] != MIP::SYNC2)
        continue;

      //Get descriptor and length
      asyncReadBlockOfData(buffer, 2, COMMAND_RW_TIMEOUT);
      received_packet_.descriptor = buffer[0];
      received_packet_.length = buffer[1];

      //Get payload
      asyncReadBlockOfData(received_packet_.payload, received_packet_.length, COMMAND_RW_TIMEOUT);

      //Get checksum
      asyncReadBlockOfData(buffer, 2, COMMAND_RW_TIMEOUT);

      received_packet_.updateCheckSum();

      if(received_packet_.crc1 != buffer[0] || received_packet_.crc2 != buffer[1])
      {
        std::cout << "Warning: Dropped packet with mismatched checksum\n" << std::endl;
      }
      else
      {
        processPacket();
        return;
      }
    }
    catch (std::exception& e)
    {
      std::cout << e.what() << std::endl;
    }
  }
}

void Hg3dmGx4::receiveDataStream()
{
  //using namespace boost::chrono;

  static Serial::Data buffer(MIP::HEADER_LENGTH + MIP::MAX_PAYLOAD + 2);

  is_running_  = true;

  while (is_running_)
  {
    try
    {
      //Get header
      asyncReadBlockOfData(buffer, 1, COMMAND_RW_TIMEOUT);
      if (buffer[0] != MIP::SYNC1)
        continue;

      asyncReadBlockOfData(buffer, 1, COMMAND_RW_TIMEOUT);
      if (buffer[0] != MIP::SYNC2)
        continue;

      //Get descriptor and length
      asyncReadBlockOfData(buffer, 2, COMMAND_RW_TIMEOUT);
      received_packet_.descriptor = buffer[0];
      received_packet_.length = buffer[1];

      //Get payload
      asyncReadBlockOfData(received_packet_.payload, received_packet_.length, COMMAND_RW_TIMEOUT);

      //Get checksum
      asyncReadBlockOfData(buffer, 2, COMMAND_RW_TIMEOUT);

      received_packet_.updateCheckSum();

      if (received_packet_.crc1 != buffer[0] || received_packet_.crc2 != buffer[1])
      {
        std::cout << "Warning: Dropped packet with mismatched checksum\n" << std::endl;
      }
      else
      {
        processPacket();
        //return;
      }
    }
    catch (std::exception& e)
    {
      std::cout << e.what() << std::endl;
      is_running_ = false;
    }
    usleep(1000);
  }

}

void Hg3dmGx4::processPacket()
{
  switch(received_packet_.descriptor)
  {
    case DATA_CLASS_IMU: processIMUPacket(); break;
    case DATA_CLASS_GPS: processGPSPacket(); break;
    case DATA_CLASS_EF: processEFPacket(); break;
    default: processRespondPacket(); break;
  }
  received_packet_.reset();
}

void Hg3dmGx4::processIMUPacket()
{
  float data[10];
  while(true)
  {
    switch(received_packet_.getFieldDescriptor())
    {
      case FILED_IMU_SCALED_ACCELEROMETER:
        received_packet_.extract(3, data);
        printf("acc: %8.3f %8.3f %8.3f\n", data[0], data[1], data[2]);
        break;
      case FILED_IMU_SCALED_GYRO:
        received_packet_.extract(3, data);
        printf("gyr: %8.3f %8.3f %8.3f\n", data[0], data[1], data[2]);
        break;
      case FILED_IMU_SCALED_MAGNETO:
        received_packet_.extract(3, data);
        printf("mag: %8.3f %8.3f %8.3f\n", data[0], data[1], data[2]);
        break;
      case FILED_IMU_SCALED_PRESSURE: break;
      case FILED_IMU_DELTA_THETA: break;
      case FILED_IMU_DELTA_VELOCITY: break;
      case FILED_IMU_GPS_CORRELATION_TIMESTAMP: break;
      default:
        return;
    }
    received_packet_.nextField();
  }
}

void Hg3dmGx4::processGPSPacket()
{
  std::cout << __FUNCTION__ << std::endl;
}

void Hg3dmGx4::processEFPacket()
{
  std::cout << __FUNCTION__ << std::endl;
}

void Hg3dmGx4::processRespondPacket()
{
  //Find ACK/NACK
  if(received_packet_.payload[1] == FIELD_ACK_NACK)
  {
    if(received_packet_.payload[3] == 0x00)
    {
      //Got ACK
      std::cout << "Found ACK field" << std::endl;
    }
    else
    {
      //Got NACK
      std::cout << "Received NACK packet (class, command, code): ";
      std::cout << std::hex << static_cast<int>(received_packet_.descriptor) << ", ";
      std::cout << static_cast<int>(received_packet_.payload[2]) << ", ";
      std::cout << static_cast<int>(received_packet_.payload[3]) << std::endl;
    }
  }
}
