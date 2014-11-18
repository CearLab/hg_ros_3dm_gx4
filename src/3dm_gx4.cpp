/*
 * 3dm_gx4.cpp
 *
 *  Created on: Nov 17, 2014
 *      Author: mahisorn
 */
#include <hg_3dm_gx4/3dm_gx4.h>
#include <boost/asio.hpp>
#include <boost/chrono.hpp>

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
  static unsigned int rates[num_rates] = {9600, 19200, 115200, 230400, 460800, 921600};


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

  int wrote = writeData(buffer, timeout);
  if (wrote < 0)
  {
    throw std::runtime_error(strerror(errno));
  }
  else if (wrote == 0)
  {
    std::stringstream ss;
    ss << "Time out while writing [";
    ss << timeout << " ms]";
    throw std::runtime_error(ss.str());
  }
}



void Hg3dmGx4::sendAndReceivePacket(const MIP& p)
{
  using namespace boost::chrono;

  sendPacket(p, 100);

  //std::cout << "========== Sent ==========\n"
  //          << p.toString() << std::endl;

  static Serial::Data buffer(MIP::HEADER_LENGTH + MIP::MAX_PAYLOAD + 2);

  high_resolution_clock::time_point tstart = high_resolution_clock::now();
  high_resolution_clock::time_point tstop = tstart + milliseconds(300);

  while(true)
  {
    if (tstop < high_resolution_clock::now())
    {
      //std::cout << "Got no respond" << std::endl;
      throw std::runtime_error("Got no respond");
    }

    try
    {
      //Get header
      asyncReadBlockOfData(buffer, 1, 200);
      if(buffer[0] != MIP::SYNC1)
        continue;

      asyncReadBlockOfData(buffer, 1, 200);
      if (buffer[0] != MIP::SYNC2)
        continue;

      //Get descriptor and length
      asyncReadBlockOfData(buffer, 2, 100);
      received_packet_.descriptor = buffer[0];
      received_packet_.length = buffer[1];

      //Get payload
      asyncReadBlockOfData(received_packet_.payload, received_packet_.length, 100);

      //Get checksum
      asyncReadBlockOfData(buffer, 2, 100);

      received_packet_.updateCheckSum();

      if(received_packet_.crc1 != buffer[0] || received_packet_.crc2 != buffer[1])
      {
        //std::cout << "Check sum error" << std::endl;
        throw std::runtime_error("Check sum error");
      }
      else
      {
        //Process
        //std::cout << "========== Received ==========\n";
        //std::cout << received_packet_.toString() << std::endl;
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

void Hg3dmGx4::processPacket()
{
  switch(received_packet_.descriptor)
  {
    case DATA_CLASS_IMU: processIMUPacket(); break;
    case DATA_CLASS_GPS: processGPSPacket(); break;
    case DATA_CLASS_EF: processEFPacket(); break;
    default: processRespondPacket(); break;
  }
}

void Hg3dmGx4::processIMUPacket()
{

}

void Hg3dmGx4::processGPSPacket()
{

}

void Hg3dmGx4::processEFPacket()
{

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
