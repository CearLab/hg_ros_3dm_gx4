/*
 * test.cpp
 *
 *  Created on: Nov 17, 2014
 *      Author: mahisorn
 */

#include <hg_3dm_gx4/3dm_gx4.h>

int main()
{
  hg_3dm_gx4::Hg3dmGx4 imu;
  //if(imu.openPort("/dev/pts/16", 115200))
  if(imu.openPort("/dev/ttyUSB0", 115200))
  //if(imu.openPort("/dev/ttyS0", 115200))
  {
    imu.ping();
    imu.selectBaudRate(460800);
    /*
    sleep(1);
    imu.idle();
    sleep(1);
    imu.ping();
    sleep(1);
    imu.resume();
    sleep(1);
    imu.ping();
    sleep(1);
    imu.reset();
    sleep(5);
    imu.ping();
    sleep(1);
    */
  }
  else
  {
    std::cout << "Shit!\n";
  }
  return 0;
}


