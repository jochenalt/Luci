/*
 * I2CInterface.cpp
 *
 *  Created on: 14.06.2015
 *      Author: JochenAlt
 */

#include <iostream>
#include "string.h"

#include "I2CInterface.h"
#include "setup.h"
#include "errno.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include "unistd.h"
#include <linux/i2c.h>
#include <linux/i2c-dev.h>


bool    setSlave(int i2cFD,unsigned int i2cDevAddress )
{
    if( ::ioctl(i2cFD, I2C_SLAVE, i2cDevAddress) < 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}
bool useSmbusIOCTL(int i2cFD, I2CInterface::Direction rwMode, uint8_t registerAddr, I2CInterface::TransactionType smbusTransaction, i2c_smbus_data &data)
{
       if( rwMode == I2CInterface::bothDirection ) { return false; }

       i2c_smbus_ioctl_data smbusPackage;

       smbusPackage.read_write = (rwMode == I2CInterface::input ) ? I2C_SMBUS_READ : I2C_SMBUS_WRITE;
       smbusPackage.command    = registerAddr;
       smbusPackage.size       = smbusTransaction;
       smbusPackage.data       = &data;


       if( ::ioctl(i2cFD, I2C_SMBUS, &smbusPackage) < 0 )
       {
           return false;
       }
       else
       {
           return true;
       }
}

void I2CInterface::setup(std::string i2cPath /* e.g. /dev/i2c-0 */, unsigned int i2cDeviceAddress) {
	this->i2cPortPath   = i2cPath;
	this->i2cDevAddress = i2cDeviceAddress;
	this->i2cFD         = -1;
	this->isOpenFlag    = false;
}

bool I2CInterface::open(OpenMode openMode) {
    uint flags = 0;

    if( (openMode & ReadOnly)   == ReadOnly     ){  flags |= O_RDONLY;  }
    if( (openMode & WriteOnly)  == WriteOnly    ){  flags |= O_WRONLY;  }
    if( (openMode & ReadWrite)  == ReadWrite    ){  flags |= O_RDWR;    }
    if( (openMode & Append)     == Append       ){  flags |= O_APPEND;  }
    if( (openMode & Truncate)   == Truncate     ){  flags |= O_TRUNC;   }
    if( (openMode & NonBlock)   == NonBlock     ){  flags |= O_NONBLOCK;}


    this->i2cFD = ::open(this->i2cPortPath.c_str(), flags);


     if( this->i2cFD < 0 )
     {
         this->isOpenFlag = false;
         std::cerr << "I2CInterface::open err " << errno << std::endl;
         return false;
     }
     else
     {
         this->isOpenFlag = true;
         if( ::ioctl(i2cFD, I2C_SLAVE, i2cDevAddress) < 0) {
             std::cerr << "I2CInterface::open.setSlave failed errno=" << errno << std::endl;
         }


         return true;
     }
}

bool I2CInterface::writeBlock(uint8_t registerAddr, uint8_t *writeBuffer, size_t bufferSize) {
	if (!isOpenFlag)
		std::cerr << "ERR:I2CInterface FD not open" << std::endl;

	if( bufferSize > 32 )
        bufferSize = 32;

	i2c_smbus_data writeFromThis;

	memset(writeFromThis.block, 0, bufferSize+1);
	memcpy( &(writeFromThis.block[1]), writeBuffer, bufferSize);
	writeFromThis.block[0] = bufferSize;

	if( useSmbusIOCTL(i2cFD, output, registerAddr, SMBUS_I2C_BLOCK_DATA, writeFromThis) )
	{
        return true;
	}
	else
	{
		std::cerr << "ERR I2CInterface::writeBlock" << std::endl;
		return false;
	}
}

bool I2CInterface::writeByte(uint8_t registerAddr, uint8_t value) {
	if (!isOpenFlag)
		std::cerr << "ERR:I2CInterface FD not open" << std::endl;

	i2c_smbus_data writeFromThis;
    writeFromThis.byte = value;

    if( useSmbusIOCTL(i2cFD, output, registerAddr, SMBUS_BYTE_DATA, writeFromThis) )
    {
        return true;
    }
    else
    {
		std::cerr << "ERR:I2CInterface.writeByte" << std::endl;
        return false;
    }
}

bool I2CInterface::readLine(uint8_t *readBuffer, size_t bufferSize) {
	if (!isOpenFlag)
		std::cerr << "ERR:I2CInterface FD not open" << std::endl;

	if( ::read(i2cFD, readBuffer, bufferSize) < 0 )
    {
		std::cerr << "ERR:I2CInterface.readLine" << std::endl;
        return false;
    }
    else
    {
        return true;
    }
}

bool I2CInterface::close() {
    if( ::close(this->i2cFD) < 0 )
    {
		std::cerr << "ERR:I2CInterface.close" << std::endl;
        return false;
    }
    else
    {
        this->isOpenFlag = false;
        return true;
    }
}
