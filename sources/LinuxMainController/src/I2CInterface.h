/*
 * I2CInterface.h
 *
 *  Created on: 14.06.2015
 *      Author: JochenAlt
 */

#ifndef SRC_I2CINTERFACE_H_
#define SRC_I2CINTERFACE_H_

#include <stdint.h>
#include <stddef.h>
#include <string>

class I2CInterface {
public:

	enum OpenMode           {   ReadOnly                = 0,
	                            WriteOnly               = 1,
	                            ReadWrite               = 2,
	                            Append                  = 4,
	                            Truncate                = 8,
	                            NonBlock                = 16
	                        };

	enum Direction          {   input                   = 1,
	                                output                  = 2,
	                                bothDirection           = (input|output)
	                            };

    /*!
     * This enum is used for selecting i2c bus transaction type.
     */
    enum TransactionType    {   SMBUS_QUICK             = 0,
                                SMBUS_BYTE              = 1,
                                SMBUS_BYTE_DATA         = 2,
                                SMBUS_WORD_DATA         = 3,
                                SMBUS_PROC_CALL         = 4,
                                SMBUS_BLOCK_DATA        = 5,
                                SMBUS_I2C_BLOCK_BROKEN  = 6,
                                SMBUS_BLOCK_PROC_CALL   = 7,
                                SMBUS_I2C_BLOCK_DATA    = 8
                            };


	I2CInterface() {
		i2cDevAddress = 0;
		i2cFD = 0;
		isOpenFlag = false;
	}

	void setup(std::string i2cPath, unsigned int i2cDeviceAddress);
	static I2CInterface& getInstance() {
		static I2CInterface instance;
		return instance;
	}

    bool        open(OpenMode openMode);
    bool        writeBlock(uint8_t registerAddr, uint8_t *writeBuffer, size_t bufferSize);
    bool        writeByte(uint8_t registerAddr, uint8_t value);
    bool        readLine(uint8_t *readBuffer, size_t bufferSize);
    bool        close();

private:
    unsigned int    i2cDevAddress;              /*!< @brief is used to hold the i2c's device address */
    int             i2cFD;                      /*!< @brief is used to hold the i2c's tty file's file descriptor */
    std::string     i2cPortPath;                /*!< @brief is used to hold the i2c's tty port path */
    bool            isOpenFlag;                 /*!< @brief is used to hold the i2c's tty file's state */



};

#endif /* SRC_I2CINTERFACE_H_ */
