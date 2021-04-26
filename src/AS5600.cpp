
/****************************************************
/* AS5600 class for RaspberryPi platform
/* Author: Abdullah Furkan Ilisu
/* Date: 23 April 2021
/* File: AS5600.h 
/* Version 1.00
/*  
/* Description: 
/*
/***************************************************/

#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <wiringPiI2C.h>

#include "AS5600.h"

using namespace std;

uint8_t lowByte(uint16_t value)
{
    return (value & 0x00FF);
}

uint8_t highByte(uint16_t value)
{
    return ((value & 0xFF00) >> 8);
}

double convertRawAngleToDegrees(uint16_t rawAngle)
{
    return rawAngle * 0.087890625;
}

/****************************************************
/* Method: AS5600
/* In: none
/* Out: none
/* Description: constructor class for AS5600
/***************************************************/
AS5600::AS5600()
{
    i2cAddress = AS5600_I2C_ADDR;
}

/****************************************************
/* Method: AS5600
/* In: none
/* Out: none
/* Description: constructor class for AS5600
/***************************************************/
AS5600::AS5600(uint8_t address)
{
    i2cAddress = address;
}

/****************************************************
/* Method: AS5600
/* In: none
/* Out: none
/* Description: destructor class for AS5600
/***************************************************/
AS5600::~AS5600()
{
}

/****************************************************
/* Method: setup
/* In: none
/* Out: none
/* Description: constructor class for AS5600
/***************************************************/
void AS5600::setup()
{
    // Setup I2C communication
    _fd = wiringPiI2CSetup(i2cAddress);
    if (_fd == -1) {
        std::cout << "Failed to init I2C communication." << std::endl;
        return;
    }
    std::cout << "I2C communication successfully setup." << std::endl;
}

/****************************************************
/* Method: getAddress
/* In: none
/* Out: i2c address of AS5600
/* Description: returns i2c address of AS5600
/***************************************************/
int AS5600::getAddress() const
{
    return i2cAddress; 
}

/*******************************************************
/* Method: getBurnCount
/* In: none
/* Out: value of zmco register
/* Description: determines how many times chip has been
/* permanently written to. 
/*******************************************************/
uint8_t AS5600::getBurnCount()
{
    return readReg8(static_cast<uint8_t>(RegisterMap::ZMCO));
}

/*******************************************************
/* Method: getStartPosition
/* In: none
/* Out: value of start position register
/* Description: gets value of start position register.
/*******************************************************/
uint16_t AS5600::getStartPosition()
{
    return readReg16(static_cast<uint8_t>(RegisterMap::ZPOS_H));
}

/*******************************************************
/* Method: setStartPosition
/* In: new start angle position
/* Out: value of start position register
/* Description: sets a value in start position register.
/* If no value is provided, method will read position of
/* magnet.  
/*******************************************************/
void AS5600::setStartPosition(uint16_t startAngle)
{
    uint16_t rawAngle = 0;
    if(startAngle == -1) {
        rawAngle = getRawAngle();
    } else {
        rawAngle = startAngle;
    }

    writeReg16(static_cast<uint8_t>(RegisterMap::ZPOS_H), rawAngle); 
    sleep(1);
}

/*******************************************************
/* Method: getEndPosition
/* In: none
/* Out: value of end position register
/* Description: gets value of end position register.
/*******************************************************/
uint16_t AS5600::getEndPosition()
{
    return readReg16(static_cast<uint8_t>(RegisterMap::MPOS_H));
}

/*******************************************************
/* Method: setEndPosition
/* In: new end angle position
/* Out: value of end position register
/* Description: sets a value in end position register.
/* If no value is provided, method will read position of
/* magnet.  
/*******************************************************/
void AS5600::setEndPosition(uint16_t endAngle)
{
    uint16_t rawAngle = 0;
    if(endAngle == -1) {
        rawAngle = getRawAngle();
    } else {
        rawAngle = endAngle;
    }
 
    writeReg16(static_cast<uint8_t>(RegisterMap::MPOS_H), rawAngle);
    sleep(1);
}

/*******************************************************
/* Method: getMaxAngle
/* In: none
/* Out: value of max angle register
/* Description: gets value of maximum angle register.
/*******************************************************/
uint16_t AS5600::getMaxAngle()
{
    return readReg16(static_cast<uint8_t>(RegisterMap::MANG_H));
}

/*******************************************************
/* Method: setMaxAngle
/* In: new maximum angle to set OR none
/* Out: value of max angle register
/* Description: sets a value in maximum angle register.
/* If no value is provided, method will read position of
/* magnet.  Setting this register zeros out max position
/* register.
/*******************************************************/
void AS5600::setMaxAngle(uint16_t maxAngle)
{
    uint16_t rawAngle = 0;   
    if(newMaxAngle == -1) {
        rawAngle = getRawAngle();
    } else {
        rawAngle = maxAngle;
    }

    writeReg16(static_cast<uint8_t>(RegisterMap::MANG_H), rawAngle); 
    sleep(1);
}

/****************************************************
/* Method: getConfig
/* In: none
/* Out: none
/* Description: constructor class for AS5600
/* mode = 0, output PWM, 
/* mode = 1 output analog (full range from 0% to 100% 
/* between GND and VDD*/
/***************************************************/
uint16_t AS5600::getConfig()
{
    return readReg16(static_cast<uint8_t>(RegisterMap::CONF_H));
}

/****************************************************
/* Method: setConfig
/* In: none
/* Out: none
/* Description: constructor class for AS5600
/* mode = 0, output PWM, 
/* mode = 1 output analog (full range from 0% to 100% 
/* between GND and VDD*/
/***************************************************/
void AS5600::setConfig(uint8_t mode)
{
    uint8_t config_status = readReg8(static_cast<uint8_t>(RegisterMap::CONF_L));
    if(mode == 1) {
        config_status = config_status & 0xcf;
    } else {
        config_status = config_status & 0xef;
    }
    writeReg8(static_cast<uint8_t>(RegisterMap::CONF_L), config_status); 
}

/*******************************************************
/* Method: getRawAngle
/* In: none
/* Out: value of raw angle register
/* Description: gets raw value of magnet position.
/* start, end, and max angle settings do not apply
/*******************************************************/
uint16_t AS5600::getRawAngle()
{
    return readReg16(static_cast<uint8_t>(RegisterMap::RAW_ANGLE_H));
}

/*******************************************************
/* Method: getAngle
/* In: none
/* Out: value of scaled angle register
/* Description: gets scaled value of magnet position.
/* start, end, or max angle settings are used to 
/* determine value
/*******************************************************/
uint16_t AS5600::getAngle()
{
    return readReg16(static_cast<uint8_t>(RegisterMap::ANGLE_H));
}

/*******************************************************
/* Method: getStatus
/* In: none
/* Out: value of scaled angle register
/* Description: gets scaled value of magnet position.
/* start, end, or max angle settings are used to 
/* determine value
/*******************************************************/
uint8_t AS5600::getStatus()
{
    return readReg8(static_cast<uint8_t>(RegisterMap::STATUS)) & 0b00111000;
}

/*******************************************************
/* Method: isMagnetDetected
/* In: none
/* Out: true if magnet is detected by AS5600
/* Description: reads status register and examines the MD
/*******************************************************/
bool AS5600::isMagnetDetected()
{
    uint8_t magStatus = 0;
    magStatus = getStatus();
    if(magStatus & (1 << 5)) {
        return true;
    }
    return false;
}

/*******************************************************
/* Method: isMagnetTooWeak
/* In: none
/* Out: true if magnet is too far to AS5600
/* Description: reads status register and examines the ML
/*******************************************************/
bool AS5600::isMagnetTooWeak()
{
    uint8_t magStatus = 0;
    magStatus = getStatus();
    if(magStatus & (1 << 4)) {
        return true;
    }
    return false;
}

/*******************************************************
/* Method: isMagnetTooStrong
/* In: none
/* Out: true if magnet is too close to AS5600
/* Description: reads status register and examines the MH
/*******************************************************/
bool AS5600::isMagnetTooStrong()
{
    uint8_t magStatus = 0;
    magStatus = getStatus();
    if(magStatus & (1 << 3)) {
        return true;
    }
    return false;
}

/*******************************************************
/* Method: getAgc
/* In: none
/* Out: value of AGC register
/* Description: gets value of AGC register.
/*******************************************************/
uint8_t AS5600::getAgc()
{
    return readReg8(static_cast<uint8_t>(RegisterMap::AGC));
}

/*******************************************************
/* Method: getMagnitude
/* In: none
/* Out: value of magnitude register
/* Description: gets value of magnitude register.
/*******************************************************/
uint16_t AS5600::getMagnitude()
{
    return readReg16(static_cast<uint8_t>(RegisterMap::MAGNITUDE_H));
}

/*******************************************************
/* Method: burnAngle
/* In: none
/* Out: 1 success
/*     -1 no magnet
/*     -2 burn limit exceeded
/*     -3 start and end positions not set (useless burn)
/* Description: burns start and end positions to chip.
/* THIS CAN ONLY BE DONE 3 TIMES
/*******************************************************/
int AS5600::burnAngle()
{
    int retVal = 1;
    _zPosition = getStartPosition();
    _mPosition = getEndPosition();
    _maxAngle  = getMaxAngle();
  
    if(isMagnetDetected()) {
        if(getBurnCount() < 3) {
            if((_zPosition == 0) && (_mPosition == 0)) {
                retVal = -3;
            } else {
                writeReg8(static_cast<uint8_t>(RegisterMap::BURN), 0x80);
            }
        } else {
            retVal = -2;
        }
    } else {
        retVal = -1;
    }
    
    return retVal;
}

/*******************************************************
/* Method: burnSettings
/* In: none
/* Out: 1 success
/*     -1 burn limit exceeded
/*     -2 max angle is to small, must be at or above 18 degrees
/* Description: burns max angle and config data to chip.
/* THIS CAN ONLY BE DONE 1 TIME
/*******************************************************/
int AS5600::burnSettings()
{
    int retVal = 1;
    _maxAngle  = getMaxAngle();
  
    if(getBurnCount() == 0) {
        if(_maxAngle*0.087 < 18) {
            retVal = -2;
        } else {
            writeReg8(static_cast<uint8_t>(RegisterMap::BURN), 0x40);
        }
    } else {
        retVal = -1;
    }
    
    return retVal;
}

/****************************************************
/* Method: readReg8
/* In: none
/* Out: none
/* Description: constructor class for AS5600
/***************************************************/
uint8_t AS5600::readReg8(uint8_t reg)
{
    uint8_t value = (uint8_t) wiringPiI2CReadReg8(_fd, reg);
    return value;
}

/****************************************************
/* Method: readReg16
/* In: none
/* Out: none
/* Description: constructor class for AS5600
/***************************************************/
uint16_t AS5600::readReg16(uint8_t reg)
{
    uint16_t msb = (uint16_t) wiringPiI2CReadReg8(_fd, reg);
    uint16_t lsb = (uint16_t) wiringPiI2CReadReg8(_fd, reg+1);
    return ((msb << 8) | lsb);
}

/****************************************************
/* Method: writeReg8
/* In: none
/* Out: none
/* Description: constructor class for AS5600
/***************************************************/
void AS5600::writeReg8(uint8_t reg, uint8_t value)
{
    int res = wiringPiI2CWriteReg8(_fd, reg, value);
    if (res) {
        std::cout << "I2C error: " << res << std::endl;
        throw "I2C Write Error";
    }
}

/****************************************************
/* Method: writeReg16
/* In: none
/* Out: none
/* Description: constructor class for AS5600
/***************************************************/
void AS5600::writeReg16(uint8_t reg, uint16_t value)
{
    int res1 = wiringPiI2CWriteReg8(_fd, reg, highByte(value));
    int res2 = wiringPiI2CWriteReg8(_fd, reg+1, lowByte(value));
    if (res1 | res2) {
        std::cout << "I2C error: " << res1 << ", " << res2 << std::endl;
        throw "I2C Write Error";
    }
}

/**********  END OF AS5600 CLASS *****************/
