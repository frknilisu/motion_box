
/****************************************************
/* AMS 5600 class for Arduino platform
/* Author: Tom Denton
/* Date: 15 Dec 2014 
/* File: AMS_5600.cpp
/* Version 1.00
/* www.ams.com
/*  
/* Description:  This class has been designed to
/* access the AMS 5600 “potuino” shield.
/*
/***************************************************/
#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <wiringPiI2C.h>

#include "AS5600.h"

using namespace std;

/****************************************************
/* Method: AMS_5600
/* In: none
/* Out: none
/* Description: constructor class for AMS 5600
/***************************************************/
AMS_5600::AMS_5600()
{
    /* set i2c address */ 
    _ams5600_Address = 0x36;

    /* load register values*/
    /* c++ class forbids pre loading of variables */
    _zmco = 0x00;
    _zpos_hi = 0x01;
    _zpos_lo = 0x02;
    _mpos_hi = 0x03;
    _mpos_lo = 0x04;
    _mang_hi = 0x05;
    _mang_lo = 0x06;
    _conf_hi = 0x07;    
    _conf_lo = 0x08;
    _raw_ang_hi = 0x0c;
    _raw_ang_lo = 0x0d;
    _ang_hi = 0x0e;
    _ang_lo = 0x0f;
    _stat = 0x0b;
    _agc = 0x1a;
    _mag_hi = 0x1b;
    _mag_lo = 0x1c;
    _burn = 0xff;
}

void AMS_5600::setup()
{
    // Setup I2C communication
    _fd = wiringPiI2CSetup(DEVICE_ID);
    if (_fd == -1) {
        std::cout << "Failed to init I2C communication." << std::endl;
        return -1;
    }
    std::cout << "I2C communication successfully setup.\n";
}

uint8_t AMS_5600::readReg8(uint8_t reg)
{
    uint8_t readValue = (uint8_t) wiringPiI2CReadReg8(_fd, reg);
    return readValue;
}

uint16_t AMS_5600::readReg16(uint8_t reg)
{
    uint16_t readValue = (uint16_t) wiringPiI2CReadReg16(_fd, reg);
    return readValue;
}

void AMS_5600::writeReg8(uint8_t reg, uint8_t value)
{
    int res = wiringPiI2CWriteReg8(_fd, reg, value);
    if (res) 
    {
        std::cout << "I2C error: " << res << std::endl;
    }
}

void AMS_5600::writeReg16(uint8_t reg, uint16_t value)
{
    int res = wiringPiI2CWriteReg8(_fd, reg, value);
    if (res) 
    {
        std::cout << "I2C error: " << res << std::endl;
    }
}

/* mode = 0, output PWM, mode = 1 output analog (full range from 0% to 100% between GND and VDD*/
void AMS_5600::setOutPut(uint8_t mode)
{
    uint8_t config_status = readReg8(_conf_lo);
    if(mode == 1)
    {
        config_status = config_status & 0xcf;
    } 
    else
    {
        config_status = config_status & 0xef;
    }
    writeReg8(_conf_lo, config_status); 
}

/****************************************************
/* Method: AMS_5600
/* In: none
/* Out: i2c address of AMS 5600
/* Description: returns i2c address of AMS 5600
/***************************************************/
int AMS_5600::getAddress()
{
    return _ams5600_Address; 
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
uint16_t AMS_5600::setMaxAngle(uint16_t newMaxAngle)
{
    if(newMaxAngle == -1)
        _maxAngle = getRawAngle();
    else
        _maxAngle = newMaxAngle;

    writeReg16(_mang_hi, _maxAngle); 
    sleep(2);

    return readReg16(_mang_hi);
}

/*******************************************************
/* Method: getMaxAngle
/* In: none
/* Out: value of max angle register
/* Description: gets value of maximum angle register.
/*******************************************************/
uint16_t AMS_5600::getMaxAngle()
{
    return readReg16(_mang_hi);
}

/*******************************************************
/* Method: setStartPosition
/* In: new start angle position
/* Out: value of start position register
/* Description: sets a value in start position register.
/* If no value is provided, method will read position of
/* magnet.  
/*******************************************************/
uint16_t AMS_5600::setStartPosition(uint16_t startAngle)
{
    if(startAngle == -1)
    {
        _rawStartAngle = getRawAngle();
    }
    else
    {
        _rawStartAngle = startAngle;
    }

    writeReg16(_zpos_hi, _rawStartAngle); 
    sleep(2);

    _zPosition = readReg16(_zpos_hi);
  
    return _zPosition;
}

/*******************************************************
/* Method: getStartPosition
/* In: none
/* Out: value of start position register
/* Description: gets value of start position register.
/*******************************************************/
uint16_t AMS_5600::getStartPosition()
{
    return readReg16(_zpos_hi);
}  

/*******************************************************
/* Method: setEndtPosition
/* In: new end angle position
/* Out: value of end position register
/* Description: sets a value in end position register.
/* If no value is provided, method will read position of
/* magnet.  
/*******************************************************/
uint16_t AMS_5600::setEndPosition(uint16_t endAngle)
{
    if(endAngle == -1)
        _rawEndAngle = getRawAngle();
    else
        _rawEndAngle = endAngle;
 
    writeReg16(_mpos_hi, _rawEndAngle);
    sleep(2);
    
    _mPosition = readReg16(_mpos_hi);
  
    return _mPosition;
}

/*******************************************************
/* Method: getEndPosition
/* In: none
/* Out: value of end position register
/* Description: gets value of end position register.
/*******************************************************/
uint16_t AMS_5600::getEndPosition()
{
    return readReg16(_mpos_hi);
}  

/*******************************************************
/* Method: getRawAngle
/* In: none
/* Out: value of raw angle register
/* Description: gets raw value of magnet position.
/* start, end, and max angle settings do not apply
/*******************************************************/
uint16_t AMS_5600::getRawAngle()
{
  return readReg16(_raw_ang_hi);
}

/*******************************************************
/* Method: getScaledAngle
/* In: none
/* Out: value of scaled angle register
/* Description: gets scaled value of magnet position.
/* start, end, or max angle settings are used to 
/* determine value
/*******************************************************/
uint16_t AMS_5600::getScaledAngle()
{
  return readReg16(_ang_hi);
}

/*******************************************************
/* Method: isMagnetDetected
/* In: none
/* Out: true if magnet is detected, false if not
/* Description: reads status register and examines the 
/* MH bit
/*******************************************************/
bool AMS_5600::isMagnetDetected()
{
    int detected = false;
    /* 0 0 MD ML MH 0 0 0 */
    /* MD high = magnet detected */
    /* ML high = AGC Maximum overflow, magnet to weak */ 
    /* MH high = AGC minimum overflow, Magnet to strong */
    uint8_t magStatus = readReg8(_stat);
    
    if(magStatus & (1 << 5)) {
        detected = true;
    }
  
    return detected;
}

/*******************************************************
/* Method: getMagnetStrength
/* In: none
/* Out: 0 if no magnet is detected
/*      1 if magnet is to weak
/*      2 if magnet is just right
/*      3 if magnet is to strong
/* Description: reads status register andexamins the MH,ML,MD bits
/*******************************************************/
int AMS_5600::getMagnetStrength()
{
    int retVal = 0;
    /* 0 0 MD ML MH 0 0 0 */
    /* MD high = magnet detected */
    /* ML high = AGC Maximum overflow, magnet to weak */ 
    /* MH high = AGC minimum overflow, Magnet to strong */ 
    uint8_t magStatus = readReg8(_stat);
    if(isMagnetDetected()) {
        retVal = 2; /*just right */
        if(magStatus & (1 << 4))
            retVal = 1; /*to weak */
        else if(magStatus & (1 << 3))
            retVal = 3; /*to strong */
    }

    return retVal;
}

/*******************************************************
/* Method: get Agc
/* In: none
/* Out: value of AGC register
/* Description: gets value of AGC register.
/*******************************************************/
int AMS_5600::getAgc()
{
    return static_cast<int>(readReg8(_agc));
}

/*******************************************************
/* Method: getMagnitude
/* In: none
/* Out: value of magnitude register
/* Description: gets value of magnitude register.
/*******************************************************/
uint16_t AMS_5600::getMagnitude()
{
  return readReg16(_mag_hi);
}

/*******************************************************
/* Method: getBurnCount
/* In: none
/* Out: value of zmco register
/* Description: determines how many times chip has been
/* permanently written to. 
/*******************************************************/
int AMS_5600::getBurnCount()
{
    return static_cast<int>(readReg8(_zmco));
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
int AMS_5600::burnAngle()
{
    int retVal = 1;
    _zPosition = getStartPosition();
    _mPosition = getEndPosition();
    _maxAngle  = getMaxAngle();
  
    if(isMagnetDetected())
    {
        if(getBurnCount() < 3)
        {
            if((_zPosition == 0) && (_mPosition == 0))
                retVal = -3;
            else
                writeReg8(_burn, 0x80);
        }
        else
        {
            retVal = -2;
        }
    } 
    else
    {
        retVal = -1;
    }
    
    return retVal;
}

/*******************************************************
/* Method: burnMaxAngleAndConfig
/* In: none
/* Out: 1 success
/*     -1 burn limit exceeded
/*     -2 max angle is to small, must be at or above 18 degrees
/* Description: burns max angle and config data to chip.
/* THIS CAN ONLY BE DONE 1 TIME
/*******************************************************/
int AMS_5600::burnMaxAngleAndConfig()
{
    int retVal = 1;
    _maxAngle  = getMaxAngle();
  
    if(getBurnCount() == 0)
    {
        if(_maxAngle*0.087 < 18)
            retVal = -2;
        else
            writeReg8(_burn, 0x40);    
    }  
    else
    {
        retVal = -1;
    }
    
    return retVal;
}

/**********  END OF AMS 5600 CALSS *****************/
