
/****************************************************
/* AMS 5600 class for Arduino platform
/* Author: Tom Denton
/* Date: 15 Dec 2014
/* File: AMS_5600.h 
/* Version 1.00
/* www.ams.com
/*  
/* Description:  This class has been designed to
/* access the AMS 5600 “potuino” shield.
/*
/***************************************************/

#ifndef AMS_5600_h
#define AMS_5600_h

#include <cstdint>

class AMS_5600
{
public:

    AMS_5600(void);
    int getAddress();       

    uint16_t setMaxAngle(uint16_t newMaxAngle = -1);
    uint16_t getMaxAngle();

    uint16_t setStartPosition(uint16_t startAngle = -1);
    uint16_t getStartPosition();

    uint16_t setEndPosition(uint16_t endAngle = -1);
    uint16_t getEndPosition();

    uint16_t getRawAngle();
    uint16_t getScaledAngle();

    bool  isMagnetDetected();
    int  getMagnetStrength();
    int  getAgc();
    uint16_t getMagnitude();

    int  getBurnCount();
    int  burnAngle();
    int  burnMaxAngleAndConfig();
    void setOutPut(uint8_t mode);
    
private:
  
    int _ams5600_Address;
      
    uint16_t _rawStartAngle;
    uint16_t _zPosition;
    uint16_t _rawEndAngle;
    uint16_t _mPosition;
    uint16_t _maxAngle;

    /* Registers */
    int _zmco;
    int _zpos_hi;    /*zpos[11:8] high nibble  START POSITION */
    int _zpos_lo;    /*zpos[7:0] */
    int _mpos_hi;    /*mpos[11:8] high nibble  STOP POSITION */
    int _mpos_lo;    /*mpos[7:0] */
    int _mang_hi;    /*mang[11:8] high nibble  MAXIMUM ANGLE */
    int _mang_lo;    /*mang[7:0] */
    int _conf_hi;    
    int _conf_lo;
    int _raw_ang_hi;
    int _raw_ang_lo;
    int _ang_hi;
    int _ang_lo;
    int _stat;
    int _agc;
    int _mag_hi;
    int _mag_lo;
    int _burn;

    int _fd;

    //methods
    void setup();
    uint8_t readReg8(uint8_t reg);
    uint16_t readReg16(uint8_t reg);
    void writeReg8(uint8_t reg, uint8_t value);
    void writeReg16(uint8_t reg, uint16_t value);   
};
#endif