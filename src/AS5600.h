
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

    AMS_5600();
    int getAddress();

    void setup();

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
  
    const int DEVICE_ADDRESS = 0x36;
      
    uint16_t _rawStartAngle;
    uint16_t _zPosition;
    uint16_t _rawEndAngle;
    uint16_t _mPosition;
    uint16_t _maxAngle;

    struct Registers {

        /* Configuration Registers */
        const int ZMCO = 0x00;
        const int ZPOS_H = 0x01;    /*zpos[11:8] high nibble  START POSITION */
        const int ZPOS_L = 0x02;    /*zpos[7:0] */
        const int MPOS_H = 0x03;    /*mpos[11:8] high nibble  STOP POSITION */
        const int MPOS_L = 0x04;    /*mpos[7:0] */
        const int MANG_H = 0x05;    /*mang[11:8] high nibble  MAXIMUM ANGLE */
        const int MANG_L = 0x06;    /*mang[7:0] */
        const int CONF_H = 0x07;
        const int CONF_L = 0x08;

        /* Output Registers */
        const int RAW_ANGLE_H = 0x0C;
        const int RAW_ANGLE_L = 0x0D;
        const int ANGLE_H = 0x0E;
        const int ANGLE_L = 0x0F;

        /* Status Registers */
        const int STATUS = 0x0B;
        const int AGC = 0x1A;
        const int MAGNITUDE_H = 0x1B;
        const int MAGNITUDE_L = 0x1C;

        /* Burn Commands */
        const int BURN = 0xFF;        // [W] Burn_Angle = 0x80; Burn_Setting = 0x40

    } REGS;

    int _fd;

    //methods
    uint8_t readReg8(uint8_t reg);
    uint16_t readReg16(uint8_t reg);
    void writeReg8(uint8_t reg, uint8_t value);
    void writeReg16(uint8_t reg, uint16_t value);
};
#endif