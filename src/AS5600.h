
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

#ifndef AS5600_h
#define AS5600_h

#include <cstdint>

class AS5600
{
public:
    AS5600();
    int getAddress() const;

    void setup();

    uint16_t setMaxAngle(uint16_t newMaxAngle = -1);
    uint16_t getMaxAngle();

    uint16_t setStartPosition(uint16_t startAngle = -1);
    uint16_t getStartPosition();

    uint16_t setEndPosition(uint16_t endAngle = -1);
    uint16_t getEndPosition();

    uint16_t getRawAngle();
    uint16_t getScaledAngle();

    uint8_t getStatus();
    bool isMagnetTooStrong();
    bool isMagnetTooWeak();
    bool isMagnetDetected();

    uint8_t  getAgc();
    uint16_t getMagnitude();

    uint8_t  getBurnCount();
    int  burnAngle();
    int  burnMaxAngleAndConfig();
    void setConfig(uint8_t mode);
    
private:
  
    const int DEVICE_ADDRESS = 0x36;
      
    uint16_t _rawStartAngle;
    uint16_t _zPosition;
    uint16_t _rawEndAngle;
    uint16_t _mPosition;
    uint16_t _maxAngle;

    enum class RegisterMap : uint8_t
    {

        /* Configuration Registers */
        ZMCO = 0x00,
        ZPOS_H = 0x01,    /*zpos[11:8] high nibble  START POSITION */
        ZPOS_L = 0x02,    /*zpos[7:0] */
        MPOS_H = 0x03,    /*mpos[11:8] high nibble  STOP POSITION */
        MPOS_L = 0x04,    /*mpos[7:0] */
        MANG_H = 0x05,    /*mang[11:8] high nibble  MAXIMUM ANGLE */
        MANG_L = 0x06,    /*mang[7:0] */
        CONF_H = 0x07,
        CONF_L = 0x08,

        /* Output Registers */
        RAW_ANGLE_H = 0x0C,
        RAW_ANGLE_L = 0x0D,
        ANGLE_H = 0x0E,
        ANGLE_L = 0x0F,

        /* Status Registers */
        STATUS = 0x0B,
        AGC = 0x1A,
        MAGNITUDE_H = 0x1B,
        MAGNITUDE_L = 0x1C,

        /* Burn Commands */
        BURN = 0xFF        // [W] Burn_Angle = 0x80; Burn_Setting = 0x40

    };

    int _fd;

    //methods
    uint8_t readReg8(RegisterMap reg);
    uint16_t readReg16(RegisterMap reg);
    void writeReg8(RegisterMap reg, uint8_t value);
    void writeReg16(RegisterMap reg, uint16_t value);
};
#endif