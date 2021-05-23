
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
    AS5600(uint8_t address);
    ~AS5600();

    void setup();
    int getAddress() const;

    uint8_t getBurnCount();
    uint16_t getStartPosition();
    void setStartPosition(uint16_t startAngle = -1);
    uint16_t getEndPosition();
    void setEndPosition(uint16_t endAngle = -1);
    uint16_t getMaxAngle();
    void setMaxAngle(uint16_t maxAngle = -1);
    uint16_t getConfig();
    void setConfig(uint8_t mode);
    uint16_t getRawAngle();
    uint16_t getAngle();
    uint8_t getStatus();
    bool isMagnetTooStrong();
    bool isMagnetTooWeak();
    bool isMagnetDetected();
    uint8_t getAgc();
    uint16_t getMagnitude();
    int burnAngle();
    int burnSettings();

    static const int RESOLUTION_BIT = 12;
    static const int TOTAL_STEP_COUNT = 2 << RESOLUTION_BIT;
    static const double STEP_ANGLE = 360.0 / TOTAL_STEP_COUNT;
    
private:
  
    const int AS5600_I2C_ADDR = 0x36;
    
    uint8_t i2cAddress;

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
    uint8_t readReg8(uint8_t reg);
    uint16_t readReg16(uint8_t reg);
    void writeReg8(uint8_t reg, uint8_t value);
    void writeReg16(uint8_t reg, uint16_t value);
};
#endif