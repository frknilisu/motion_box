#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

//#include "../src/AS5600.h"

#include <gtest/gtest.h>

class AS5600Test : public ::testing::Test
{
public:

    AS5600Test()
    {
        std::cout << "AS5600Test()" << std::endl;
        //AS5600 encoder(0x40);
        //encoder.setup();
        this->_i2cAddress = 0x40;
    }

    virtual void SetUp()
    {
        std::cout << "SetUp()" << std::endl;
    }

    virtual void TearDown()
    {
        std::cout << "TearDown()" << std::endl;
    }

    ~AS5600Test()
    {
        std::cout << "~AS5600Test()" << std::endl;
    }

    uint16_t getStartPosition() const { return this->_startPosition; }
    void setStartPosition(uint16_t startPosition) { this->_startPosition = startPosition; }
    uint16_t getEndPosition() const { return this->_endPosition; }
    void setEndPosition(uint16_t endPosition) { this->_endPosition = endPosition; }

    uint16_t getAddress() const { return this->_i2cAddress; }

private:
    uint16_t _startPosition;
    uint16_t _endPosition;
    uint16_t _i2cAddress;

};

TEST_F(AS5600Test, DefaultConstructor)
{
    EXPECT_EQ(0x40, getAddress());
}


TEST_F(AS5600Test, SetStartPosition) {
    const uint16_t startPosition = 0x14;
    setStartPosition(startPosition);
    //encoder.setStartPosition(startPosition);
    //EXPECT_EQ(startPosition, encoder.getStartPosition());
    EXPECT_EQ(startPosition, getStartPosition());
}

TEST_F(AS5600Test, SetEndPosition) {
    const uint16_t endPosition = 0x0c80;
    setEndPosition(endPosition);
    //encoder.setEndPosition(endPosition);
    //EXPECT_EQ(endPosition, encoder.getEndPosition());
    EXPECT_EQ(endPosition, getEndPosition());
}

TEST(SquareRootTest, PositiveNos) { 
    EXPECT_DOUBLE_EQ(18.0, sqrt(324.0));
    EXPECT_DOUBLE_EQ(25.4, sqrt(645.16));
    EXPECT_DOUBLE_EQ(50.332, sqrt(2533.310224));
}
