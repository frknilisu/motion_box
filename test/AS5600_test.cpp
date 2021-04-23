#include <iostream>
#include <thread>
#include <chrono>
#include <wiringPiI2C.h>

#include "../src/AS5600.h"

int main (int argc, char **argv)
{
    AS5600 encoder;
    encoder.setup();

    while (1) {
        uint16_t raw_angle = encoder.getRawAngle();
        std::cout << "Raw Angle: " << raw_angle << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}