#include <iostream>

#include "../../src/AS5600.h"

AS5600 encoder(0x40);
uint16_t output;

void setup() {
    encoder.setup();
}

void loop() {
    // get the angle in degrees of the encoder
    output = encoder.getAngle();
    std::cout << output << std::endl;
}

int main(int argc, char* argv[]) {
    setup();
    while(true) { 
      loop();
    }
    return 0;
}