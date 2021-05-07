#include <iostream>

#include "../../src/AS5600.h"

AS5600 encoder(0x40);

int revolutions = 0;   // number of revolutions the encoder has made
int position = 0;    // the calculated value the encoder is at
uint16_t output;          // raw value from AS5600
uint16_t lastOutput;        // last output from AS5600


void setup() {
    encoder.setup();

    output = encoder.getRawAngle();
    lastOutput = output;
    position = (int)output;
}

void loop() {
    output = encoder.getRawAngle();           // get the raw value of the encoder
    if ((lastOutput - output) > 2047 )        // check if a full rotation has been made
        revolutions++;
    if ((lastOutput - output) < -2047 )
        revolutions--;
    
    position = revolutions * 4096 + output;   // calculate the position the the encoder is at based off of the number of revolutions

    std::cout << position << std::endl;

    lastOutput = output;                      // save the last raw value for the next loop 
}

int main(int argc, char* argv[]) {
    setup();
    while(true) {
      loop();
    }
    return 0;
}