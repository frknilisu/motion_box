/*
 * Microstepping demo
 *
 * This requires that microstep control pins be connected in addition to STEP,DIR
 *
 * Copyright (C)2015 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <unistd.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120

#define DIR 20
#define STEP 21
#define SLEEP 16 // optional (just delete SLEEP from everywhere if not used)

/*
 * Choose one of the sections below that match your board
 */

#include "../../src/DRV8825.h"
#define MS1 14
#define MS2 15
#define MS3 18
DRV8825 stepper(MOTOR_STEPS, DIR, STEP, SLEEP, MS1, MS2, MS3);

// #include "BasicStepperDriver.h" // generic
// BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

void setup() {
    /*
     * Set target motor RPM.
     */
    stepper.begin(RPM);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);
    stepper.enable();
}

void loop() {
    usleep(1000);

    /*
     * Moving motor in full step mode is simple:
     */
    stepper.setMicrostep(1);  // Set microstep mode to 1:1

    // One complete revolution is 360°
    stepper.rotate(360);     // forward revolution
    stepper.rotate(-360);    // reverse revolution

    // One complete revolution is also MOTOR_STEPS steps in full step mode
    stepper.move(MOTOR_STEPS);    // forward revolution
    stepper.move(-MOTOR_STEPS);   // reverse revolution

    /*
     * Microstepping mode: 1, 2, 4, 8, 16 or 32 (where supported by driver)
     * Mode 1 is full speed.
     * Mode 32 is 32 microsteps per step.
     * The motor should rotate just as fast (at the set RPM),
     * but movement precision is increased, which may become visually apparent at lower RPMs.
     */
    stepper.setMicrostep(8);   // Set microstep mode to 1:8

    // In 1:8 microstepping mode, one revolution takes 8 times as many microsteps
    stepper.move(8 * MOTOR_STEPS);    // forward revolution
    stepper.move(-8 * MOTOR_STEPS);   // reverse revolution
    
    // One complete revolution is still 360° regardless of microstepping mode
    // rotate() is easier to use than move() when no need to land on precise microstep position
    stepper.rotate(360);
    stepper.rotate(-360);

    usleep(5000);
}

int main(int argc, char* argv[]) {
    setup();
    while(true) { 
        loop();
        break;
    }
    return 0;
}