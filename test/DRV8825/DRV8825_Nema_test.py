#!/usr/bin/env python3
""" test example file for StepperDriverLib.py DRV8825 NEMA"""

import time 
import RPi.GPIO as GPIO
import sys
sys.path.append('../scripts/')

from StepperDriverLib import A4988_Nema

def main():
    """main function loop"""
    
    # ====== tests for motor ====
    
    #GPIO pins 
    GPIO_pins = (14, 15, 18) # Microstep Resolution MS1-MS3 -> GPIO Pin
    direction = 20       # Direction -> GPIO Pin
    step = 21      # Step -> GPIO Pin
    en = 16
    
    # Declare an named instance of class pass GPIO-PINs
    mymotortest = A4988_Nema(step, direction, en, GPIO_pins, "DRV8825")

    """
    rotate(self, degree=90.0, clockwise=False, rpm=3, step_type="1/8", verbose=False, init_delay=.05):
    """
    
    # ====================== section A ===================
    print("TEST SECTION A")
    
    input("TEST: Full Step 180 [Press <Enter> to continue]")
    mymotortest.rotate(degree=180.0, clockwise=False, rpm=3, step_type="Full", verbose=True, init_delay=.05)
    time.sleep(1)
    input("TEST: Full Step Clockwise [Press <Enter> to continue]")
    mymotortest.rotate(degree=180.0, clockwise=True, rpm=3, step_type="Full", verbose=True, init_delay=.05)
    time.sleep(1)
    input("TEST: Full Step 360 [Press <Enter> to continue]")
    mymotortest.rotate(degree=360.0, clockwise=False, rpm=3, step_type="Full", verbose=False, init_delay=.05)
    time.sleep(1) 
    input("TEST: Full Step Slow [Press <Enter> to continue]")
    mymotortest.rotate(degree=180.0, clockwise=True, rpm=0.5, step_type="Full", verbose=True, init_delay=.05)
    time.sleep(1)
    input("TEST: Full Step InitDelay [Press <Enter> to continue]")
    mymotortest.rotate(degree=180.0, clockwise=False, rpm=3, step_type="Full", verbose=True, init_delay=10)
    time.sleep(1)
    
    # ========================== section B =========================
    print("TEST SECTION B")
    
    input("TEST: Half Step [Press <Enter> to continue]")
    mymotortest.rotate(degree=180.0, clockwise=True, rpm=3, step_type="Half", verbose=True, init_delay=.05)
    time.sleep(1)
    input("TEST: 1/4 Step [Press <Enter> to continue]")
    mymotortest.rotate(degree=180.0, clockwise=False, rpm=3, step_type="1/4", verbose=True, init_delay=.05)
    time.sleep(1)
    input("TEST: 1/8 Step [Press <Enter> to continue]")
    mymotortest.rotate(degree=180.0, clockwise=True, rpm=3, step_type="1/8", verbose=True, init_delay=.05)
    time.sleep(1)
    input("TEST: 1/16 Step [Press <Enter> to continue]")
    mymotortest.rotate(degree=180.0, clockwise=False, rpm=3, step_type="1/16", verbose=True, init_delay=.05)
    time.sleep(1)
    input("TEST: 1/32 Step [Press <Enter> to continue]")
    mymotortest.rotate(degree=180.0, clockwise=True, rpm=3, step_type="1/32", verbose=True, init_delay=.05)
    time.sleep(1)
    
# ===================MAIN===============================

if __name__ == '__main__':
   
    print("TEST START")
    main()
    GPIO.cleanup()
    print("TEST END")
    exit()
    
    

# =====================END===============================
