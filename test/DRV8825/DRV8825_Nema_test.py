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
    
    # ====================== section A1 ===================

    """
    mymotortest.setMicrostep(microstep="Full")
    mymotortest.setDirection(clockwise=False)
    mymotortest.setRPM(rpm=3)
    mymotortest.rotate(degree=180.0)
    """

    print("TEST SECTION A1")
    
    input("TEST: Full Step 180 [Press <Enter> to continue]")
    mymotortest.setMicrostep(microstep="Full")
    mymotortest.setDirection(clockwise=False)
    mymotortest.setRPM(rpm=3)
    mymotortest.rotate(degree=180.0)
    time.sleep(1)
    input("TEST: Full Step Clockwise [Press <Enter> to continue]")
    mymotortest.setDirection(clockwise=True)
    mymotortest.rotate(degree=180.0)
    time.sleep(1)
    input("TEST: Full Step 360 [Press <Enter> to continue]")
    mymotortest.rotate(degree=360.0)
    time.sleep(1)
    input("TEST: Full Step Slow [Press <Enter> to continue]")
    mymotortest.setRPM(rpm=0.5)
    mymotortest.rotate(degree=180.0)
    time.sleep(1)
    
    # ========================== section A2 =========================
    print("TEST SECTION A2")
    
    input("TEST: Half Step [Press <Enter> to continue]")
    mymotortest.setMicrostep(microstep="Half")
    mymotortest.setDirection(clockwise=True)
    mymotortest.setRPM(rpm=3)
    mymotortest.rotate(degree=180.0)
    time.sleep(1)
    input("TEST: 1/4 Step [Press <Enter> to continue]")
    mymotortest.setMicrostep(microstep="1/4")
    mymotortest.rotate(degree=180.0)
    time.sleep(1)
    input("TEST: 1/8 Step [Press <Enter> to continue]")
    mymotortest.setMicrostep(microstep="1/8")
    mymotortest.rotate(degree=180.0)
    time.sleep(1)
    input("TEST: 1/16 Step [Press <Enter> to continue]")
    mymotortest.setMicrostep(microstep="1/16")
    mymotortest.rotate(degree=180.0)
    time.sleep(1)
    input("TEST: 1/32 Step [Press <Enter> to continue]")
    mymotortest.setMicrostep(microstep="1/32")
    mymotortest.rotate(degree=180.0)
    time.sleep(1)

    # ========================== section B =========================

    """
    mymotortest.setMicrostep(microstep="Full")
    mymotortest.setDirection(clockwise=False)
    mymotortest.setRPM(rpm=3)
    mymotortest.rotate(degree=180.0)
    """

    print("TEST SECTION B")
    input("TEST: 1/4 Step Move [Press <Enter> to continue]")
    mymotortest.setMicrostep(microstep="1/4")
    mymotortest.setDirection(clockwise=False)
    mymotortest.setRPM(rpm=3)
    mymotortest.move(steps=400)
    time.sleep(1)
    
# ===================MAIN===============================

if __name__ == '__main__':
   
    print("TEST START")
    main()
    GPIO.cleanup()
    print("TEST END")
    exit()
    
    

# =====================END===============================
