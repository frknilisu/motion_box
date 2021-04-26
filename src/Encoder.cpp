#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <unistd.h>

#include "AS5600.h"

double convertRawAngleToDegrees(uint16_t rawAngle)
{
    return rawAngle * 0.087890625;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "encoder_node");
    ros::NodeHandle nh;
    ROS_INFO("Hello, World!");

    AS5600 encoder(0x40);
    encoder.setup();
    uint16_t startPosition = 0;
    uint16_t prevPosition = 0;
    uint16_t currPosition = 0;
    
    encoder.setStartPosition();
    startPosition = encoder.getStartPosition();
    std::cout << "Start Position: " << startPosition << std::endl;
    while(ros::ok()) {
        if(encoder.isMagnetDetected()) {
            currPosition = encoder.getRawAngle();
            if(prevPosition != currPosition) {
                std::cout << "Current Position: " << currPosition << std::endl;
                std::cout << "Diff: " << currPosition-startPosition << std::endl;
                std::cout << "Diff Angle: " << convertRawAngleToDegrees(currPosition-startPosition) << std::endl;
            }
            prevPosition = currPosition;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ros::spinOnce();
    }
}