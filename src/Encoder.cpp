#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <chrono>

#include "AS5600.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "encoder_node");
    ros::NodeHandle nh;
    ROS_INFO("Hello, World!");

    AS5600 encoder(0x40);
    encoder.setup();
    uint16_t start_raw_angle = 0;
    uint16_t prev_raw_angle = 0;
    uint16_t curr_raw_angle = 0;
    curr_raw_angle = encoder.getRawAngle();
    std::cout << curr_raw_angle << std::endl;
    std::cout << encoder.setStartPosition(curr_raw_angle) << std::endl;
    //std::cout << encoder.setEndPosition(180) << std::endl;
    //std::cout << encoder.setMaxAngle(135) << std::endl;
    start_raw_angle = curr_raw_angle;
    std::cout << "Start Position: " << start_raw_angle << std::endl;
    std::cout << "End Position: " << encoder.getEndPosition() << std::endl;
    std::cout << "Max Angle: " << encoder.getMaxAngle() << std::endl;
    while(ros::ok()) {
        if(encoder.isMagnetDetected()) {
            curr_raw_angle = encoder.getRawAngle();
            if(prev_raw_angle != curr_raw_angle) {
                std::cout << "Raw Angle: " << curr_raw_angle << std::endl;
                std::cout << "Scaled Angle[0-4095]: " << encoder.getAngle() << std::endl;
                std::cout << "Diff: " << curr_raw_angle - start_raw_angle << std::endl;
            }
            prev_raw_angle = curr_raw_angle;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ros::spinOnce();
    }
}