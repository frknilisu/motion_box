#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <stdint.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include "motion_box/StringTrigger.h"
#include "motion_box/StringTriggerRequest.h"

#include "AS5600.h"

AS5600 encoder(0x40);

int pressCounter = 0;
uint16_t startPosition = 0;
uint16_t endPosition = 0;
bool isStartPositionSet = false;
bool isEndPositionSet = false;

struct JoystickStatus {
    int x = 0;
    int y = 0;
    bool pressed = false;
};

JoystickStatus prev_joy_stat;
JoystickStatus curr_joy_stat;

double convertRawAngleToDegrees(int rawAngle)
{
    if(rawAngle < 0) {
        rawAngle += AS5600::TOTAL_STEP_COUNT;
    }
    return rawAngle * AS5600::STEP_ANGLE;
}

void joyCallback(const std_msgs::Int32MultiArray::ConstPtr& joy)
{
    std::vector<int> data = joy->data;
    ROS_INFO("joyCallback: [%d, %d, %d]", data[0], data[1], data[2]);
    int x = (int)data[0];
    int y = (int)data[1];
    bool pressed = (bool)data[2];
    curr_joy_stat.x = x;
    curr_joy_stat.y = y;
    curr_joy_stat.pressed = pressed;
    if(!prev_joy_stat.pressed && curr_joy_stat.pressed) {
        ++pressCounter;
        if(pressCounter%2) {
            startPosition = encoder.getRawAngle();
            isStartPositionSet = true;
        } else {
            endPosition = encoder.getRawAngle();
            isEndPositionSet = true;
        }
    }
    prev_joy_stat = curr_joy_stat;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "encoder_node");
    ros::NodeHandle nh;

    encoder.setup();
    
    uint16_t prevPosition = 0;
    uint16_t currPosition = 0;

    ros::Subscriber sub = nh.subscribe("joystick_status", 10, joyCallback);

    while(ros::ok()) {
        if(!(isStartPositionSet && isEndPositionSet))
            continue;
        
        ROS_INFO("Start Position: %d", startPosition);
        ROS_INFO("End Position: %d", endPosition);
    }
    

    /*

    ros::ServiceClient motor_cmd_service = ros::ServiceClient('motor/cmd', motion_box::StringTrigger);
    // Go to Start Position
    
    
    step_number = int(d["step_number"])
    direction = True if d["direction"] == "True" else False
    step_type = str(d["step_types"]).split()[0]
    speed = float(d["speed"])
    
    data = json.dumps({
        'command': 'move', 
        'steps': step_number, 
        'direction': direction,  
        'step_type': step_type, 
        'speed': speed
    })
    

    std::string data_str = "";
    ROS_DEBUG(data_str.c_str());
    motion_box::StringTriggerRequest motor_cmd_msg;
    motor_cmd_msg.data = data_str.c_str();
    //result = motor_cmd_service(motor_cmd_msg);
    //ROS_DEBUG(result);
    */

    ros::spin();
}

