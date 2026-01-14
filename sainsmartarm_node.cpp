//------------------------------------------------------------------------------
//
// Title  : A node of six dof arm for sainsmart
// Program Name : sainsmart_node.cpp
//
// Detail : this is a node of six dof arm for sainsmart.
//          subscribe /joint_state
//
// Date   : 2026/01/14
// Author : Akihiro Kashiwagi
// e-mail : a-kashiwagi@hotmail.com
//
// Replace ---------------------------------------------------------------------
//
// Date   : 
// Author : 
// Detail : 
//
//------------------------------------------------------------------------------

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <iostream>
#include <iomanip>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int fd;
                                       // File descriptor
double joint[6];
                                       // Array of joints
char command_str[256];
                                       // Command string

                                       // Adjustment (degree)
double adjust[6] = {
     0,                                // For joint 1 
     0,                                // For joint 2
  -110,                                // For joint 3
     0,                                // For joint 4
    30,                                // For joint 5
     2                                 // For joint 6
};

char buffer[256];
                                       // Buffer of string
#define INVERT (-1)
                                       // Invert const
#define JOINT_NUM 6
                                       // Number of jonts
#define DEVICE_NAME "/dev/ttyACM0"
                                       // Device name
#define SERIAL_SPEED B19200
                                       // Serial speed

//------------------------------------------------------------------------------
//
// Title  : Convert radian to degree
//
// Function Name : RadToDeg
//
// Detail : Convert radian to degree
//
// Date   : 2026/01/14
// Author : Akihiro Kashiwagi
// e-mail : a-kashiwagi@hotmail.com
//
// Replace ---------------------------------------------------------------------
//
// Date   : 
// Author : 
// Detail : 
//
//------------------------------------------------------------------------------

double RadToDeg(double radian){
                                       // Convert radian to degree
    return atan2(sin(radian), cos(radian)) * 180.0 / M_PI;
}

//------------------------------------------------------------------------------
//
// Title  : A callback function of JointState
//
// Function Name : jointStateCallback
//
// Detail : Subscribe /joint_states and sending a digree to sarvos.
//
// Date   : 2026/01/14
// Author : Akihiro Kashiwagi
// e-mail : a-kashiwagi@hotmail.com
//
// Replace ---------------------------------------------------------------------
//
// Date   : 
// Author : 
// Detail : 
//
//------------------------------------------------------------------------------

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {

    size_t n = msg->name.size();
                                       // number of names
    if (msg->position.size() != n ) {
                                       // Check for number of array
        ROS_WARN("/joint_states message has different sizes.");
        return;
    }

    if (n < JOINT_NUM ) {
                                       // Check for number of joints
        return;
    }

    for (size_t i = 0; i < n; i++) {

        strcpy(buffer,msg->name[i].c_str());
                                       // Copy name to buffer
        // debug write
        //printf("name[%d]:%s",n,buffer);
        //continue;
                                       // Convert degree to radian
        if( strcmp(buffer,"joint1") == 0){
                                       // for joint1
            joint[0] = (RadToDeg(msg->position[i]) + adjust[0]);

        }else if( strcmp(buffer,"joint2") == 0){
                                       // for joint2
            joint[1] = (RadToDeg(msg->position[i]) + adjust[1]) * INVERT;

        }else if( strcmp(buffer,"joint3") == 0){
                                       // for joint3
            joint[2] = (RadToDeg(msg->position[i]) + adjust[2]) - joint[1];

        }else if( strcmp(buffer,"joint4") == 0){
                                       // for joint4
            joint[3] = (RadToDeg(msg->position[i]) + adjust[3]);

        }else if( strcmp(buffer,"joint5") == 0){
                                       // for joint5
            joint[4] = (RadToDeg(msg->position[i]) + adjust[4]);

        }else if( strcmp(buffer,"joint6") == 0){
                                       // for joint6
            joint[5] = (RadToDeg(msg->position[i]) + adjust[5]);

        }
    }

    for(int i=0; i<=5; i++){
                                       // Convert to decimal point
        joint[i] = round(joint[i] * 10) * 0.1;
    }
                                       // Make a command for sarvos
    sprintf(command_str,"%.1f %.1f %.1f %.1f %.1f %.1fc",
        joint[0],joint[1],joint[2],
        joint[3],joint[4],joint[5]    
    );
                                       // Send to servos
    ssize_t ret = write(fd, command_str, strlen(command_str));
    if (ret < 0) {
        perror("send to servos errors.");
        return;
    }
                                       // Print logs
    ROS_INFO_STREAM(
        "Received JointState with " << n << " joints:" 
        << command_str
    );
}

//------------------------------------------------------------------------------
//
// Title  : main function
//
// Function Name : main
//
// Detail : main function of a node of six dof arm for sainsmart.
//
// Date   : 2026/01/14
// Author : Akihiro Kashiwagi
// e-mail : a-kashiwagi@hotmail.com
//
// Replace ---------------------------------------------------------------------
//
// Date   : 
// Author : 
// Detail : 
//
//------------------------------------------------------------------------------

int main(int argc, char** argv) {

    const char *portname = DEVICE_NAME;
                                       // Device name of Serial port
    struct termios tty;
                                       // termios for serial port

                                       // Open a serial port
    fd = open(portname, O_RDWR | O_NOCTTY);
    if (fd < 0) {
                                       // Error terminate
        fprintf(stderr, "Can't open(%s): %s\n", portname, strerror(errno));
        return 1;
    }

    if (tcgetattr(fd, &tty) != 0) {
                                       // Get tty attribute 
        perror("Can't get a tcgetattr.");
        close(fd);
        return 1;
    }
                                       // Set serial speed
    cfsetospeed(&tty, SERIAL_SPEED);
    cfsetispeed(&tty, SERIAL_SPEED);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
                                       // Set tty attribute
        perror("tcsetattr");
        close(fd);
        return 1;
    }
                                       // Buffer flush
    tcflush(fd, TCIOFLUSH);

                                       // Initialize ROS node
    ros::init(argc, argv, "joint_state_subscriber");
    ros::NodeHandle nh;

                                       // Subscribe to /joint_states topic
    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>(
        "/joint_states", 10, jointStateCallback);

    ROS_INFO("JointState subscriber started. Waiting for messages...");
                                       // Output information
    ros::Rate loop_rate(4);
                                       // Rate of spin (Hz)
    while (ros::ok()) {
                                       // ROS spin
        ros::spinOnce();
        loop_rate.sleep();
                                       // process callbacks
    }

    close(fd);
                                       // Close serial port
    return 0;
                                       // Normal terminate
}
