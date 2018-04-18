#ifndef PROJECT_EMG_H
#define PROJECT_EMG_H

#include <ros/ros.h>

#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>

class AHKeyboard
{
 public:
  AHKeyboard();
  void keyLoop();
  void printUsage();

 private:
  ros::NodeHandle nh_;
  int count_;
  float edit_;
  ros::Publisher cmd_pub_;
  ros::Publisher vel_pub_;
};

void quit(int sig);

#endif //PROJECT_EMG_H
