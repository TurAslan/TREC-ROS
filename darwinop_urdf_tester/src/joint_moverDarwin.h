#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <boost/algorithm/string.hpp>
#include <iostream>

void initDarwin(sensor_msgs::JointState &joints);
void move_joint_to_target(sensor_msgs::JointState &joint, std::string name, double target);
void jointToPos(const std::vector<std::string> &s, sensor_msgs::JointState &joint);
int find(sensor_msgs::JointState &joint, std::string seek);

