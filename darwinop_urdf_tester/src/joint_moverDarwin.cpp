#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include "ros/transport_hints.h"

void initDarwin(sensor_msgs::JointState &joints);
int find(sensor_msgs::JointState &joint, std::string seek);
void move_joint_to_target(sensor_msgs::JointState &joint,
                          std::string name, double target);
void jointToPos(const std::vector<std::string> &s,
                      sensor_msgs::JointState &joint);

// Defining a sensor_msgs message to be published in rostopic
sensor_msgs::JointState joint_state;
/*
This function is called whenever the nodehandle sees a new message in rostopic.
Right now, it just parses the read values from the ethercat_master to the
rostopic, so that ethercat_master.cpp and RVIZ can use them.
*/
void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  //ROS_INFO("inside chatterCallback");
  for(int i=0;i<24;i++){
    joint_state.position[i] = msg->position[i];
  }
}

int main(int argc, char** argv) {

  std::string inputText;

  // this is used only when taking inputs from the terminal
  std::vector<std::string> splitString;

  // initialize ros
  ros::init(argc, argv, "joint_moverDarwin");
  ros::NodeHandle n;

  // creating a pub to advertise joint_states into rostopic
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  // creating a sub to subscribe to read_joint_states from rostopic
  ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("read_joint_states", 1000, chatterCallback, ros::TransportHints().udp());

  ros::Rate loop_rate(200); // 200Hz

  // this is used a wait buffer
  ROS_WARN_STREAM("Enter 0 to initialize darwin");
  std::getline(std::cin,value);
  joint_state.header.stamp = ros::Time::now();
  initDarwin(joint_state);
  joint_pub.publish(joint_state);

  double increment = 0.002, pos;
  while (ros::ok()) {
    joint_state.header.stamp = ros::Time::now();

/* Commented section A-------------------------------------------------------
This was used as enter goal positions the motor should move to

    ROS_WARN_STREAM("Enter the position: ");
    std::getline(std::cin,inputText);
    boost::split(splitString, inputText, [](char c){return c == ' ';});
    jointToPos(splitString, joint_state);

------------------------------------------------------------------------*/

/* Commented section B
This is used to have the motors rotate

    if ( (joint_state.position[3] > 1.6) || (joint_state.position[3] < -1.6) ) {
      increment*=-1;
    }
    joint_state.position[3]+=increment;
    std::cin>>pos;
    joint_state.position[3] = pos;
    joint_state.position[8] = -pos;

-----------------------------------------------------------------------------*/

    joint_pub.publish(joint_state);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

/*
This function is used to find the index of the joint using a string inside
joint_states
*/
int find(sensor_msgs::JointState &joint, std::string seek){
  for (int i=0; i<joint.name.size() ; ++i)  {
    if (joint.name[i] == seek) return i;
  }
  return -1;
}

/*
This is a shortcut function that will update the position of the desired joint
in joint_states
*/
void move_joint_to_target(sensor_msgs::JointState &joint, std::string name, double target){
  int index = find(joint, name);
  joint.position[index] = target;
  ROS_WARN_STREAM(index);

}

/*
This is a function that takes a string with the following syntax:
              move {joint name} to {goal position}
where {joint_name}    is the desired joint
      {goal position} is the desired goal position that is converted into a double
  A list of joints is available in the URDF or in the void initDarwin function

This function is used in the commented section A, where the already-split text
inputted in the terminal is processed here
*/
void jointToPos(const std::vector<std::string> &s,
                      sensor_msgs::JointState &joint){

  move_joint_to_target(joint, s[1], std::stod(s[3]));

}

/*
This function is nessesary to call before publishing joint_states into
rostopic. When the joints are not declared, RVIZ does not place them in
the visualization.
For now, its hardcoded with the non-fixes joints for DarwinOP. There is
a way of automatically having all joints initialized using the URDF, but
I was not able to find it.
The /opt/ros/melodic/lib/joint_state_publisher/joint_state_publisher.py
has a way of initializing all of the joints automatically but is written
in python.
*/
void initDarwin(sensor_msgs::JointState &joints){


  joints.name.resize(24);
  joints.position.resize(24);

  joints.name[0] ="j_pan";
  joints.position[0] = 0;
  joints.name[1] ="j_tilt";
  joints.position[1] = 0;
  joints.name[2] ="j_shoulder_l";
  joints.position[2] = 0;
  joints.name[3] ="j_high_arm_l";
  joints.position[3] = 0;
  joints.name[4] ="j_low_arm_l";
  joints.position[4] = 0;
  joints.name[5] ="j_wrist_l";
  joints.position[5] = 0;
  joints.name[6] ="j_gripper_l";
  joints.position[6] = 0;
  joints.name[7] ="j_shoulder_r";
  joints.position[7] = 0;
  joints.name[8] ="j_high_arm_r";
  joints.position[8] = 0;
  joints.name[9] ="j_low_arm_r";
  joints.position[9] = 0;
  joints.name[10] ="j_wrist_r";
  joints.position[10] = 0;
  joints.name[11] ="j_gripper_r";
  joints.position[11] = 0;
  joints.name[12] ="j_pelvis_l";
  joints.position[12] = 0;
  joints.name[13] ="j_thigh1_l";
  joints.position[13] = 0;
  joints.name[14] ="j_thigh2_l";
  joints.position[14] = 0;
  joints.name[15] ="j_tibia_l";
  joints.position[15] = 0;
  joints.name[16] ="j_ankle1_l";
  joints.position[16] = 0;
  joints.name[17] ="j_ankle2_l";
  joints.position[17] = 0;
  joints.name[18] ="j_pelvis_r";
  joints.position[18] = 0;
  joints.name[19] ="j_thigh1_r";
  joints.position[19] = 0;
  joints.name[20] ="j_thigh2_r";
  joints.position[20] = 0;
  joints.name[21] ="j_tibia_r";
  joints.position[21] = 0;
  joints.name[22] ="j_ankle1_r";
  joints.position[22] = 0;
  joints.name[23] ="j_ankle2_r";
  joints.position[23] = 0;

}
