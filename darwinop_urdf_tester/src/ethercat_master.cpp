#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include "ros/transport_hints.h"
#include "math.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatsoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"


#define EC_TIMEOUTMON 500
#define LOOSENED 0
#define CLOSED 1

long LONGBYTE = pow(2,16)-1;

int doubleToHigher(double var);
int doubleToLower(double var);
double splitToDouble(unsigned long lower, unsigned long higher, int ID);
int changeID(int ID);

char IOmap[4096];
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

//ROS stuff starts here---------------------------------------------------------


static double joint_offset[] = {M_PI, M_PI, // j_pan, j_tilt
  -M_PI, -3.97, -1.55, 0.0, 0.0, // left arm
  M_PI, 3.97, 1.55, 0.0, 0.0, // right arm
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

static int joint_direction[] = {1, 1, // j_pan, j_tilt
    1, 1, 1, 1, 1, // left arm
    1, -1, 1, 1, 1, // right arm
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
/*
The same function in joint_moverDarwin, needed in order to publish the read
joints
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
/*
This function takes the split bytes and combines them into a long
*/
double splitToDouble(unsigned long lower, unsigned long higher, int ID){
  unsigned long long var = (higher << 8) | lower;
  std::cout<<"VAR: "<<var<<'\n';
  return joint_direction[ID]*double(2*var*M_PI/LONGBYTE - joint_offset[ID]);
}

/*
This function takes a double and splits it to the lower byte
*/
int doubleToLower(double var){
  int x = (var+1.6)*LONGBYTE/3.2;
  return x & 0xFF;
}

/*
This function takes a double and splits it to the higher byte
*/
int doubleToHigher(double var){
  int x = (var+1.6)*LONGBYTE/3.2;
  std::cout<<"Inside Higher: "<<x<<'\n';
  return (x & 0xFF00) >> 8;
}

/*
This function is needed to change the indexing of the default joints in
order to work with the Arduino code developed by Nas, the IHMC developer.
*/
int changeID(int ID){
  switch(ID){
    case 0: // tilt
      return 1;

    case 1: // j_shoulder_r
      return 7;

    case 2: // j_shoulder_l
      return 2;

    case 3: // j_high_arm_r
      return 8;

    case 4: // j_high_arm_l
      return 3;

    case 5: // j_low_arm_r
      return 9;

    case 6: // j_low_arm_l
      return 4;

    case 7: // j_pan
      return 0;
    default:
      break;
    }
}

/*
This function is called whenever the nodehandle sees a new message in rostopic.
It is used to send the goal positions from joint_states to the Arduino
There is a syntax which is:
              [motor ID][State][Higher Byte][Lower Byte]
where [State] is either LOOSENED or CLOSED
*/
void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
//  ROS_INFO("inside chatterCallback");

  if(wkc >= expectedWKC)
  {
    //ec_slave[0].outputs[0] = doubleToHigher(msg->position[3]);
    //ec_slave[0].outputs[3] = doubleToLower(msg->position[3]);
    //ec_slave[0].outputs[1] = LOOSENED;
    //ec_slave[0].outputs[1] = CLOSED;


    //std::cout<<"Higher = "<<unsigned(ec_slave[0].outputs[0])<< std::endl;
    // std::cout<<"Lower = "<<unsigned(ec_slave[0].outputs[3])<< std::endl;
    // std::cout<<"Value = "<<( (doubleToHigher(msg->position[3]) << 8) | doubleToLower(msg->position[3]) )<< std::endl;
  }
}
//ROS stuff end here------------------------------------------------------------

void simpletest(char *ifname)
{
    int i, j, oloop, iloop, wkc_count, chk;
    needlf = FALSE;
    inOP = FALSE;
    ros::NodeHandle n;

   printf("Starting simple test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */


       if ( ec_config_init(FALSE) > 0 )
      {
         printf("%d slaves found and configured.\n",ec_slavecount);

         ec_config_map(&IOmap);

         ec_configdc();

         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
         if (oloop > 8) oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
         if (iloop > 8) iloop = 8;

         printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* send one valid process data to make outputs in slaves happy*/
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         /* request OP state for all slaves */
         ec_writestate(0);
         chk = 40;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         }
         while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            printf("Operational state reached for all slaves.\n");
            wkc_count = 0;
            inOP = TRUE;
                /* cyclic loop */
//ROS stuff starts here---------------------------------------------------------

            ros::Rate r(200); // 200 hz

            // creating a sub to subscribe to read_joint_states from rostopic
            ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("joint_states", 1000, chatterCallback, ros::TransportHints().udp());
            // creating a pub to advertise joint_states into rostopic
            ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("read_joint_states", 1);

            // Defining a sensor_msgs message to be published in rostopic
            sensor_msgs::JointState joint_state;

            // Initialising Darwin
            initDarwin(joint_state);
            int totalIDs = 8;
            while (ros::ok())  {
               //SOEM stuff----------------------------------------------------
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET); // workcounter
               //SOEM ---------------------------------------------------------

               // takes the two split bytes from the arduino and combines them
               // and stores into the joint_state, ready to publish to rostopic
               for (int ID = 0; ID<totalIDs; ID++){
                 joint_state.position[changeID(ec_slave[0].inputs[ID*3])] =
                        splitToDouble(ec_slave[0].inputs[ID*3+2],
                                      ec_slave[0].inputs[ID*3+1], changeID(ID));
               }

               // Publishes joint_state to rostopic
               joint_state.header.stamp = ros::Time::now();
               joint_pub.publish(joint_state);
               ros::spinOnce();
               r.sleep();
             }
//ROS stuff end here------------------------------------------------------------

                inOP = FALSE;
          }
          else
          {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "ethercat_master");
   int iret1;
   printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");
   std::cout<<"argv[0] = "<<argv[0]<<" argv[1] = "<<argv[1];
   if (argc > 1)
   {
      /* start cyclic part */
      simpletest(argv[1]);
   }
   else
   {
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
   }

   printf("End program\n");
   return (0);
}
