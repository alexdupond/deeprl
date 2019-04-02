#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "dynamixel_driver/SetDynamixelPositions.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <bitset>
#include <string>

#include "/home/quist/bin/DynamixelSDK/c++/include/dynamixel_sdk/dynamixel_sdk.h"                                 // Uses Dynamixel SDK library
// https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/master/c%2B%2B/src/dynamixel_sdk

// Control table address   Control table -> http://support.robotis.com/en/product/actuator/dynamixel_x/xm_series/xm430-w350.html
#define ADDR_PRO_TORQUE_ENABLE          64
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_OPERATING_MODE         11
#define ADDR_PRO_GOAL_VEL               104
#define ADDR_PRO_PRESENT_VEL            128

// Data Byte Length
#define LEN_PRO_GOAL_VEL                 4
#define LEN_PRO_PRESENT_VEL              4
#define LEN_PRO_PRESENT_POSITION         4

// Protocol version
#define PROTOCOL_VERSION                2.0

// Default setting
#define DXL1_ID                         1
#define DXL2_ID                         2
#define BAUDRATE                        4000000
#define DEVICENAME                      "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     25                  // Dynamixel moving status threshold
#define VEL_CONTROL_MODE                1

// Init Port Handler and Packet Handler
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// Initialize GroupSyncWrite instance
dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_VEL, LEN_PRO_GOAL_VEL);

// Initialize Groupsyncread instance for Present Position
dynamixel::GroupSyncRead groupSyncRead_Pos(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
dynamixel::GroupSyncRead groupSyncRead_Vel(portHandler, packetHandler, ADDR_PRO_PRESENT_VEL, LEN_PRO_PRESENT_VEL);

int32_t syncRead(dynamixel::GroupSyncRead &gsr, uint8_t id, uint16_t addr, u_int16_t dataLen);
void syncWrite(dynamixel::GroupSyncWrite &gsw, uint8_t id, uint32_t data);
void setTorque(dynamixel::PacketHandler *paH, dynamixel::PortHandler *poH, uint8_t data);
void initRobot(dynamixel::PacketHandler *paH, dynamixel::PortHandler *poH, dynamixel::GroupSyncRead &gsrPos, dynamixel::GroupSyncRead &gsrVel);

void setDynamixelCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  syncWrite(groupSyncWrite, DXL1_ID, msg->data[0]);
  syncWrite(groupSyncWrite, DXL2_ID, msg->data[1]);

  // Syncwrite goal position
  int dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf(packetHandler->getTxRxResult(dxl_comm_result));

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}

bool setDynamixelService(dynamixel_driver::SetDynamixelPositions::Request  &req, dynamixel_driver::SetDynamixelPositions::Response &res)
{
  // Use input data req.inputPos[i] to get motor positions sendt

  // Response with a requst of some type res.outputPos[i]

  // Return true when ready to send data back
   return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_driver");

  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("dynamixel_set_positions_service", setDynamixelService);
  ros::Subscriber sub = n.subscribe("dynamixel_set_velocity", 1000, setDynamixelCallback);
  ros::Publisher pub_pos = n.advertise<std_msgs::Float32MultiArray>("dynamixel_current_position", 1000);
  ros::Publisher pub_vel = n.advertise<std_msgs::Float32MultiArray>("dynamixel_current_velocity", 1000);

  ros::Rate loop_rate(10);

  int dxl1_target_vel = 0, dxl2_target_vel = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int32_t dxl1_present_position = 0, dxl2_present_position = 0, dxl1_present_vel = 0, dxl2_present_vel = 0;

  try
  {
     initRobot(packetHandler, portHandler, groupSyncRead_Pos, groupSyncRead_Vel);
  }
  catch (const char* error_msg)
  {
     std::cerr << error_msg << std::endl;
     return 0;
  }

  setTorque(packetHandler, portHandler, TORQUE_ENABLE);

  while (ros::ok())
  {
  try
       {    // Syncread present position
           dxl_comm_result = groupSyncRead_Pos.txRxPacket();
           if (dxl_comm_result != COMM_SUCCESS) printf(packetHandler->getTxRxResult(dxl_comm_result));
           dxl1_present_position = syncRead(groupSyncRead_Pos, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
           dxl2_present_position = syncRead(groupSyncRead_Pos, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
           // Syncread present velocity
           dxl_comm_result = groupSyncRead_Vel.txRxPacket();
           if (dxl_comm_result != COMM_SUCCESS) printf(packetHandler->getTxRxResult(dxl_comm_result));
           dxl1_present_vel = syncRead(groupSyncRead_Vel, DXL1_ID, ADDR_PRO_PRESENT_VEL, LEN_PRO_PRESENT_VEL);
           dxl2_present_vel = syncRead(groupSyncRead_Vel, DXL2_ID, ADDR_PRO_PRESENT_VEL, LEN_PRO_PRESENT_VEL);
       }
       catch (const char* error_msg){
           std::cerr << error_msg << std::endl;
           break;
  }

  // Maybe publish current motor position every 10 Hz
  // Create msg
  std_msgs::Float32MultiArray msg_pos;
  std_msgs::Float32MultiArray msg_vel;
  // push in the data
  msg_pos.data.push_back(dxl1_present_position);
  msg_pos.data.push_back(dxl2_present_position);
  msg_vel.data.push_back(dxl1_present_vel);
  msg_vel.data.push_back(dxl2_present_vel);
  // publish to topic
  pub_pos.publish(msg_pos);
  pub_vel.publish(msg_vel);

  ros::spinOnce();
  loop_rate.sleep();
  }

  setTorque(packetHandler, portHandler, TORQUE_DISABLE);
  portHandler->closePort();
  return 0;
}

int32_t syncRead(dynamixel::GroupSyncRead &gsr, uint8_t id, uint16_t addr, u_int16_t dataLen)
{
    // Check if groupsyncread data of is available
    bool dxl_getdata_result = gsr.isAvailable(id, addr, dataLen);
    if (dxl_getdata_result != true)
        throw "SyncRead Failed";
    else
        return gsr.getData(id, addr, dataLen);
}

void syncWrite(dynamixel::GroupSyncWrite &gsw, uint8_t id, uint32_t data)
{
    uint8_t dxl_param[4];
    // Allocate goal position value into byte array
    dxl_param[0] = DXL_LOBYTE(DXL_LOWORD(data));
    dxl_param[1] = DXL_HIBYTE(DXL_LOWORD(data));
    dxl_param[2] = DXL_LOBYTE(DXL_HIWORD(data));
    dxl_param[3] = DXL_HIBYTE(DXL_HIWORD(data));

    // Add Dynamixel#1 goal vel value to the Syncwrite storage
    bool dxl_addparam_result = gsw.addParam(id, dxl_param);
    if (dxl_addparam_result != true) {
        throw "SyncWrite Failed";
    }
}

void setTorque(dynamixel::PacketHandler *paH, dynamixel::PortHandler *poH, uint8_t data)
{
    // Enable Dynamixel#1 Torque
    paH->write1ByteTxOnly(poH, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, data);
    // Enable Dynamixel#2 Torque
    paH->write1ByteTxOnly(poH, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, data);
}

void initRobot(dynamixel::PacketHandler *paH, dynamixel::PortHandler *poH, dynamixel::GroupSyncRead &gsrPos, dynamixel::GroupSyncRead &gsrVel)
{
    // Open port
    if (poH->openPort())
        std::cout << "Port opened successfully" << std::endl;
    else {
        throw "Failed to open port";
    }

    // Set port baudrate
    if (poH->setBaudRate(BAUDRATE))
        std::cout << "Baudrate set to: " << BAUDRATE << std::endl;
    else {
        throw "Failed to set Baudrate";
    }

    // Add parameter storage for Dynamixel#1 present position value
    bool dxl_addparam_result = gsrPos.addParam(DXL1_ID);
    if (!dxl_addparam_result)
        throw "addparam failed";

    // Add parameter storage for Dynamixel#2 present position value
    dxl_addparam_result = gsrPos.addParam(DXL2_ID);
    if (!dxl_addparam_result)
        throw "addparam failed";


    // Add parameter storage for Dynamixel#1 present velocity value
    dxl_addparam_result = gsrVel.addParam(DXL1_ID);
    if (!dxl_addparam_result)
        throw "addparam failed";


    // Add parameter storage for Dynamixel#2 present velocity value
    dxl_addparam_result = gsrVel.addParam(DXL2_ID);
    if (!dxl_addparam_result)
        throw "addparam failed";

    uint8_t dxl_error = 0;
    // Enable Velocity Controlmode #1
    paH->write1ByteTxRx(poH, DXL1_ID, ADDR_PRO_OPERATING_MODE, VEL_CONTROL_MODE, &dxl_error);
    if (dxl_error != COMM_SUCCESS)
        throw paH->getRxPacketError(dxl_error);

    // Enable Velocity Controlmode #2
    paH->write1ByteTxRx(poH, DXL2_ID, ADDR_PRO_OPERATING_MODE, VEL_CONTROL_MODE, &dxl_error);
    if (dxl_error != COMM_SUCCESS)
        throw paH->getRxPacketError(dxl_error);
}
