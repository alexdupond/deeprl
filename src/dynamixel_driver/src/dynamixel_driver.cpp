#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "dynamixel_driver/SetDynamixelPositions.h"

void setDynamixelCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  // Insert code to set the dynamixel motors
  for(int i = 0; i < msg->data.size(); i++){
      ROS_INFO("I heard: [%f]", msg->data[i]);
  }
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
  ros::Subscriber sub = n.subscribe("dynamixel_set_positions", 1000, setDynamixelCallback);
  ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("dynamixel_set_positions", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // Maybe publish current motor position every 10 Hz 
    // Create msg 
    std_msgs::Float32MultiArray msg; 
    // push in the data 
    msg.data.push_back(0.50); 
    msg.data.push_back(0.75); 
    msg.data.push_back(0.99); 
   
    // publish to topic
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
