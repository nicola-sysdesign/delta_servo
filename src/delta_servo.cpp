// STL
#include <string>
#include <vector>
#include <chrono>
#include <thread>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>
// transmission_interface
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
// std_srvs
#include <std_srvs/Trigger.h>
// industrial_msgs
#include <industrial_msgs/RobotStatus.h>
// delta_servo
#include "delta_servo/asda/hardware_interface/asda_hardware_interface.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "delta_servo");

  // Node
  ros::NodeHandle node("~");

  ros::CallbackQueue callback_queue;
  node.setCallbackQueue(&callback_queue);

  // Parameters
  auto freq = node.param<double>("publish_frequency", 10);

  std::string urdf;
  if (!ros::param::get("robot_description", urdf))
  {
    ROS_FATAL("Failed to get parameter: 'robot_description'");
    return 1;
  }

  double loop_hz;
  if (!node.getParam("/rail/hardware_interface/loop_hz", loop_hz))
  {
    std::string param_name = node.resolveName("/rail/hardware_interface/loop_hz");
    ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
    return 1;
  }

  std::vector<std::string> joints;
  if (!node.getParam("/rail/hardware_interface/joints", joints))
  {
    std::string param_name = node.resolveName("/rail/hardware_interface/joints");
    ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
    return 1;
  }

  std::vector<std::string> actuators;
  if (!node.getParam("/rail/hardware_interface/actuators", actuators))
  {
    std::string param_name = node.resolveName("/rail/hardware_interface/actuators");
    ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
    return 1;
  }

  ros::AsyncSpinner spinner(2, &callback_queue);
  spinner.start();

  // Init
  delta::asda::ServoHW servo_hw(node);
  if (servo_hw.init(loop_hz, joints, actuators))
  {
    ROS_INFO("Hardware Interface initialized correctly");
  }
  else
  {
    ROS_FATAL("Failed to initialize Hardware Interface");
    return 1;
  }

  // Transmission Interface
  transmission_interface::RobotTransmissions servo_tr;
  transmission_interface::TransmissionInterfaceLoader transmission_loader(&servo_hw, &servo_tr);
  if (transmission_loader.load(urdf))
  {
    servo_hw.act_to_jnt_state_interface = servo_tr.get<transmission_interface::ActuatorToJointStateInterface>();
    servo_hw.jnt_to_act_pos_interface = servo_tr.get<transmission_interface::JointToActuatorPositionInterface>();
    ROS_INFO("Transmission Interface loaded from URDF.");
  }
  else
  {
    ROS_FATAL("Failed to load Transmission Interface from URDF");
    return 1;
  }

  // Advertised Services
  auto fault_reset_srv = node.advertiseService("fault_reset", &delta::asda::ServoHW::fault_reset, &servo_hw);
  auto ready_to_switch_on_srv = node.advertiseService("ready_to_switch_on", &delta::asda::ServoHW::ready_to_switch_on, &servo_hw);
  auto switch_on_srv = node.advertiseService("switch_on", &delta::asda::ServoHW::switch_on, &servo_hw);
  auto enable_operation_srv = node.advertiseService("enable_operation", &delta::asda::ServoHW::enable_operation, &servo_hw);
  auto halt_srv = node.advertiseService("halt", &delta::asda::ServoHW::halt, &servo_hw);
  auto quick_stop_srv = node.advertiseService("quick_stop", &delta::asda::ServoHW::quick_stop, &servo_hw);

  // Advertised Topics
  auto status_pub = node.advertise<industrial_msgs::RobotStatus>("status", 10);

  // Start
  if (servo_hw.start())
  {
    ROS_INFO("Hardware Interface started.");
  }
  else
  {
    ROS_FATAL("Failed to start Hardware Interface");
    return 1;
  }

  // Loop
  ros::Rate rate(freq);
  while (ros::ok())
  {
    rate.sleep();
    const ros::Time time = ros::Time::now();

    industrial_msgs::RobotStatus status_msg;
    status_msg.header.stamp = time;
    status_msg.mode.val = industrial_msgs::RobotMode::UNKNOWN;
    status_msg.e_stopped.val = (!servo_hw.status.quick_stop) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
    status_msg.drives_powered.val = (servo_hw.status.switched_on) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
    status_msg.motion_possible.val = (servo_hw.status.operation_enabled) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
    status_msg.in_motion.val = industrial_msgs::TriState::UNKNOWN;
    status_msg.in_error.val = (servo_hw.status.fault) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;
    status_msg.error_code = 0;

    if (servo_hw.status.fault)
    {
      for (int i = 0; i < actuators.size(); i++)
      {
        const uint16 slave_idx = 1 + i;
        uint16 error_code;
        servo_hw.get_error_code(slave_idx, error_code);
        status_msg.error_code = error_code;
      }
    }

    if (servo_hw.status.warning)
    {
      for (int i = 0; i < actuators.size(); i++)
      {
        const uint16 slave_idx = 1 + i;
        uint16 error_code;
        servo_hw.get_error_code(slave_idx, error_code);
        status_msg.error_code = error_code;
      }
    }

    status_pub.publish(status_msg);
  }


  servo_hw.close();
  return 0;
}
