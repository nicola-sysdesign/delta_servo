#ifndef DELTA_ASDA_HARDWARE_INTERFACE_H
#define DELTA_ASDA_HARDWARE_INTERFACE_H
#include <string>
#include <vector>
// UNIX
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
// roscpp
#include <ros/ros.h>
#include <ros/console.h>
// std_srvs
#include <std_srvs/Trigger.h>
// controller_manager
#include <controller_manager/controller_manager.h>
// transmission_interface
#include <transmission_interface/transmission_interface.h>
// hardware_interface
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/robot_hw.h>
// xmlrpcpp
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
// delta_servo
#include "delta_servo/asda/ethercat/master.h"
#include "delta_servo/asda/ethercat/time.h"

#define POSITION_STEP_FACTOR  100000.0
#define VELOCITY_STEP_FACTOR  100000.0


namespace delta { namespace asda {

void* control_loop(void* arg);

class ServoHW : public hardware_interface::RobotHW {
private:

  bool init_ethercat(unsigned long cycletime, const std::string &ifname, const std::vector<std::string> &slaves)
  {
    ec_master = delta::asda::ethercat::Master(cycletime, ifname, slaves);

    if (ec_master.init())
    {
      ROS_INFO("EtherCAT Master interface: %s", ifname.c_str());
    }
    else
    {
      ROS_FATAL("Failed to initialize EtherCAT master.");
      return false;
    }

    for (int i = 0; i < slaves.size(); i++)
    {
      ROS_INFO("EtherCAT Slave[%d]: %s", 1 + i, slaves[i].c_str());
    }

    return true;
  }

protected:

  ros::NodeHandle node;

  hardware_interface::ActuatorStateInterface act_state_interface;
  hardware_interface::PositionActuatorInterface pos_act_interface;
  hardware_interface::VelocityActuatorInterface vel_act_interface;
  hardware_interface::EffortActuatorInterface eff_act_interface;

  std::vector<std::string> joints;
  std::vector<std::string> actuators;

  std::vector<double> a_pos, a_pos_cmd;
  std::vector<double> a_vel, a_vel_cmd;
  std::vector<double> a_eff, a_eff_cmd;

  std::vector<double> joint_lower_limits;
  std::vector<double> joint_upper_limits;

  bool config_slaves(XmlRpc::XmlRpcValue &slaves_param)
  {
    for (int i = 0; i < slaves_param.size(); i++)
    {
      try
      {
        int position_window = slaves_param[i]["position_window"];
        int position_window_time = slaves_param[i]["position_window_time"];
        int interpolation_sub_mode = slaves_param[i]["interpolation_sub_mode"];
        int interpolation_time_period = slaves_param[i]["interpolation_time_period"];
        int following_error_window = slaves_param[i]["following_error_window"];
        int position_offset = slaves_param[i]["position_offset"];
        int position_factor[2] = { slaves_param[i]["position_factor"][0], slaves_param[i]["position_factor"][1] };
        int quickstop_deceleration = slaves_param[i]["quickstop_deceleration"];

        const uint16 slave_idx = 1 + i;
        ROS_DEBUG("EtherCAT Slave[%d] Following Error Window: %u", slave_idx, following_error_window);
        ROS_DEBUG("EtherCAT Slave[%d] Position Offset: %u", slave_idx, position_offset);
        ROS_DEBUG("EtherCAT Slave[%d] Position Factor: %u : %u", slave_idx, position_factor[0], position_factor[1]);
        ROS_DEBUG("EtherCAT Slave[%d] QuickStop Deceleration: %u", slave_idx, quickstop_deceleration);

        ec_master.config_position_interpolation(slave_idx, delta::asda::ethercat::interpolation_sub_mode_t::LINEAR_INTERPOLATION, interpolation_time_period);
        ec_master.set_following_error_window(slave_idx, following_error_window);
        ec_master.set_position_offset(slave_idx, position_offset);
        ec_master.set_position_factor(slave_idx, position_factor[0], position_factor[1]);
        ec_master.set_quickstop_deceleration(slave_idx, quickstop_deceleration);
      }
      catch (const XmlRpc::XmlRpcException &ex)
      {
        auto code = ex.getCode();
        auto message = ex.getMessage();
        ROS_ERROR("Error Code: %d, %s", code, message.c_str());
        return false;
      }
    }

    return true;
  }

public:

  double loop_hz;
  delta::asda::ethercat::Master ec_master;

  std::shared_ptr<controller_manager::ControllerManager> controller_manager;
  bool reset_controllers = true;

  // Transmission Interface
  transmission_interface::ActuatorToJointStateInterface* act_to_jnt_state_interface;
  transmission_interface::JointToActuatorPositionInterface* jnt_to_act_pos_interface;
  transmission_interface::JointToActuatorVelocityInterface* jnt_to_act_vel_interface;
  transmission_interface::JointToActuatorEffortInterface* jnt_to_act_eff_interface;

  struct {
    bool ready_to_switch_on;
    bool switched_on;
    bool operation_enabled;
    bool fault;
    bool voltage_enabled;
    bool quick_stop;
    bool switch_on_disabled;
    bool warning;
    bool remote;
    bool target_reached;
    bool internal_limit_active;
    bool homing_attained;
    bool homing_error;
    bool following_error;
  } status;


  ServoHW(ros::NodeHandle &node) :
    node(node),
    controller_manager(new controller_manager::ControllerManager(this, node)) { }


  bool init(double loop_hz, const std::vector<std::string> &joints, const std::vector<std::string> &actuators)
  {
    this->loop_hz = loop_hz;
    this->joints = joints;
    this->actuators = actuators;

    // EtherCAT
    unsigned long cycletime;
    cycletime = (1.0 / loop_hz) * 1000000000;

    std::string ifname;
    if (!node.getParam("ethercat/ifname", ifname))
    {
      std::string param_name = node.resolveName("ethercat/ifname");
      ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
      return false;
    }

    XmlRpc::XmlRpcValue slaves_param;
    if (!node.getParam("ethercat/slaves", slaves_param))
    {
      std::string param_name = node.resolveName("ethercat/slaves");
      ROS_ERROR("Failed to get '%s' parameter.", param_name.c_str());
      return false;
    }

    std::vector<std::string> slaves;
    for (int i = 0; i < slaves_param.size(); i++)
    {
      try
      {
        std::string device_name = slaves_param[i]["device_name"];
        slaves.push_back(device_name);
      }
      catch (const XmlRpc::XmlRpcException &ex)
      {
        auto code = ex.getCode();
        auto message = ex.getMessage();
        ROS_ERROR("Error Code: %d, %s", code, message.c_str());
      }
    }

    if (!init_ethercat(cycletime, ifname, slaves))
    {
      ROS_ERROR("Failed to initialize EtherCAT master");
      close();
      return false;
    }

    if (!config_slaves(slaves_param))
    {
      ROS_ERROR("Failed to configure EtherCAT slaves");
      close();
      return false;
    }

    // Hardware Interface
    const int n_actuators = actuators.size();

    a_pos.resize(n_actuators, 0.0); a_pos_cmd.resize(n_actuators, 0.0);
    a_vel.resize(n_actuators, 0.0); a_vel_cmd.resize(n_actuators, 0.0);
    a_eff.resize(n_actuators, 0.0); a_eff_cmd.resize(n_actuators, 0.0);

    for (int i = 0; i < n_actuators; i++)
    {
      hardware_interface::ActuatorStateHandle act_state_handle(actuators[i], &a_pos[i], &a_vel[i], &a_eff[i]);
      act_state_interface.registerHandle(act_state_handle);

      hardware_interface::ActuatorHandle pos_act_handle(act_state_handle, &a_pos_cmd[i]);
      pos_act_interface.registerHandle(pos_act_handle);

      hardware_interface::ActuatorHandle vel_act_handle(act_state_handle, &a_vel_cmd[i]);
      vel_act_interface.registerHandle(vel_act_handle);

      hardware_interface::ActuatorHandle eff_act_handle(act_state_handle, &a_eff_cmd[i]);
      eff_act_interface.registerHandle(eff_act_handle);
    }

    registerInterface(&act_state_interface);
    registerInterface(&pos_act_interface);
    registerInterface(&vel_act_interface);
    registerInterface(&eff_act_interface);

    return true;
  }


  bool start()
  {
    if (!ec_master.start())
    {
      return false;
    }

    pthread_t pthread;
    pthread_attr_t pthread_attr;

    errno = pthread_attr_init(&pthread_attr);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_init");
      return false;
    }

    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(1, &cpu_set);
    errno = pthread_attr_setaffinity_np(&pthread_attr, sizeof(cpu_set), &cpu_set);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_setaffinity_np");
      return 1;
    }

    errno = pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_setschedpolicy");
      return false;
    }

    errno = pthread_attr_setschedpolicy(&pthread_attr, SCHED_FIFO);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_setschedpolicy");
      return false;
    }

    sched_param sched_param
    {
      .sched_priority = 80
    };
    errno = pthread_attr_setschedparam(&pthread_attr, &sched_param);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_setschedparam");
      return false;
    }

    errno = pthread_create(&pthread, &pthread_attr, &control_loop, this);
    if (errno != 0)
    {
      ROS_FATAL("pthread_create");
      return false;
    }

    errno = pthread_attr_destroy(&pthread_attr);
    if (errno != 0)
    {
      ROS_FATAL("pthread_attr_destroy");
      return false;
    }
  }

  /* */
  bool fault_reset();
  bool fault_reset(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool ready_to_switch_on();
  bool ready_to_switch_on(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool switch_on();
  bool switch_on(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool switch_off();
  bool switch_off(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /**/
  bool enable_operation();
  bool enable_operation(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool start_homing();
  bool start_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool halt();
  bool halt(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool quick_stop();
  bool quick_stop(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* */
  bool get_error_code(const uint16 slave_idx, uint16 &error_code);


  void read(const ros::Time &time, const ros::Duration &period)
  {
    const int n_actuators = actuators.size();

    for (int i = 0; i < n_actuators; i++)
    {
      const uint16 slave_idx = 1 + i;
      uint16 status_word = ec_master.tx_pdo[slave_idx].status_word;
      int32 actual_position = ec_master.tx_pdo[slave_idx].actual_position;

      status.ready_to_switch_on = (status_word >> 0) & 0x01;
      status.switched_on = (status_word >> 1) & 0x01;
      status.operation_enabled = (status_word >> 2) & 0x01;
      status.fault = (status_word >> 3) & 0x01;
      status.voltage_enabled = (status_word >> 4) & 0x01;
      status.quick_stop = (status_word >> 5) & 0x01;
      status.switch_on_disabled = (status_word >> 6) & 0x01;
      status.warning = (status_word >> 7) & 0x01;
      status.remote = (status_word >> 9) & 0x01;
      status.target_reached = (status_word >> 10) & 0x01;
      status.internal_limit_active = (status_word >> 11) & 0x01;

      if (!status.operation_enabled)
      {
        reset_controllers = true;
      }

      // switch (mode_of_operation_display)
      // {
      //   case delta::asda::ethercat::mode_of_operation_t::HOMING:
      //     status.homing_attained = (status_word >> 12) & 0x01;
      //     status.homing_error = (status_word >> 13) & 0x01;
      //     break;
      //   case delta::asda::ethercat::mode_of_operation_t::CYCLIC_SYNCHRONOUS_POSITION:
      //     status.following_error = (status_word >> 13) & 0x01;
      //     break;
      //   case delta::asda::ethercat::mode_of_operation_t::CYCLIC_SYNCHRONOUS_VELOCITY:
      //     status.following_error = (status_word >> 13) & 0x01;
      //     break;
      //   case delta::asda::ethercat::mode_of_operation_t::CYCLIC_SYNCHRONOUS_TORQUE:
      //     status.following_error = (status_word >> 13) & 0x01;
      //     break;
      // }

      a_pos[i] = actual_position / POSITION_STEP_FACTOR;
    }
  }


  void write(const ros::Time &time, const ros::Duration &period)
  {
    const int n_actuators = actuators.size();

    for (int i = 0; i < n_actuators; i++)
    {
      const uint16 slave_idx = 1 + i;
      uint32 target_position = a_pos_cmd[i] * POSITION_STEP_FACTOR;

      ec_master.rx_pdo[slave_idx].target_position = target_position;
    }

    ec_master.update();
  }


  void close()
  {
    ec_master.close();
    ROS_INFO("EtherCAT socket closed.");
  }

};


inline void* control_loop(void* arg)
{
  delta::asda::ServoHW* servo_hw = (delta::asda::ServoHW*)arg;
  servo_hw->reset_controllers = true;

  struct timespec t, t_1;
  clock_gettime(CLOCK_MONOTONIC, &t);

  while (ros::ok())
  {
    delta::asda::ethercat::add_timespec(&t, servo_hw->ec_master.t_cycle + servo_hw->ec_master.t_off);

    struct timespec t_left;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, &t_left);

    struct timespec t_period;
    delta::asda::ethercat::diff_timespec(t, t_1, &t_period);

    if (delta::asda::ethercat::to_nsec(t_period) < (servo_hw->ec_master.t_cycle - 100000) || (servo_hw->ec_master.t_cycle + 100000) < delta::asda::ethercat::to_nsec(t_period))
    {
      ROS_WARN("EtherCAT Master period: %lu [ns]", delta::asda::ethercat::to_nsec(t_period));
    }

    const ros::Time now = ros::Time::now();
    const ros::Duration period(delta::asda::ethercat::to_sec(t_period));

    servo_hw->read(now, period);
    servo_hw->act_to_jnt_state_interface->propagate();

    servo_hw->controller_manager->update(now, period, servo_hw->reset_controllers);
    servo_hw->reset_controllers = false;

    servo_hw->jnt_to_act_pos_interface->propagate();
    // servo_hw->jnt_to_act_vel_interface->propagate();
    // servo_hw->jnt_to_act_eff_interface->propagate();
    servo_hw->write(now, period);

    t_1 = t;
  }
}


} }  // namespace
#endif
