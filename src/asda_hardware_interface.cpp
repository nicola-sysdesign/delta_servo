#include "delta_servo/asda/hardware_interface/asda_hardware_interface.h"


bool delta::asda::ServoHW::fault_reset()
{
  if (ec_master.fault_reset())
  {
    reset_controllers = true;
    return true;
  }
  else
  {
    return false;
  }
}


bool delta::asda::ServoHW::fault_reset(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (fault_reset())
  {
    res.success = true;
    res.message = "Fault Reset";
  }
  else
  {
    res.success = false;
    res.message = "Failed to Fault Reset";
  }
  return true;
}


bool delta::asda::ServoHW::ready_to_switch_on()
{
  if (ec_master.ready_to_switch_on())
  {
    reset_controllers = true;
    return true;
  }
  else
  {
    return false;
  }
}


bool delta::asda::ServoHW::ready_to_switch_on(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (ready_to_switch_on())
  {
    res.success = true;
    res.message = "Ready to Switch On";
  }
  else
  {
    res.success = false;
    res.message = "Failed to enter state 'Ready to Switch On'";
  }
  return true;
}


bool delta::asda::ServoHW::switch_on()
{
  if (ec_master.switch_on())
  {
    reset_controllers = true;
    return true;
  }
  else
  {
    return false;
  }
}


bool delta::asda::ServoHW::switch_on(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (switch_on())
  {
    res.success = true;
    res.message = "Switch On";
  }
  else
  {
    res.success = false;
    res.message = "Failed to Switch On";
  }
  return true;
}


bool delta::asda::ServoHW::switch_off()
{
  if (ec_master.switch_off())
  {
    reset_controllers = true;
    return true;
  }
  else
  {
    return false;
  }
}


bool delta::asda::ServoHW::switch_off(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (switch_off())
  {
    res.success = true;
    res.message = "Switch Off";
  }
  else
  {
    res.success = false;
    res.message = "Failed to Switch Off";
  }

  return true;
}


bool delta::asda::ServoHW::start_homing()
{
  if (ec_master.start_homing())
  {
    reset_controllers = true;
    return true;
  }
  else
  {
    return false;
  }
}


bool delta::asda::ServoHW::start_homing(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (start_homing())
  {
    res.success = true;
    res.message = "Homing started...";
  }
  else
  {
    res.success = false;
    res.message = "Failed to start Homing preocedure.";
  }
  return true;
}


bool delta::asda::ServoHW::enable_operation()
{
  if (ec_master.enable_operation())
  {
    reset_controllers = true;
    return true;
  }
  else
  {
    return false;
  }
}


bool delta::asda::ServoHW::enable_operation(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (enable_operation())
  {
    res.success = true;
    res.message = "Motion started.";
  }
  else
  {
    res.success = false;
    res.message = "Failed to start motion!";
  }

  return true;
}


bool delta::asda::ServoHW::halt()
{
  if (ec_master.halt())
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool delta::asda::ServoHW::halt(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (halt())
  {
    res.success = true;
    res.message = "Halt!";
  }
  else
  {
    res.success = false;
    res.message = "Halt failed!";
  }
  return true;
}


bool delta::asda::ServoHW::quick_stop()
{
  if (ec_master.quick_stop())
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool delta::asda::ServoHW::quick_stop(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
  if (quick_stop())
  {
    res.success = true;
    res.message = "Quick Stop!";
  }
  else
  {
    res.success = false;
    res.message = "Quick Stop failed!!!";
  }

  return true;
}


bool delta::asda::ServoHW::get_error_code(const uint16 slave_idx, uint16 &error_code)
{
  for (int i = 0; i < actuators.size(); i++)
  {
    const uint16 slave_idx = 1 + i;
    ec_master.get_error_code(slave_idx, error_code);
  }
  return true;
}
