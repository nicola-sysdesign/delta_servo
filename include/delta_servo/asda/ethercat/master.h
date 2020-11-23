#ifndef DELTA_ASDA_ETHERCAT_MASTER_H
#define DALTA_ASDA_ETHERCAT_MASTER_H
// STL
#include <string>
#include <vector>
// Boost
#include <boost/variant.hpp>
// soem
#include "ethercat.h"
// delta_servo
#include "delta_servo/asda/ethercat/common.h"
#include "delta_servo/asda/ethercat/registry_idx.h"
#include "delta_servo/asda/ethercat/pdo.h"


namespace delta { namespace asda { namespace ethercat {


inline int slave_setup(uint16 slave_idx)
{
  int wkc = 0;

  // Sync Managers mapping
  uint16 sdo_1c12[] = { 0x01, 0x1601 };
  uint16 sdo_1c13[] = { 0x01, 0x1A01 };
  wkc += writeSDO<uint16>(slave_idx, 0x1c12, 0x00, sdo_1c12[0]);
  wkc += writeSDO<uint16>(slave_idx, 0x1c13, 0x00, sdo_1c13[0]);
  wkc += writeSDO<uint16>(slave_idx, 0x1c12, 0x01, sdo_1c12[1]);
  wkc += writeSDO<uint16>(slave_idx, 0x1c13, 0x01, sdo_1c13[1]);

  // Sync Managers synchronization
  uint16 sdo_1c32[] = { 0x20, 0x02 };
  uint16 sdo_1c33[] = { 0x20, 0x02 };
  wkc += writeSDO<uint16>(slave_idx, 0x1c32, 0x01, sdo_1c32[1]);
  wkc += writeSDO<uint16>(slave_idx, 0x1c33, 0x01, sdo_1c33[1]);

  return wkc;
}


class Master {
private:
  int ec_state = EC_STATE_NONE;

  const static size_t MAX_IO_MAP_SIZE = 4096;
  uint8 io_map[MAX_IO_MAP_SIZE];

  std::string ifname;
  std::vector<std::string> slaves;


  bool network_configuration()
  {
    for (int i = 0; i < slaves.size(); i++)
    {
      const uint16 slave_idx = 1 + i;
      if (strcmp(ec_slave[slave_idx].name, slaves[i].c_str()))
      {
        return false;
      }
    }
    return true;
  }

public:
  int wkc = 0;
  delta::asda::ethercat::pdo::RxPDO2 rx_pdo[10];
  delta::asda::ethercat::pdo::TxPDO2 tx_pdo[10];

  Master() { }

  Master(const std::string &ifname, const std::vector<std::string> &slaves) :
    ifname(ifname), slaves(slaves) { }


  bool init()
  {
    if (ec_init(ifname.c_str()) > 0)
    {
      printf("EtherCAT socket on: %s\n", ifname.c_str());
    }
    else
    {
      printf("Coludn't initialize EtherCAT Master socket on: %s\n", ifname.c_str());
      return false;
    }

    if (ec_config_init(FALSE) > 0)
    {
      printf("Slaves found and configured: %d\n", ec_slavecount);
    }
    else
    {
      printf("Coludn't find and configure any slave.\n");
      return false;
    }

    ec_state = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    print_ec_state(0);

    // Network Configuration
    if (!network_configuration())
    {
      printf("Mismatch of network units!\n");
      return false;
    }

    // Distributed Clock
    ec_configdc();
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      ec_dcsync0(slave_idx, TRUE, 2000000U, 0);
    }

    // Pre-Operational -> Safe-Operational
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      ec_slave[slave_idx].PO2SOconfig = slave_setup;
    }

    int used_mem = ec_config_map(&io_map);
    if (used_mem > sizeof(io_map))
    {
      printf("IO Map size: %d > MAX_IO_MAP_SIZE: %lu\n", used_mem, sizeof(io_map));
      return false;
    }
    printf("IO Map size: %d\n", used_mem);

    // print slaves configuration
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      print_sm(slave_idx, 0);     // SM0
      print_sm(slave_idx, 1);     // SM1
      print_sm(slave_idx, 2);     // SM2 (output)
      print_sm(slave_idx, 3);     // SM3 (input)
      print_fmmu(slave_idx, 0);   // FMMU0
      print_fmmu(slave_idx, 1);   // FMUU1
    }

    // SAFE OPERATIONAL
    ec_state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    print_ec_state(0);

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      fault_reset(slave_idx);
      set_mode_of_operation(slave_idx, mode_of_operation_t::CYCLIC_SYNCHRONOUS_POSITION);
      config_position_interpolation(slave_idx, interpolation_sub_mode_t::LINEAR_INTERPOLATION, 2);
    }

    return true;
  }


  int fault_reset(uint16 slave_idx)
  {
    uint16 control_word = 0x0086;
    wkc += writeSDO<uint16>(slave_idx, CONTROL_WORD_IDX, 0x00, control_word);
    return wkc;
  }


  int set_mode_of_operation(uint16 slave_idx, mode_of_operation_t mode_of_operation)
  {
    wkc += writeSDO<int8>(slave_idx, MODE_OF_OPERATION_IDX, 0x00, mode_of_operation);
    return wkc;
  }


  int set_position_window(uint16 slave_idx, uint32 position_window, uint16 position_window_time)
  {
    wkc += writeSDO<uint32>(slave_idx, POSITION_WINDOW_IDX, 0x00, position_window);
    wkc += writeSDO<uint16>(slave_idx, POSITION_WINDOW_TIME_IDX, 0x00, position_window_time);
    return wkc;
  }


  int config_profile(uint16 slave_idx, uint32 profile_velocity, uint32 profile_acceleration, uint32 profile_deceleration)
  {
    wkc += writeSDO<int32>(slave_idx, PROFILE_VELOCITY_IDX, 0x00, profile_velocity);
    wkc += writeSDO<int32>(slave_idx, PROFILE_ACCELERATION_IDX, 0x00, profile_acceleration);
    wkc += writeSDO<int32>(slave_idx, PROFILE_DECELERATION_IDX, 0x00, profile_deceleration);
    return wkc;
  }


  int config_homing(uint16 slave_idx, int8 homing_method, uint32 homing_speed_to_switch, uint32 homing_speed_to_zero, uint32 homing_acceleration, int32 home_offset, uint8 home_switch)
  {
    wkc += writeSDO<int8>(slave_idx, HOMING_METHOD_IDX, 0x00, homing_method);
    wkc += writeSDO<uint32>(slave_idx, HOMING_SPEEDS_IDX, 0x01, homing_speed_to_switch);
    wkc += writeSDO<uint32>(slave_idx, HOMING_SPEEDS_IDX, 0x02, homing_speed_to_zero);
    wkc += writeSDO<uint32>(slave_idx, HOMING_ACCELERATION_IDX, 0x00, homing_acceleration);
    wkc += writeSDO<int32>(slave_idx, HOME_OFFSET_IDX, 0x00, home_offset);
    return wkc;
  }


  int config_position_interpolation(uint16 slave_idx, interpolation_sub_mode_t interpolation_sub_mode_select, uint8 interpolation_time_units, int8 interpolation_time_index = -3)
  {
    wkc += writeSDO<int16>(slave_idx, INTERPOLATION_SUB_MODE_SELECT_IDX, 0x00, interpolation_sub_mode_select);
    wkc += writeSDO<uint8>(slave_idx, INTERPOLATION_TIME_PERIOD_IDX, 0x01, interpolation_time_units);
    wkc += writeSDO<int8>(slave_idx, INTERPOLATION_TIME_PERIOD_IDX, 0x02, interpolation_time_index);
    return wkc;
  }


  int set_following_error_window(uint16 slave_idx, uint32 following_error_window)
  {
    wkc += writeSDO<uint32>(slave_idx, FOLLOWING_ERROR_WINDOW_IDX, 0x00, following_error_window);
    return wkc;
  }


  int set_quickstop_deceleration(uint16 slave_idx, uint32 quickstop_deceleration)
  {
    wkc += writeSDO<uint32>(slave_idx, QUICKSTOP_DECELERATION_IDX, 0x00, quickstop_deceleration);
    return wkc;
  }


  int set_position_offset(uint16 slave_idx, int32 position_offset)
  {
    wkc += writeSDO<int32>(slave_idx, POSITION_OFFSET_IDX, 0x00, position_offset);
    return wkc;
  }


  int set_position_factor(uint16 slave_idx, uint32 numerator, uint32 feed_costant)
  {
    wkc += writeSDO<uint32>(slave_idx, POSITION_FACTOR_IDX, 0x01, numerator);
    wkc += writeSDO<uint32>(slave_idx, POSITION_FACTOR_IDX, 0x02, feed_costant);
    return wkc;
  }


  int get_actual_position(uint16 slave_idx, int32 &actual_position)
  {
    wkc += readSDO<int32>(slave_idx, POSITION_ACTUAL_VALUE_IDX, 0x00, actual_position);
    return wkc;
  }


  int get_error_code(const uint16 slave_idx, uint16 &error_code)
  {
    wkc += readSDO<uint16>(slave_idx, ERROR_CODE_IDX, 0x00, error_code);
    return wkc;
  }


  bool start()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x0006;
      rx_pdo[slave_idx].target_position = 0;
    }

    update();

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      ec_slave[slave_idx].state = EC_STATE_OPERATIONAL;
      ec_writestate(slave_idx);
    }

    ec_state = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    print_ec_state(0);
    return true;
  }


  bool fault_reset()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word |= 0x0080;
    }
    return true;
  }


  bool ready_to_switch_on()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x0006;
    }
    return true;
  }


  bool switch_on()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x0007;
    }
    return true;
  }


  bool switch_off()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word &= 0xFFFE;
    }
    return true;
  }


  bool enable_operation()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x000F;
    }
    return true;
  }


  // Starting the Homing Procedure

  /* Set the Homing Method required using OD entry 6098h. To start the homing
   * procedure, bit 4 of the controlword OD entry located at dictionary address
   * 6040h, must transition from 0 to 1. The status of the homing procedure can
   * be monitored using the statusword OD entry 6041h. */

  bool start_homing()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x001F;
    }
    return true;
  }

  // Enable Cyclic Synchronous Position Mode

  /* In this mode the master controller generates a trajectory and sends target
   * position (0x607A) to the drive at every PDO update cycle. The primary feedback
   * from the drive is the actual motor position and optionally, actual motor
   * velocity and torque. Position, velocity, and torque control loops are all
   closed in the drive which acts as a follower for the position commands. */

  bool start_cyclic_syncronous_position()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x000F;
    }
    return true;
  }

  // Enable Cyclic Synchronous Velocity Mode

  /* In this mode the master controller sends target velocity (0x60FF) to the
   * drive at every PDO update cycle. The primary feedback from the drive is the
   * actual motor position and optionally, actual motor velocity and torque.
   * Velocity and torque control loops are closed in the drive. If necessary,
   * position loop is closed in the master controller. */

  bool start_cyclic_syncronous_velocity()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x000F;
    }
    return true;
  }


  int update()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx] >> ec_slave[slave_idx].outputs;
    }

    ec_send_processdata();
    wkc += ec_receive_processdata(EC_TIMEOUTRET3);

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      tx_pdo[slave_idx] << ec_slave[slave_idx].inputs;
    }

    return wkc;
  }


  bool halt()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word |= 0x0100;
    }
    return true;
  }


  bool quick_stop()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word &= 0b1111111111111011;
    }
    return true;
  }


  void close()
  {
    ec_close();
  }

};

} } }  // namespace
#endif
