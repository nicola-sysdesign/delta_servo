#ifndef DELTA_ASDA_ETHERCAT_COMMON_H
#define DELTA_ASDA_ETHERCAT_COMMON_H
// STL
#include <string>
#include <map>
// soem
#include "ethercat.h"


namespace delta { namespace asda { namespace ethercat {

/* Mode of Operation */
enum mode_of_operation_t : int8
{
  PROFILE_POSITION = 1,              // Profile Position Mode
  PROFILE_VELOCITY = 3,              // Profile Velocity Mode
  PROFILE_TORQUE = 4,                // Torque Profile Mode
  HOMING = 6,                        // Homing Mode
  INTERPOLATED_POSITION = 7,         // Interpolated Position Mode
  CYCLIC_SYNCHRONOUS_POSITION = 8,   // Cyclic Synchronous Position Mode
  CYCLIC_SYNCHRONOUS_VELOCITY = 9,   // Cyclic Synchronous Velocity Mode
  CYCLIC_SYNCHRONOUS_TORQUE = 10,    // Cyclic Synchronous Torque Mode
};
// In SM synchronization mode, PP, PV, TQ, HM and Q modes are supported.
// In DC synchronization mode, CSP, CSV and HM modes are supported.


/* Interpolation Sub-Mode */
enum interpolation_sub_mode_t : int16
{
  LINEAR_INTERPOLATION = 0,           // Linear Interpolation
};


/* Error Codes */
// static std::map<uint16, std::string> error_codes
// {
//   { 0x7500, "EtherCAT communication error"},
//   { 0xFF01, "Over Current"},
//   { 0xFF02, "Over Voltage"},
//   { 0xFF03, "Over Temperature"},
//   { 0xFF04, "Open Motor Winding"},
//   { 0xFF05, "Internal Voltage Bad"},
//   { 0xFF06, "Position Limit"},
//   { 0xFF07, "Bad Encoder"},
//   { 0xFF08, "reserved"},
//   { 0xFF09, "reserved"},
//   { 0xFF0A, "Excess regen"},
//   { 0xFF0B, "Safe Torque Off"},
//   { 0xFF31, "CW Limit"},
//   { 0xFF32, "CCW Limit"},
//   { 0xFF33, "CW Limit and CCW Limit"},
//   { 0xFF34, "Current Foldback"},
//   { 0xFF35, "Move while Disabled"},
//   { 0xFF36, "Under Voltage"},
//   { 0xFF37, "Blank Q segment"},
//   { 0xFF41, "Save Failed"},
//   { 0xFFFF, "Other Error"},
// };


/* Error Codes */
static std::map<uint32, std::string> error_codes
{
  { 0x23100001, "Overcurrent"},
  { 0x31100002, "Overvoltage"},
  { 0x31200003, "Undervoltage"},
  { 0x71220004, "Motor error"},
  { 0x32100005, "Regenerator error"},
  { 0x32300006, "Overload"},
  { 0x84000007, "Overspeed"},
  { 0x86000008, "Abnormal pulse control command"},
  { 0x86110009, "Excessive deviation"},
  { 0x00000010, "Reserved"},
  { 0x73050011, "Encoder Error"},
  { 0x63200012, "Adjustment error"},
  { 0x54410013, "Emergency stop activated"},
  { 0x54430014, "Reverse limit switch error"},
  { 0x54420015, "Forward limit switch error"},
  { 0x42100016, "IGBT temperature error"},
  { 0x53300017, "Memory error"},
  { 0x73060018, "Encoder output error"},
  { 0x75100019, "Serial communication error"},
  { 0x75200020, "Serial communication time out"},
  { 0x31300022, "Input power phase loss"},
  { 0x32310023, "Early warning for overload"},
  { 0x73050024, "Encoder initial magnetic field error"},
  { 0x73050025, "Encoder internal error"},
  { 0x73050026, "Unreliable internal data of the encoder"},
  { 0x73050027, "Encoder data error"},
  { 0x71210030, "Motor protection error"},
  { 0x33000031, "U,V,W wiring error"},
  { 0x86100040, "Full-closed loop excessive deviation"},
  { 0x55000099, "DSP firmware upgrade"},
  { 0x63100201, "CANopen Data Initial Error"},
  { 0x54440283, "Forward software limit"},
  { 0x54450285, "Reverse software limit"},
  { 0x81200185, "EtherCAT connection error (Servo Off)"},
  { 0x81300180, "Node guarding or Heartbeat error (Servo Off)"},
  { 0x82000122, "Sub-index error occurs when accessing CANopen PDO object"},
  { 0x82000123, "Data type (size) error occurs when accessing CANopen PDO object"},
  { 0x82000124, "Data range error occurs when accessing CANopen PDO object"},
  { 0x82000125, "CANopen PDO object is read-only and write-protected"},
  { 0x82000126, "CANopen PDO object does not support PDO"},
  { 0x82000127, "CANopen PDO object is write-protected when Servo On"},
  { 0x82000128, "Error occurs when reading CANopen PDO object from EEPROM"},
  { 0x82000129, "Error occurs when writing CANopen PDO object into EEPROM"},
  { 0x82000130, "EEPROM invalid address range"},
  { 0x82000131, "EEPROM checksum error"},
  { 0x82000132, "EEPROM zone error"},
  { 0x63100201, "CANopen load/save 1010/1011 error"},
  { 0x620003E1, "CANopen SYNC failed (Servo Off)"},
  { 0x620003E2, "CANopen SYNC signal error (Servo Off)"},
  { 0x620003E3, "CANopen SYNC time out (Servo Off)"},
  { 0x620003E4, "CANopen IP command failed (Servo Off)"},
  { 0x620003E5, "SYNC period error (Servo Off)"},
  { 0x90000500, "Safe torque off (Servo Off)"},
  { 0x90000501, "STO_A lost (Servo Off)"},
  { 0x90000502, "STO_B lost (Servo Off)"},
  { 0x90000503, "STO_error (Servo Off)"},
};


template<class T>
int writeSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T value)
{
  int wkc = 0;

  T data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOwrite(slave, index, sub_index, FALSE, size_of_data, &data, EC_TIMEOUTRXM);

  return wkc;
}


template<class T>
int writeSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T *value)
{
  int wkc = 0;

  T *data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOwrite(slave, index, sub_index, TRUE, size_of_data, data, EC_TIMEOUTRXM);

  return wkc;
}


template<class T>
int readSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T &value)
{
  int wkc = 0;

  T data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOread(slave, index, sub_index, FALSE, &size_of_data, &data, EC_TIMEOUTRXM);

  value = data;

  return wkc;
}


template<class T>
int readSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T *value)
{
  int wkc = 0;

  T *data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOread(slave, index, sub_index, TRUE, &size_of_data, data, EC_TIMEOUTRXM);

  *value = *data;

  return wkc;
}


inline void print_ec_state(uint16 slave_idx)
{
  switch (ec_slave[slave_idx].state)
  {
    case EC_STATE_NONE:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "NONE");
      break;
    case EC_STATE_INIT:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "INIT");
      break;
    case EC_STATE_PRE_OP:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "PRE_OP");
      break;
    case EC_STATE_BOOT:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "BOOT");
      break;
    case EC_STATE_SAFE_OP:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "SAFE_OP");
      break;
    case EC_STATE_OPERATIONAL:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "OPERATIONAL");
      break;
    //case EC_STATE_ACK:
    //  ROS_INFO("%s: ESM: %s", ec_slave[slave].name, "EC_STATE_ACK");
    //  break;
    case EC_STATE_PRE_OP + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "PRE_OP", "ERROR");
      break;
    case EC_STATE_SAFE_OP + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "SAFE_OP", "ERROR");
      break;
    case EC_STATE_OPERATIONAL + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "OPERATIONAL", "ERROR");
      break;
  }
}


inline void print_sm(uint16 slave_idx, int sm)
{
  uint16 A = ec_slave[slave_idx].SM[sm].StartAddr;
  uint16 L = ec_slave[slave_idx].SM[sm].SMlength;
  uint32 F = ec_slave[slave_idx].SM[sm].SMflags;
  uint8 Type = ec_slave[slave_idx].SMtype[sm];
  printf("SM%d A:%4.4x L:%4d F:%8.8x Type:%d\n", sm, A, L, F, Type);
}


inline void print_fmmu(uint16 slave_idx, int fmmu)
{
  uint32 Ls = ec_slave[slave_idx].FMMU[fmmu].LogStart;
  uint16 Ll = ec_slave[slave_idx].FMMU[fmmu].LogLength;
  uint8 Lsb = ec_slave[slave_idx].FMMU[fmmu].LogStartbit;
  uint8 Leb = ec_slave[slave_idx].FMMU[fmmu].LogEndbit;
  uint16 Ps = ec_slave[slave_idx].FMMU[fmmu].PhysStart;
  uint8 Psb = ec_slave[slave_idx].FMMU[fmmu].PhysStartBit;
  uint8 Ty = ec_slave[slave_idx].FMMU[fmmu].FMMUtype;
  uint8 Act = ec_slave[slave_idx].FMMU[fmmu].FMMUactive;
  printf("FMMU%d Ls:%.8x Ll:%4.2d Lsb:%d Leb:%d Ps:%.4x Psb:%d Ty:%.2d Act:%.2d\n", fmmu, Ls, Ll, Lsb, Leb, Ps, Psb, Ty, Act);
}


} } } // namespace
#endif
