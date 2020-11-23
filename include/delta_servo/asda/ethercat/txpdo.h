#ifndef DELTA_ASDA_ETHERCAT_TXPDO_H
#define DELTA_ASDA_ETHERCAT_TXPDO_H


namespace delta { namespace asda { namespace ethercat { namespace pdo {


struct TxPDO1
{
  uint16 status_word;
  int32 actual_position;
  int32 actual_velocity;
  int32 actual_torque;
  int8 mode_of_operation_display;

  void operator<<(uint8 *data_ptr)
  {
    status_word = 0x0000;
    actual_position = 0x00000000;
    actual_velocity = 0x00000000;
    actual_torque = 0x00000000;
    mode_of_operation_display = 0x00;

    status_word |= (0x00FF & *data_ptr++) << 0;
    status_word |= (0x00FF & *data_ptr++) << 8;

    actual_position |= (0x000000FF & *data_ptr++) << 0;
    actual_position |= (0x000000FF & *data_ptr++) << 8;
    actual_position |= (0x000000FF & *data_ptr++) << 16;
    actual_position |= (0x000000FF & *data_ptr++) << 24;

    actual_velocity |= (0x000000FF & *data_ptr++) << 0;
    actual_velocity |= (0x000000FF & *data_ptr++) << 8;
    actual_velocity |= (0x000000FF & *data_ptr++) << 16;
    actual_velocity |= (0x000000FF & *data_ptr++) << 24;

    actual_torque |= (0x000000FF & *data_ptr++) << 0;
    actual_torque |= (0x000000FF & *data_ptr++) << 8;
    actual_torque |= (0x000000FF & *data_ptr++) << 16;
    actual_torque |= (0x000000FF & *data_ptr++) << 24;

    mode_of_operation_display |= (0xFF & *data_ptr++) << 0;
  }

  // std::stream operator <<(std::stream &os, const T &obj) {
  //   os << "RxPDO0:\n");
  //   printf("\tErrorCode: %x\n", rx_pdo.error_code);
  //   printf("\tStatusWord: %d\n", rx_pdo.status_word);
  //   printf("\tModes of Operation Display: %d\n", rx_pdo.modes_of_operation_display);
  //   printf("\tPosition Actual Value: %d\n", rx_pdo.position_actual_value);
  //   printf("\tFollow Error Actual Value: %d\n", rx_pdo.follow_error_actual_value);
  //   printf("\tTouch Probe Status: %d\n", rx_pdo.touch_probe_status);
  //   printf("\tTouch probe pos1 neg value: %d\n", rx_pdo.touch_probe_pos1_neg_value);
  //   printf("\tTouch probe pos1 pos value: %d\n", rx_pdo.touch_probe_pos1_pos_value);
  //   printf("\tTouch probe pos2 pos value: %d\n", rx_pdo.touch_probe_pos2_pos_value);
  //   printf("\tTouch probe pos2 neg value: %d\n", rx_pdo.touch_probe_pos2_neg_value);
  //   printf("\tDigital Inputs: %.32x\n", rx_pdo.digital_inputs);
  // }

};


struct TxPDO2
{
  uint16 status_word;
  int32 actual_position;

  void operator<<(uint8 *data_ptr)
  {
    status_word = 0x0000;
    actual_position = 0x00000000;

    status_word |= (0x00FF & *data_ptr++) << 0;
    status_word |= (0x00FF & *data_ptr++) << 8;

    actual_position |= (0x000000FF & *data_ptr++) << 0;
    actual_position |= (0x000000FF & *data_ptr++) << 8;
    actual_position |= (0x000000FF & *data_ptr++) << 16;
    actual_position |= (0x000000FF & *data_ptr++) << 24;
  }

  // std::stream operator <<(std::stream &os, const T &obj) {
  //   os << "RxPDO0:\n");
  //   printf("\tErrorCode: %x\n", rx_pdo.error_code);
  //   printf("\tStatusWord: %d\n", rx_pdo.status_word);
  //   printf("\tModes of Operation Display: %d\n", rx_pdo.modes_of_operation_display);
  //   printf("\tPosition Actual Value: %d\n", rx_pdo.position_actual_value);
  //   printf("\tFollow Error Actual Value: %d\n", rx_pdo.follow_error_actual_value);
  //   printf("\tTouch Probe Status: %d\n", rx_pdo.touch_probe_status);
  //   printf("\tTouch probe pos1 neg value: %d\n", rx_pdo.touch_probe_pos1_neg_value);
  //   printf("\tTouch probe pos1 pos value: %d\n", rx_pdo.touch_probe_pos1_pos_value);
  //   printf("\tTouch probe pos2 pos value: %d\n", rx_pdo.touch_probe_pos2_pos_value);
  //   printf("\tTouch probe pos2 neg value: %d\n", rx_pdo.touch_probe_pos2_neg_value);
  //   printf("\tDigital Inputs: %.32x\n", rx_pdo.digital_inputs);
  // }

};


struct TxPDO3
{
  uint16 status_word;
  int32 actual_position;
  int32 actual_velocity;

  void operator<<(uint8 *data_ptr)
  {
    status_word = 0x0000;
    actual_position = 0x00000000;
    actual_velocity = 0x00000000;

    status_word |= (0x00FF & *data_ptr++) << 0;
    status_word |= (0x00FF & *data_ptr++) << 8;

    actual_position |= (0x000000FF & *data_ptr++) << 0;
    actual_position |= (0x000000FF & *data_ptr++) << 8;
    actual_position |= (0x000000FF & *data_ptr++) << 16;
    actual_position |= (0x000000FF & *data_ptr++) << 24;

    actual_velocity |= (0x000000FF & *data_ptr++) << 0;
    actual_velocity |= (0x000000FF & *data_ptr++) << 8;
    actual_velocity |= (0x000000FF & *data_ptr++) << 16;
    actual_velocity |= (0x000000FF & *data_ptr++) << 24;
  }

  // std::stream operator <<(std::stream &os, const T &obj) {
  //   os << "RxPDO0:\n");
  //   printf("\tErrorCode: %x\n", rx_pdo.error_code);
  //   printf("\tStatusWord: %d\n", rx_pdo.status_word);
  //   printf("\tModes of Operation Display: %d\n", rx_pdo.modes_of_operation_display);
  //   printf("\tPosition Actual Value: %d\n", rx_pdo.position_actual_value);
  //   printf("\tFollow Error Actual Value: %d\n", rx_pdo.follow_error_actual_value);
  //   printf("\tTouch Probe Status: %d\n", rx_pdo.touch_probe_status);
  //   printf("\tTouch probe pos1 neg value: %d\n", rx_pdo.touch_probe_pos1_neg_value);
  //   printf("\tTouch probe pos1 pos value: %d\n", rx_pdo.touch_probe_pos1_pos_value);
  //   printf("\tTouch probe pos2 pos value: %d\n", rx_pdo.touch_probe_pos2_pos_value);
  //   printf("\tTouch probe pos2 neg value: %d\n", rx_pdo.touch_probe_pos2_neg_value);
  //   printf("\tDigital Inputs: %.32x\n", rx_pdo.digital_inputs);
  // }

};


struct TxPDO4
{
  uint16 status_word;
  int32 actual_position;
  int32 actual_torque;

  void operator<<(uint8 *data_ptr)
  {
    status_word = 0x0000;
    actual_position = 0x00000000;
    actual_torque = 0x00000000;

    status_word |= (0x00FF & *data_ptr++) << 0;
    status_word |= (0x00FF & *data_ptr++) << 8;

    actual_position |= (0x000000FF & *data_ptr++) << 0;
    actual_position |= (0x000000FF & *data_ptr++) << 8;
    actual_position |= (0x000000FF & *data_ptr++) << 16;
    actual_position |= (0x000000FF & *data_ptr++) << 24;

    actual_torque |= (0x000000FF & *data_ptr++) << 0;
    actual_torque |= (0x000000FF & *data_ptr++) << 8;
    actual_torque |= (0x000000FF & *data_ptr++) << 16;
    actual_torque |= (0x000000FF & *data_ptr++) << 24;
  }

  // std::stream operator <<(std::stream &os, const T &obj) {
  //   os << "RxPDO0:\n");
  //   printf("\tErrorCode: %x\n", rx_pdo.error_code);
  //   printf("\tStatusWord: %d\n", rx_pdo.status_word);
  //   printf("\tModes of Operation Display: %d\n", rx_pdo.modes_of_operation_display);
  //   printf("\tPosition Actual Value: %d\n", rx_pdo.position_actual_value);
  //   printf("\tFollow Error Actual Value: %d\n", rx_pdo.follow_error_actual_value);
  //   printf("\tTouch Probe Status: %d\n", rx_pdo.touch_probe_status);
  //   printf("\tTouch probe pos1 neg value: %d\n", rx_pdo.touch_probe_pos1_neg_value);
  //   printf("\tTouch probe pos1 pos value: %d\n", rx_pdo.touch_probe_pos1_pos_value);
  //   printf("\tTouch probe pos2 pos value: %d\n", rx_pdo.touch_probe_pos2_pos_value);
  //   printf("\tTouch probe pos2 neg value: %d\n", rx_pdo.touch_probe_pos2_neg_value);
  //   printf("\tDigital Inputs: %.32x\n", rx_pdo.digital_inputs);
  // }

};


// union TxPDO
// {
//   TxPDO0 _0;
//   TxPDO1 _1;
//   TxPDO2 _2;
//   TxPDO3 _3;
// };


} } } } // namespace
#endif
