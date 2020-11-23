#ifndef DELTA_ASDA_ETHERCAT_RXPDO_H
#define DELTA_ASDA_ETHERCAT_RXPDO_H


namespace delta { namespace asda { namespace ethercat { namespace pdo {


struct RxPDO1
{
  uint16 control_word;
  int32 target_position;
  int32 target_velocity;
  int32 target_torque;
  int8 mode_of_operation;

  void operator>>(uint8 *data_ptr)
  {
    *data_ptr++ = (control_word >> 0) & 0xFF;
    *data_ptr++ = (control_word >> 8) & 0xFF;

    *data_ptr++ = (target_position >> 0) & 0xFF;
    *data_ptr++ = (target_position >> 8) & 0xFF;
    *data_ptr++ = (target_position >> 16) & 0xFF;
    *data_ptr++ = (target_position >> 24) & 0xFF;

    *data_ptr++ = (target_velocity >> 0) & 0xFF;
    *data_ptr++ = (target_velocity >> 8) & 0xFF;
    *data_ptr++ = (target_velocity >> 16) & 0xFF;
    *data_ptr++ = (target_velocity >> 24) & 0xFF;

    *data_ptr++ = (target_torque >> 0) & 0xFF;
    *data_ptr++ = (target_torque >> 8) & 0xFF;
    *data_ptr++ = (target_torque >> 16) & 0xFF;
    *data_ptr++ = (target_torque >> 24) & 0xFF;

    *data_ptr++ = (mode_of_operation >> 0) & 0xFF;
  }

  // std::stream operator <<(std::stream &os, const RxPDO0 &obj)
  // {
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


struct RxPDO2
{
  uint16 control_word;
  int32 target_position;

  void operator>>(uint8 *data_ptr)
  {
    *data_ptr++ = (control_word >> 0) & 0xFF;
    *data_ptr++ = (control_word >> 8) & 0xFF;

    *data_ptr++ = (target_position >> 0) & 0xFF;
    *data_ptr++ = (target_position >> 8) & 0xFF;
    *data_ptr++ = (target_position >> 16) & 0xFF;
    *data_ptr++ = (target_position >> 24) & 0xFF;
  }

  // std::stream operator <<(std::stream &os, const T &obj)
  // {
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


struct RxPDO3
{
  uint16 control_word;
  int32 target_velocity;

  void operator>>(uint8 *data_ptr)
  {
    *data_ptr++ = (control_word >> 0) & 0xFF;
    *data_ptr++ = (control_word >> 8) & 0xFF;

    *data_ptr++ = (target_velocity >> 0) & 0xFF;
    *data_ptr++ = (target_velocity >> 8) & 0xFF;
    *data_ptr++ = (target_velocity >> 16) & 0xFF;
    *data_ptr++ = (target_velocity >> 24) & 0xFF;
  }

  // std::stream operator <<(std::stream &os, const T &obj)
  // {
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


struct RxPDO4
{
  uint16 control_word;
  int32 target_torque;

  void operator>>(uint8 *data_ptr)
  {
    *data_ptr++ = (control_word >> 0) & 0xFF;
    *data_ptr++ = (control_word >> 8) & 0xFF;

    *data_ptr++ = (target_torque >> 0) & 0xFF;
    *data_ptr++ = (target_torque >> 8) & 0xFF;
    *data_ptr++ = (target_torque >> 16) & 0xFF;
    *data_ptr++ = (target_torque >> 24) & 0xFF;
  }

  // std::stream operator <<(std::stream &os, const T &obj)
  // {
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


// union RxPDO
// {
//   RxPDO1 _0;
//   RxPDO2 _1;
//   RxPDO3 _2;
//   RxPDO4 _3;
// };


} } } } // namespace
#endif
