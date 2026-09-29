#include "../../switch_fw_mode/switch_fw_mode.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "switch_fw3_mode");

  SWITCH_FW_MODE _switch_fw3;

  _switch_fw3.set_planeID(3);
  _switch_fw3.run();

  return 0;
}
