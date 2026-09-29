#include "../../switch_fw_mode/switch_fw_mode.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "switch_fw0_mode");

  SWITCH_FW_MODE _switch_fw0;

  _switch_fw0.set_planeID(0);
  _switch_fw0.run();

  return 0;
}
