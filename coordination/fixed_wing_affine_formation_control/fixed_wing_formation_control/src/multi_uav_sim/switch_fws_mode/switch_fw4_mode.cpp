#include "../../switch_fw_mode/switch_fw_mode.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "switch_fw4_mode");

  SWITCH_FW_MODE _switch_fw4;

  _switch_fw4.set_planeID(4);
  _switch_fw4.run();

  return 0;
}
