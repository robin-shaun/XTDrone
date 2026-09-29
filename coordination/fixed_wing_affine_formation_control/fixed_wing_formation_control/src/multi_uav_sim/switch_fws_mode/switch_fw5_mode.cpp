#include "../../switch_fw_mode/switch_fw_mode.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "switch_fw5_mode");

  SWITCH_FW_MODE _switch_fw5;

  _switch_fw5.set_planeID(5);
  _switch_fw5.run();

  return 0;
}
