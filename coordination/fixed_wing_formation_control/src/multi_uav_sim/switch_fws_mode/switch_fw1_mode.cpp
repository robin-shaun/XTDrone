#include "../../test/switch_fw_mode/switch_fw_mode.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "switch_fw1_mode");

  SWITCH_FW_MODE _switch_fw1;

  _switch_fw1.set_planeID(1);
  _switch_fw1.run();

  return 0;
}
