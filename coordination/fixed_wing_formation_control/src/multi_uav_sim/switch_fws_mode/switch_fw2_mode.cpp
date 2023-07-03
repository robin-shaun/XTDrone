#include "../../test/switch_fw_mode/switch_fw_mode.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "switch_fw2_mode");

  SWITCH_FW_MODE _switch_fw2;

  _switch_fw2.set_planeID(2);
  _switch_fw2.run();

  return 0;
}
