#include "../../test/switch_fw_mode/switch_fw_mode.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "switch_leader_mode");

  SWITCH_FW_MODE _switch_leader;

  _switch_leader.set_planeID(0);
  _switch_leader.run();

  return 0;
}
