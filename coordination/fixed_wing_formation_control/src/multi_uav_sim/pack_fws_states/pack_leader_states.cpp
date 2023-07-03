#include "../../pack_fw_states/pack_fw_states.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pack_leader_states");

  PACK_FW_STATES _pack_leader;
  if (true) {
    _pack_leader.set_planeID(0);
    _pack_leader.run(argc, argv);
  }

  return 0;
}
