#include "../../pack_fw_states/pack_fw_states.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pack_fw4_states");//发布rod节点pack_fw4_states

  PACK_FW_STATES _pack_fw4;
  if (true) {
    _pack_fw4.set_planeID(4);
    _pack_fw4.set_scurve(100);//初始化虚拟目标点弧长为100，后续开始迭代
    _pack_fw4.run(argc, argv);
  }

  return 0;
}
