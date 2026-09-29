#include "../../pack_fw_states/pack_fw_states.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pack_fw1_states");//发布rod节点pack_fw1_states

  PACK_FW_STATES _pack_fw1;
  if (true) {
    _pack_fw1.set_planeID(1);
    _pack_fw1.set_scurve(100);//初始化虚拟目标点弧长为100，后续开始迭代
    _pack_fw1.run(argc, argv);
  }

  return 0;
}
