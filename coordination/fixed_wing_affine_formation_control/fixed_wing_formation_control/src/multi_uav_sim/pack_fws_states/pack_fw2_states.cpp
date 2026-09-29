#include "../../pack_fw_states/pack_fw_states.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pack_fw2_states");//发布rod节点pack_fw2_states

  PACK_FW_STATES _pack_fw2;
  if (true) {
    _pack_fw2.set_planeID(2);
    _pack_fw2.set_scurve(100);//初始化虚拟目标点弧长为100，后续开始迭代
    _pack_fw2.run(argc, argv);
  }

  return 0;
}
