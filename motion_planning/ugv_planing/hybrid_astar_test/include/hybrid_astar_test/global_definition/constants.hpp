#ifndef CONSTANTS
#define CONSTANTS

#include <iostream>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "hybrid_astar_test/global_definition/global_definition.h"

namespace HybridAStar {
namespace Constants {
extern bool coutDEBUG;                          // 调试模式， 是否打印输出
extern bool manual;                             // true : 静态地图   false ： 动态地图
extern bool visualization;                      // 3D节点可视化
extern bool visualization2D;                    // 2D节点可视化
extern bool reverse;                            // 车辆是否能够倒退
extern bool dubinsShot;                         // 使用杜宾斯曲线
extern bool dubins;                             // 使用杜宾斯代价函数 （如果车辆可倒退，使用true或false，否则使用false）
extern bool dubinsLookup;                       // 杜宾斯曲线前向查路
extern bool twoD;                               // 2D 贪心算法


extern int iterations;                          // A*算法最大叠代层数
extern double bloating;                         // 车辆安全距离膨胀[m]
extern double width;                            // 车辆宽度[m]
extern double length;                           // 车辆长度[m]
extern float r;                                 // 车辆最小转弯半径[m]     车辆前后轮轴长 / tan(最大前轮转角)   (rad)
extern float vehicle_wheelbase;                 // 车辆前后轮轴长
extern float vehicle_max_rudder;                // 车辆前轮方向舵最大角度 【deg】
static const int headings = 72;                            // 前向离散点的个数
extern float deltaHeadingDeg;                   // [°] --- The discretization value of the heading (goal condition)
extern float deltaHeadingRad;
extern float deltaHeadingNegRad;
extern float cellSize;                          // 地图珊格分辨率
extern float tieBreaker;                        // tieBreaker

// HEURISTIC CONSTANTS
extern float factor2D;
extern float penaltyTurning;
extern float penaltyReversing;
extern float penaltyCOD;
extern float dubinsShotDistance;
extern float dubinsStepSize;

// DUBINS LOOKUP SPECIFIC
extern int dubinsWidth;
extern int dubinsArea;


// _________________________
// COLLISION LOOKUP SPECIFIC

/// [m] -- The bounding box size length and width to precompute all possible headings
extern int bbSize;
/// [#] --- The sqrt of the number of discrete positions per cell
static const int positionResolution = 10;
/// [#] --- The number of discrete positions per cell
static const int positions = positionResolution * positionResolution;

struct relPos {
    int x;
    int y;
};

struct config {
    int length;
    relPos pos[64];
};

// _________________
// 轨迹优化
static const float minRoadWidth = 2;        // 最小道路的宽度

struct color {
  float red;
  float green;
  float blue;
};

static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};

extern bool LoadParamFromYaml();
}
}

#endif // CONSTANTS

