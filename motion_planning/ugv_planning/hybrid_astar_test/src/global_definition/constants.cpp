#include "hybrid_astar_test/global_definition/constants.hpp"

namespace HybridAStar {
namespace Constants {
bool coutDEBUG = false;                      // 调试模式， 是否打印输出
bool manual = true;                          // true : 静态地图   false ： 动态地图
bool visualization = true  && manual;        // 3D节点可视化
bool visualization2D = true  && manual;      // 2D节点可视化
bool reverse = true;                         // 车辆是否能够倒退
bool dubinsShot = true;                      // 使用杜宾斯曲线
bool dubins = false;                         // 使用杜宾斯代价函数 （如果车辆可倒退，使用true或false，否则使用false）
bool dubinsLookup = false && dubins;         // 杜宾斯曲线前向查路
bool twoD = true;                            // 2D 贪心算法


int iterations = 30000;                      // A*算法最大叠代层数
double bloating = 0;                         // 车辆安全距离膨胀[m]
double width = 1.75 + 2 * bloating;          // 车辆宽度[m]
double length = 2.65 + 2 * bloating;         // 车辆长度[m]
float r = 6;                                 // 最小转弯半径
float vehicle_wheelbase = 2.7;               // 车辆前后轮轴长
float vehicle_max_rudder = 30;               // 车辆前轮方向舵最大角度 【deg】
//int headings = 72;                           // 前向离散点的个数
float deltaHeadingDeg = 360 / (float)headings; // [°] --- The discretization value of the heading (goal condition)
float deltaHeadingRad = 2 * M_PI / (float)headings;
float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
float cellSize = 1.0;                        // 地图珊格分辨率
float tieBreaker = 0.01;                     // tieBreaker

// HEURISTIC CONSTANTS
float factor2D = sqrt(5) / sqrt(2) + 1;
float penaltyTurning = 1.05;
float penaltyReversing = 2.0;
float penaltyCOD = 2.0;
float dubinsShotDistance = 100;
float dubinsStepSize = 1;

// DUBINS LOOKUP SPECIFIC
int dubinsWidth = 15;
int dubinsArea = dubinsWidth * dubinsWidth;


// _________________________
// COLLISION LOOKUP SPECIFIC

int bbSize = std::ceil((sqrt(width * width + length* length) + 4) / cellSize);


bool LoadParamFromYaml(){
    std::string config_file_path = WORK_SPACE_PATH + "/config/base_config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    std::cout << "\e[1;32;49m-------------------------加载基本配置-------------------------\e[0m" << std::endl;

    try{
        vehicle_wheelbase   = config_node["vehicle_wheelbase"].as<double>();
        vehicle_max_rudder  = config_node["vehicle_max_rudder"].as<double>();
        length              = config_node["vehicle_length"].as<double>();
        width               = config_node["vehicle_width"].as<double>();
        bloating            = config_node["vehicle_safe_distance"].as<double>();
        cellSize            = config_node["map_resolution"].as<float>();
        manual              = config_node["robot_environment"].as<bool>();
    }
    catch(...){
        std::cout << "\e[1;31;49m >>> 加载YAML参数失败, 请检查yaml文件格式以及参数\e[0m" << std::endl;
        return false;
    }

    r = vehicle_wheelbase / tan(vehicle_max_rudder * M_PI / 180.0);
    length = length + 2 * bloating;
    width  = width + 2 * bloating;
    bbSize = std::ceil((sqrt(width * width + length* length) + 4) / cellSize);
    reverse  = false;
    std::cout << "\e[1;35;49m>>> 车辆长度 ： " << length <<  " m\e[0m" << std::endl;
    std::cout << "\e[1;35;49m>>> 车辆宽度 ： " << width <<  " m\e[0m" << std::endl;
    std::cout << "\e[1;35;49m>>> 车辆轴距 ： " << vehicle_wheelbase <<  " m\e[0m" << std::endl;
    std::cout << "\e[1;35;49m>>> 车辆前轮最大转角 ： " << vehicle_max_rudder <<  " m\e[0m" << std::endl;
    std::cout << "\e[1;35;49m>>> 车辆安全距离 ： " << bloating <<  " m\e[0m" << std::endl;
    std::cout << "\e[1;32;49m-----------------------加载基本配置完成------------------------\e[0m" << std::endl;

    return true;
}

}
}