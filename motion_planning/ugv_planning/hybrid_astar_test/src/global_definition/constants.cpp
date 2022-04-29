#include "hybrid_astar_test/global_definition/constants.hpp"

namespace HybridAStar {
namespace Constants {
bool coutDEBUG = false;                      
bool manual = true;                          
bool visualization = true  && manual;        
bool visualization2D = true  && manual;      
bool reverse = true;                         
bool dubinsShot = true;                      
bool dubins = false;                         
bool dubinsLookup = false && dubins;         
bool twoD = true;                            


int iterations = 30000;                      
double bloating = 0;                         
double width = 1.75 + 2 * bloating;          
double length = 2.65 + 2 * bloating;         
float r = 6;                                 
float vehicle_wheelbase = 2.7;               
float vehicle_max_rudder = 30;               
//int headings = 72;                           
float deltaHeadingDeg = 360 / (float)headings; 
float deltaHeadingRad = 2 * M_PI / (float)headings;
float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
float cellSize = 1.0;                        
float tieBreaker = 0.01;                     

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
    std::cout << "\e[1;32;49m-------------------------loading params-------------------------\e[0m" << std::endl;

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
        std::cout << "\e[1;31;49m >>> failed, please check rhe params\e[0m" << std::endl;
        return false;
    }

    r = vehicle_wheelbase / tan(vehicle_max_rudder * M_PI / 180.0);
    length = length + 2 * bloating;
    width  = width + 2 * bloating;
    bbSize = std::ceil((sqrt(width * width + length* length) + 4) / cellSize);
    reverse  = false;
    std::cout << "\e[1;35;49m>>> vehicle_length : " << length <<  " m\e[0m" << std::endl;
    std::cout << "\e[1;35;49m>>> vehicle_width : " << width <<  " m\e[0m" << std::endl;
    std::cout << "\e[1;35;49m>>> vehicle_wheelbase : " << vehicle_wheelbase <<  " m\e[0m" << std::endl;
    std::cout << "\e[1;35;49m>>> vehicle_max_rudder : " << vehicle_max_rudder <<  " m\e[0m" << std::endl;
    std::cout << "\e[1;35;49m>>> safe_distance : " << bloating <<  " m\e[0m" << std::endl;
    std::cout << "\e[1;32;49m-----------------------success------------------------\e[0m" << std::endl;

    return true;
}

}
}
