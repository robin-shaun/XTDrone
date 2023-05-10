#include <gazebo_state_plugins/GazeboMapPublisher.h>
#include <gazebo_version_helpers/GazeboVersionHelpers.h>

#include <gazebo/physics/physics.hh>

#include <Eigen/Core>

#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Float64.h>

#define DEFAULT_REQUEST_MAP_SERVICE "dynamic_map"

//publishing rate
#define DEFAULT_MAP_PUB_FREQ "1" 

#define MAP_OFFSET_X -10
#define MAP_OFFSET_Y -10
#define MAP_LEN_X 20
#define MAP_LEN_Y 20
//map resolution (m/cell)
#define MAP_RESOLUTION 0.1

//height of obstacles to consider for generating the map
#define MAP_HEIGHT 1.0

#define MAP_QUEUE_SIZE 50

#define MSG_PRINT(o)\
    std::cout<<o<<std::endl;

#define MAX_MAP_VAL 100 

using gazebo::GazeboMapPublisher;
using gazebo::GzVector3;

GZ_REGISTER_WORLD_PLUGIN(GazeboMapPublisher)

class GazeboMapPublisher::CollisionMapRequest {
    public:
    CollisionMapRequest(): threshold(MAX_MAP_VAL) {
    }
    CollisionMapRequest(const CollisionMapRequest& o):
        upperLeft(o.upperLeft),
        upperRight(o.upperRight),
        lowerRight(o.lowerRight),
        lowerLeft(o.lowerLeft),
        height(o.height),
        resolution(o.resolution),
        threshold(o.threshold){}

    Eigen::Vector2i  upperLeft;
    Eigen::Vector2i  upperRight;
    Eigen::Vector2i  lowerRight;
    Eigen::Vector2i  lowerLeft;
    double height;
    double resolution;
    unsigned int  threshold;
};

GazeboMapPublisher::GazeboMapPublisher() : WorldPlugin() {
   
    ros::NodeHandle _node("/gazebo_state_plugins");

    _node.param<std::string>("publish_map_topic", MAP_TOPIC, "");
    MSG_PRINT("Got gazebo map topic name :"<<MAP_TOPIC.c_str());
    
    _node.param<std::string>("map_frame_id", MAP_FRAME, "/map");
    MSG_PRINT("Got gazebo map frame id: "<<MAP_FRAME.c_str());
    
    _node.param<std::string>("map_metadata_topic", METADATA_TOPIC, "map_metadata");
    MSG_PRINT("Got gazebo map metadata topic name:"<<METADATA_TOPIC.c_str());
    
    _node.param<std::string>("request_map_service", REQUEST_MAP_SERVICE, DEFAULT_REQUEST_MAP_SERVICE);
    MSG_PRINT("Got gazebo map service name: " <<REQUEST_MAP_SERVICE.c_str());
    
    _node.param<std::string>("robot_name", ROBOT_NAME, "robot");
    MSG_PRINT("Got gazebo map publisher robot name: " <<ROBOT_NAME.c_str());

    std::string MAP_PUB_FREQUENCY;
    _node.param<std::string>("map_pub_frequency", MAP_PUB_FREQUENCY, DEFAULT_MAP_PUB_FREQ);
    MAP_PUB_FREQ=atoi(MAP_PUB_FREQUENCY.c_str());
    MSG_PRINT("Got gazebo map publish frequency: <"<<MAP_PUB_FREQ<<"> from string "<<MAP_PUB_FREQUENCY.c_str());

    map_offset_x=MAP_OFFSET_X;
    map_offset_y=MAP_OFFSET_Y;
    map_len_x=MAP_LEN_X;
    map_len_y=MAP_LEN_Y;
    map_resolution=MAP_RESOLUTION;
    map_height=MAP_HEIGHT;
    
    worldChangedSinceLastAdvert=true; 
    
}

void GazeboMapPublisher::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
    world=_world;    

    request_map_srv = node.advertiseService(REQUEST_MAP_SERVICE, &GazeboMapPublisher::requestMap,this);

    if (MAP_TOPIC.length()==0) {
        ROS_WARN("No map being published from gazebo, as the parameter 'gazebo_publish_map' was specified empty. This may be intended.");
        return;
    }
    
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboMapPublisher::onWorldUpdate, this));

    ros::Rate rate(MAP_PUB_FREQ);
    publishTimer=node.createTimer(rate,&GazeboMapPublisher::advertEvent, this);

    map_pub = node.advertise<nav_msgs::OccupancyGrid>(MAP_TOPIC, MAP_QUEUE_SIZE);
    meta_pub = node.advertise<nav_msgs::MapMetaData>(METADATA_TOPIC,MAP_QUEUE_SIZE);
}


bool GazeboMapPublisher::requestMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res) {
    nav_msgs::OccupancyGrid map=getMap();
    res.map=map;
    MSG_PRINT("Received service request for map!");
    return true;
}

void GazeboMapPublisher::advertEvent(const ros::TimerEvent& e) {

    if (!worldChangedSinceLastAdvert) return;

    if (map_pub.getNumSubscribers()!=0) {
        nav_msgs::OccupancyGrid map=getMap();
        map_pub.publish(map);
    }

    if (meta_pub.getNumSubscribers()!=0) {
        nav_msgs::MapMetaData mdata=getMetaData();
        meta_pub.publish(mdata);
    }

    worldChangedSinceLastAdvert=false;
}


void GazeboMapPublisher::onWorldUpdate() {
    worldChangedSinceLastAdvert=true; 
}

//see http://gazebosim.org/wiki/Tutorials/1.4/custom_messages#Collision_Map_Creator_Plugin
bool GazeboMapPublisher::createMap(const CollisionMapRequest &msg, const std::string& map_frame, nav_msgs::OccupancyGrid& map) {
    /*MSG_PRINT("Creating collision map with corners at (" <<
                 msg.upperLeft.x() << ", " << msg.upperLeft.y() << "), (" <<
                 msg.upperRight.x() << ", " << msg.upperRight.y() << "), (" <<
                 msg.lowerRight.x() << ", " << msg.lowerRight.y() << "), (" <<
                 msg.lowerLeft.x() << ", " << msg.lowerLeft.y() << ") with collision projected from z = " <<
                 msg.height << "\nResolution = " << msg.resolution << " m\n" <<
                 "Occupied spaces will be filled with: " << msg.threshold);
    */
    double z = msg.height;
    double dX_vertical = msg.upperLeft.x() - msg.lowerLeft.x();
    double dY_vertical = msg.upperLeft.y() - msg.lowerLeft.y();
    double mag_vertical = sqrt(dX_vertical * dX_vertical + dY_vertical * dY_vertical);
    dX_vertical = msg.resolution * dX_vertical / mag_vertical;
    dY_vertical = msg.resolution * dY_vertical / mag_vertical;

    double step_s = msg.resolution;

    double dX_horizontal = msg.upperRight.x() - msg.upperLeft.x();
    double dY_horizontal = msg.upperRight.y() - msg.upperLeft.y();
    double mag_horizontal = sqrt(dX_horizontal * dX_horizontal + dY_horizontal * dY_horizontal);
    dX_horizontal = msg.resolution * dX_horizontal / mag_horizontal;
    dY_horizontal = msg.resolution * dY_horizontal / mag_horizontal;



    int count_vertical = mag_vertical / msg.resolution;
    int count_horizontal = mag_horizontal / msg.resolution;

    if (count_vertical == 0 || count_horizontal == 0)
    {
        MSG_PRINT("Image has a zero dimensions, check coordinates");
        return false;
    }
    double x,y;

    unsigned int threshold=msg.threshold;
    if (threshold > MAX_MAP_VAL) threshold=MAX_MAP_VAL;
    unsigned int blank=MAX_MAP_VAL-threshold;
    unsigned int fill=MAX_MAP_VAL;
    std::vector<std::vector<int> > grid;

    double dist;
    std::string entityName;
    GzVector3 start = gazebo::GetVector(0, 0, msg.height);
    GzVector3 end = gazebo::GetVector(0, 0, 0.001);

    gazebo::physics::PhysicsEnginePtr engine = gazebo::GetPhysics(world);
    engine->InitForThread();
    //gazebo::physics::RayShapePtr ray = boost::shared_dynamic_cast<gazebo::physics::RayShape>(
    gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
          engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    //MSG_PRINT("Rasterizing model and checking collisions");
    grid.resize(count_horizontal,std::vector<int>(count_vertical,blank));

    for (int i = 0; i < count_vertical; ++i) {
        //MSG_PRINT("Percent complete: " << i * 100.0 / count_vertical);
        x = i * dX_vertical + msg.lowerLeft.x();
        y = i * dY_vertical + msg.lowerLeft.y();
        for (int j = 0; j < count_horizontal; ++j) {
            x += dX_horizontal;
            y += dY_horizontal;

            gazebo::SetX(start, x);
            gazebo::SetX(end, x);
            gazebo::SetY(start, y);
            gazebo::SetY(end, y);
            ray->SetPoints(start, end);
            ray->GetIntersection(dist, entityName);
            if (!entityName.empty()) {
                std::string model=entityName.substr(0,entityName.find("::"));
                //MSG_PRINT("Collides: "<<entityName<<" robot name="<<ROBOT_NAME<<" model: "<<model);
                if (model!=ROBOT_NAME) 
                    grid[i][j]=fill;
            }
        }
    }

    map.header.frame_id=map_frame;
    map.header.stamp=ros::Time::now();
    
    nav_msgs::MapMetaData meta=getMetaData();
    map.info=meta;

    /*map.info.map_load_time=ros::Time::now();
    map.info.resolution=msg.resolution;
    map.info.width=count_horizontal;
    map.info.height=count_vertical;
    map.info.origin.position.x=msg.lowerLeft.x();
    map.info.origin.position.y=msg.lowerLeft.y();
    map.info.origin.position.z=0;
    map.info.origin.orientation.x=0;
    map.info.origin.orientation.y=0;
    map.info.origin.orientation.z=0;
    map.info.origin.orientation.w=1;
    MSG_PRINT("Meta compare: "<<std::endl<<meta<<"===="<<std::endl<<map.info);
    */

    //MSG_PRINT("Completed calculations, created map");
    
    int _x=0;
    int _y=0;
    for (std::vector<std::vector<int> >::iterator yIt=grid.begin(); yIt!=grid.end(); ++yIt) {
        _x=0;
        for (std::vector<int>::iterator xIt=yIt->begin(); xIt!=yIt->end(); ++xIt) {
            map.data.push_back(*xIt);
            ++_x;
        }
        ++_y;
    }
    return true;
}

GazeboMapPublisher::CollisionMapRequest GazeboMapPublisher::getCollisionRequest(){
    CollisionMapRequest r;
    r.lowerLeft= Eigen::Vector2i(map_offset_x,map_offset_y);
    r.lowerRight=Eigen::Vector2i(map_offset_x+map_len_x,map_offset_y);
    r.upperLeft= Eigen::Vector2i(map_offset_x,map_offset_y+map_len_y);
    r.upperRight=Eigen::Vector2i(map_offset_x+map_len_x,map_offset_y+map_len_y);
    r.resolution=map_resolution;
    r.height=map_height;
    r.threshold=MAX_MAP_VAL;
    return r;
}


nav_msgs::MapMetaData GazeboMapPublisher::getMetaData() {
    nav_msgs::MapMetaData mdata;
    mdata.map_load_time=ros::Time::now();
    mdata.resolution=map_resolution;
    mdata.width=map_len_x/map_resolution;
    mdata.height=map_len_y/map_resolution;
    mdata.origin.position.x=map_offset_x;
    mdata.origin.position.y=map_offset_y;
    mdata.origin.position.z=0;
    mdata.origin.orientation.w=0;
    return mdata;
}


nav_msgs::OccupancyGrid GazeboMapPublisher::getMap() {
    nav_msgs::OccupancyGrid map;

#if 1
    CollisionMapRequest r=getCollisionRequest();    
    if (!createMap(r,MAP_FRAME,map)) {
        ROS_ERROR("Could not request map");
    }    
#else
    map.header.frame_id=MAP_FRAME;
    map.header.stamp=ros::Time::now();
    
    map.info=getMetaData();

    float map_cells_x=floor(map_len_x/map_resolution);
    float map_cells_y=floor(map_len_y/map_resolution);
    //create an empty occupancy grid
    map.data.resize(map_cells_x*map_cells_y,0);
    
    //... fill it with data!
    //for each object found in gazebo, locate it on the map and check if it intersects a cell,
    //and put a value of 100 in that cell.

    //skip the robot from the occupancy grid!
#endif
    return map;
}


