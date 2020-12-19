// Author: Jonathan Sprinkle (modified by Matt Bunting)
// This (very simple) node reads a laser scan, and 
// publishes the distance to the nearest point based on the steering angle and bounds
//
// TODO: ensure nearest few points are somewhat close
// TODO: what to return if no closest points?
// TODO: enable angle range we care about
// TODO: enable distance range we care about

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Wrench.h"

#include <cstdio>
#include <cstdlib>
#include<ctime>

//#define DEBUG

// this global var holds the distance
std_msgs::Float64 angle;
std_msgs::Float64 dist;
std_msgs::Float64 Xdist;
std_msgs::Float64 Ydist;
std_msgs::Float64 dist_1;
std_msgs::Float64 vel;
std_msgs::Float64 vel_1;

// memory for bad data
std_msgs::Float64 vel_lastGood;
std_msgs::Float64 dist_lastGood;
float weightedVel;

std_msgs::Float64 accel;
ros::Time lastUpdate, currentUpdateTime;

std::vector<std_msgs::Float64> distVector;
std::vector<std_msgs::Float64> velVector;
std::vector<double> timeVector;

ros::Duration deltaT;
bool newMessage;

double tmin, tmax, smin, smax, closestPoint_s, x, y;

// This is annoying, but the simulation does not match the car. 
// This becomes set based on subscribing to /catvehicle/steering (not 1.0) or something else (1.0)
double SteeringAngleFudgeFactor = 1.0;

// From Benni:
double latestRho;		// Current turning radius
double epsilon = 0.005;	// Minimum turning angle magnitude before using straight distance
struct RingCoordinate {
	double s;	
	double t;
};

double steeringAngle = 0;
double vehicleSpeed = 0.0;	// TODO: Should be updated from ROS 
const double wheelBase = 2.62;	// From TORC
double understeerCoefficient = 0.0015;	// from TORC

// Use TORC's model to compute turning radius.
// A negative value represents a right turn, positive for left.
double turningAngleToRadius( double turningAngleInRadians, double speed ) {
	return wheelBase / tan(turningAngleInRadians/(1.0 + understeerCoefficient* speed*speed));
}

// Convert laser data to ring coordinates
RingCoordinate getRingCoordinates(double turningRadius, double laserRadius, double laserAngle) {
	RingCoordinate result;

	double theta = M_PI/2.0 - laserAngle;
	bool reverse = false;
	if(turningRadius < 0) {	// Turning right...?
		//theta = laserAngle - M_PI/2.0;
		theta = M_PI - theta;
		turningRadius = fabs(turningRadius);
		reverse = true;
	}

	double t_bar = sqrt(turningRadius*turningRadius + laserRadius*laserRadius - 2.0*turningRadius*laserRadius*cos(theta));	

	result.t = t_bar - turningRadius;

	result.t = reverse ? -result.t : result.t;

	//double alpha = asin(sin(theta)/t_bar * laserRadius);
	// I had to switch this from Law of Sin to Law of Cos deus to domain/range issues:
	double alpha = acos((t_bar*t_bar + turningRadius*turningRadius - laserRadius*laserRadius) / (2.0 * t_bar * turningRadius));

	result.s = alpha * turningRadius;

	return result;
}

int reportCount = 0;
void steeringCallback( const geometry_msgs::Wrench::ConstPtr& steering )
{
	steeringAngle = steering->torque.z * SteeringAngleFudgeFactor;
	latestRho = turningAngleToRadius( steeringAngle, vehicleSpeed);
#ifdef DEBUG
	if(reportCount++ > 10) {
		ROS_INFO_STREAM("steeringAngle = " << steeringAngle*180.0/M_PI << " degrees");
		ROS_INFO_STREAM("radius = " << latestRho);
		reportCount = 0;
	}
#endif
}

// This very simple callback looks through the data array, and then
// returns the value (not the index) of that distance
void scanCallback( const sensor_msgs::LaserScan::ConstPtr& scan )
{
//ROS_INFO_STREAM("scanCallback");

    // update what was 1 time ago
    /*
       dist_1 = dist_lastGood;
       vel_1 = vel;
     */
    // we weight prev observations of distance by 0.8 for velocity estimates
    //   weightedVel=vel_1.data;
    double factor = 5.0;


    //float velTmp=vel_1.data*0.99;

    dist_1.data = dist.data;
    lastUpdate = currentUpdateTime;
    currentUpdateTime = scan->header.stamp;
    deltaT = currentUpdateTime - lastUpdate;
    //    ROS_INFO_STREAM(lastUpdate); 
    dist.data = scan->range_max;
    angle.data = scan->angle_min;
    float angle_tmp=scan->angle_min;	// Starts at -pi/2
    float inc = scan->angle_increment;
    closestPoint_s = scan->range_max;

	RingCoordinate mRingCoordinate, mMin;
	bool useStraightCalculation;
	bool GoodStuff = false;

	double rho = latestRho;	// Turning radius
	double r;	// laser measured distance

	double minT;	//for debugging

	// Benni's implementation:
	if (epsilon < steeringAngle && steeringAngle < M_PI/2.0) {
		useStraightCalculation = false;
	} else if (-epsilon > steeringAngle && steeringAngle > -M_PI/2.0) {
		useStraightCalculation = false;
	} else { // Traditional cartesion estimator (straight line):
		useStraightCalculation = true;
	}

    for( std::vector<float>::const_iterator it=scan->ranges.begin();
            it!=scan->ranges.end(); it++, angle_tmp=angle_tmp+inc )
    {
		r = *it;

		// Check if sensor saw anything:
		if(r >= scan->range_max) {
			continue;
		}

		// From cartesion implementation, used for publishing data:
		x = r*cos(angle_tmp + M_PI/2.0);
		y = r*sin(angle_tmp + M_PI/2.0);
		
		// Determine The right method for the ring coordinates:
		if(useStraightCalculation) {
			mRingCoordinate.t = x;
			mRingCoordinate.s = y;
		} else {
			mRingCoordinate = getRingCoordinates(rho, r, angle_tmp);
		}

		// Update the closest pointk, if in bounds:
        if(	mRingCoordinate.t > tmin && 
			mRingCoordinate.t < tmax && 
			mRingCoordinate.s > smin && 
			mRingCoordinate.s < smax)
        {
            closestPoint_s = mRingCoordinate.s;
            
            if(closestPoint_s  < dist.data)
            {
                dist.data = closestPoint_s;
                Xdist.data = x;
                Ydist.data = y;
                angle.data = angle_tmp;	// straight out the laser
				GoodStuff = true;
				minT = mRingCoordinate.t;
            }
        }
    }

	if(GoodStuff) {
#ifdef DEBUG
		//ROS_INFO_STREAM("useStraightCalculation = " << useStraightCalculation);
		ROS_INFO_STREAM("Closest Point = " << dist.data << "\tt = " << minT);
		//ROS_INFO_STREAM("Turn Radius =   " << rho);
		//ROS_INFO_STREAM("Xdist.data  =   " << Xdist.data);
		//ROS_INFO_STREAM("Ydist.data  =   " << Ydist.data);
		//ROS_INFO_STREAM("angle.data  =   " << angle.data);
		ROS_INFO_STREAM("rho = " << rho);
		reportCount = 0;
#endif
		newMessage = true;
		GoodStuff = false;
	} else {
        dist.data = dist_1.data;
        newMessage = true;
    }
}

int main( int argc, char **argv )
{
    // initialize global vars
    dist.data = dist_1.data = dist_lastGood.data = INT_MAX;
    vel.data = vel_1.data = 0.0;
    accel.data = 0.0;
    weightedVel = 0.0;
    Xdist.data = 81.0;
    Ydist.data = 81.0;

    std::string scan_topic;
	std::string steering_topic;
    std::string dist_topic;
    std::string Xdist_topic;
    std::string Ydist_topic;
    std::string angle_topic;
    std::string vel_topic;
    srand(time(0));

    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line. For programmatic
     * remappings you can use a different version of init() which takes remappings
     * directly, but for most command-line programs, passing argc and argv is the easiest
     * way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "DistanceSteeringBased");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    //	ros::NodeHandle n;
    // set up the handle for this node, in order to publish information
    // the node handle is retrieved from the parameters in the .launch file,
    // but we have to initialize the handle to have the name in that file.
    ros::NodeHandle n("~");

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    n.param("dist_topic", dist_topic, std::string("dist"));
    n.param("Xdist_topic", Xdist_topic, std::string("Xdist"));
    n.param("Ydist_topic", Ydist_topic, std::string("Ydist"));
    n.param("angle_topic", angle_topic, std::string("angle"));
//    n.param("vel_topic", vel_topic, std::string("vel"));
    n.param("scan_topic", scan_topic, std::string("/catvehicle/front_laser_points"));
    n.param("steering_topic", steering_topic, std::string("/catvehicle/steering"));
    //	n.param("angle_min", angle_min, -M_PI/32.0f);
    //	n.param("angle_max", angle_max, M_PI/32.0f);

	// HACK: True steering commands absolutely do not match between the vehicle and simulation:
	if(steering_topic.compare("/catvehicle/steering") == 0) {
		// Here the subscribed topic provides a value from -100%-100%
		// Left is -100-0, Right is 0-100, therefeor this needs to be negated
		//SteeringAngleFudgeFactor = (-4.0*9.0/(20.0*M_PI))*M_PI/180.0;	// From REU Y2K group... mostly
		SteeringAngleFudgeFactor = (-1.0/100.0) * 0.536068503423;	// From output of 
	} else {
		// Here the subscribed topic should be steering angle in radians:
		SteeringAngleFudgeFactor = 1.0;
	}
    
    n.param("tmin", tmin, -3.0);
    n.param("tmax", tmax, 3.0);
    n.param("smin", smin, 0.0);
    n.param("smax", smax, 80.0);
    n.param("epsilon", epsilon, 0.005);

    ROS_INFO_STREAM("Node namespace is " << ros::this_node::getNamespace());
    ROS_INFO_STREAM("Node name is " << ros::this_node::getName( ) );


    // TODO: make this not just a float value
    ros::Publisher dist_pub = n.advertise<std_msgs::Float64>(dist_topic, 1);
    ros::Publisher Xdist_pub = n.advertise<std_msgs::Float64>(Xdist_topic, 1);
    ros::Publisher Ydist_pub = n.advertise<std_msgs::Float64>(Ydist_topic, 1);
    ros::Publisher angle_pub = n.advertise<std_msgs::Float64>(angle_topic, 1);
  //  ros::Publisher vel_pub = n.advertise<std_msgs::Float64>(vel_topic, 1);
    //  	ros::Publisher accel_pub = n.advertise<std_msgs::Float64>("/azcar_sim/accel", 100);

    // we also want to subscribe to the signaller
    ros::Subscriber sub = n.subscribe(scan_topic, 1, &scanCallback);
    ros::Subscriber substeering = n.subscribe(steering_topic, 1, &steeringCallback);

    ROS_INFO_STREAM("Looking for scan in topic " << scan_topic);
ROS_INFO_STREAM("Looking for steering in topic " << steering_topic);
	ROS_INFO_STREAM("Steering fudge factor = " << SteeringAngleFudgeFactor);


    ROS_INFO_STREAM("Publishing estimated distance to target in topic " << ros::this_node::getName( ) << "/" << dist_topic);
    ROS_INFO_STREAM("Publishing estimated angle to target in topic " << ros::this_node::getName( ) << "/" << angle_topic);

	// Corner case checking:
	{
		// Let's test the epsilon value:
		// Consider a following distance of ~15m:
		double x = tmax;
		double y = 15;
		double laserAngle = atan2(-x,y);
		double laserMag = sqrt(x*x+y*y);
		// The straight distance estimator will provide a distance equal to y.

		// At the epsilon value, let's compute the distance (assume 0 speed):
		double turnRadiusLeft =  turningAngleToRadius(  epsilon, 0.0 );
		double turnRadiusRight = turningAngleToRadius( -epsilon, 0.0 );

		RingCoordinate mRingCoordinateLeft = getRingCoordinates(turnRadiusLeft, laserMag, laserAngle);
		RingCoordinate mRingCoordinateRight = getRingCoordinates(turnRadiusRight, laserMag, laserAngle);

		ROS_INFO_STREAM("Consider a point at ("<<x<<","<<y<<") in front of the laser:");
		ROS_INFO_STREAM("Laser Mag = "<<laserMag<<"\tangle="<<laserAngle);
		ROS_INFO_STREAM("Straight: t = "<<x<<"\ts="<<y);
		ROS_INFO_STREAM("Right:    t = "<<mRingCoordinateRight.t<<"\ts="<<mRingCoordinateRight.s);
		ROS_INFO_STREAM("Left:     t = "<<mRingCoordinateLeft.t<<"\ts="<<mRingCoordinateLeft.s);
		double errorLeft = fabs(y - mRingCoordinateLeft.s);
		double errorRight = fabs(y - mRingCoordinateRight.s);
		ROS_INFO_STREAM("Expeceted error based on Epsilon: " << (errorLeft > errorRight ? errorLeft : errorRight));
	}

    // run at 1kHz?
    ros::Rate loop_rate(1000);
    lastUpdate = ros::Time();
    newMessage = false;

    while( ros::ok() )
    {
        //		ROS_INFO( "Nearest object=%lf", dist );

        // TODO: publish only if the time stamp is newer from the sensor
        double eps = (double)1/(double)100; // the epsilon time that is 'equal' in diff
        if( newMessage )
        {
            dist_pub.publish(dist);
            Xdist_pub.publish(Xdist);
            Ydist_pub.publish(Ydist);
            angle_pub.publish(angle);
    //        vel_pub.publish(vel);
            newMessage = false;
        }
        //        vel_pub.publish(vel);
        //        accel_pub.publish(accel);
        ros::spinOnce( );
        loop_rate.sleep( );

    }

    return EXIT_SUCCESS;
}

