// Author: Jonathan Sprinkle
// This (very simple) node reads a laser scan, and 
// publishes the distance to the nearest point
//
// TODO: ensure nearest few points are somewhat close
// TODO: what to return if no closest points?
// TODO: enable angle range we care about
// TODO: enable distance range we care about

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include <cassert>

#include <cstdio>
#include <cstdlib>

/*
Updated version to account for a window size that varies
based on the latest discontinuity in the data.
*/

#define LOOKBACK 80
#define CURRENT (LOOKBACK-1)
#define K 80
#define nMin 3
#define alpha 0.15
#define MAXDISTANCE 80
#define LASERRATE 75

#define USEOFFSET
//#define USEOFFSETBENNI

#ifdef USEOFFSET
double distanceOffset=0.0;
#endif


#define LASTTWENTY(VARNAME) \
VARNAME[CURRENT] << "," << VARNAME[CURRENT-1] << "," << \
            VARNAME[CURRENT-2] << "," << VARNAME[CURRENT-3] << "," <<\
            VARNAME[CURRENT-4] << "," << VARNAME[CURRENT-5] << "," <<\
            VARNAME[CURRENT-6] << "," << VARNAME[CURRENT-7] << "," <<\
            VARNAME[CURRENT-8] << "," << VARNAME[CURRENT-9] << "," <<\
            VARNAME[CURRENT-10] << "," << VARNAME[CURRENT-11] << "," <<\
            VARNAME[CURRENT-12] << "," << VARNAME[CURRENT-13] << "," <<\
            VARNAME[CURRENT-14] << "," << VARNAME[CURRENT-15] << "," <<\
            VARNAME[CURRENT-16] << "," << VARNAME[CURRENT-17] << "," <<\
            VARNAME[CURRENT-18] << "," << VARNAME[CURRENT-19]

#define LASTTWENTYDISTANCE(VARNAME) \
VARNAME[CURRENT].distance << "," << VARNAME[CURRENT-1].distance << "," << \
VARNAME[CURRENT-2].distance << "," << VARNAME[CURRENT-3].distance << "," <<\
VARNAME[CURRENT-4].distance << "," << VARNAME[CURRENT-5].distance << "," <<\
VARNAME[CURRENT-6].distance << "," << VARNAME[CURRENT-7].distance << "," <<\
VARNAME[CURRENT-8].distance << "," << VARNAME[CURRENT-9].distance << "," <<\
VARNAME[CURRENT-10].distance << "," << VARNAME[CURRENT-11].distance << "," <<\
VARNAME[CURRENT-12].distance << "," << VARNAME[CURRENT-13].distance << "," <<\
VARNAME[CURRENT-14].distance << "," << VARNAME[CURRENT-15].distance << "," <<\
VARNAME[CURRENT-16].distance << "," << VARNAME[CURRENT-17].distance << "," <<\
VARNAME[CURRENT-18].distance << "," << VARNAME[CURRENT-19].distance


// this global var holds the distance
std_msgs::Float64 angle;
std_msgs::Float64 dist;
std_msgs::Float64 dist_1;
std_msgs::Float64 vel;
std_msgs::Float64 vel_1;

// memory for bad data
std_msgs::Float64 vel_lastGood;
std_msgs::Float64 dist_lastGood;
double weightedVel;

// memory array for filtered distance
typedef struct distanceInfo_tag {
    ros::Time stamp;
    double distance;
} distanceInfo;
distanceInfo distances[LOOKBACK];
// raw distance information
distanceInfo rawDistances[LOOKBACK];
// memory array for pairwise computed (not published) velocities
double vels[LOOKBACK];
// memory array for published velocities
double velsOut[LOOKBACK];

std_msgs::Float64 accel;
ros::Time lastUpdate;
ros::Time startUp;
bool newMessage;
double angle_min;
double angle_max;

// This very simple callback looks through the data array, and then
// returns the value (not the index) of that distance
void distCallback( const std_msgs::Float64::ConstPtr& distance )
{

    dist.data = distance->data;
    ros::Time nowtime = ros::Time::now();
    int n=0;
    double velTmp=0.0;

    // shift the data in memory back
    for( int i=1; i<LOOKBACK; i++ )
    {
        distances[i-1] = distances[i];
        rawDistances[i-1] = rawDistances[i];
        vels[i-1] = vels[i];
        velsOut[i-1] = velsOut[i];
    }

    // Note: we don't use exact times, because of clock jitter
    // in the sensor acquisition in ROS; rather, we stick with
    // 75.0 Hz and trust the sensor hardare

    if( dist.data < MAXDISTANCE )
    {
        dist_lastGood = dist;
//        velTmp = (dist.data - dist_1.data)/(1.0/75.0);
        // add to the memory array for distance
        distances[CURRENT].distance = dist.data;
        // fixed the deltaT 
        // to include the time
        distances[CURRENT].stamp = nowtime;

        rawDistances[CURRENT]=distances[CURRENT];
    }
    else
    {
        distances[CURRENT]=distances[CURRENT-1];
        // we still ahve to use the stamp, or we get weird vels later
        distances[CURRENT].stamp = nowtime;
        rawDistances[CURRENT]=distances[CURRENT];
    }

#ifdef USEOFFSET
    double distDiff=rawDistances[CURRENT].distance
        -rawDistances[CURRENT-1].distance;
    if( fabs(distDiff) > alpha && 
        distances[CURRENT-1].distance != 0 )
    {
        double distanceOffset_old=distanceOffset;
        distanceOffset-=distDiff;
        ROS_INFO_STREAM("Found a jump of " << distDiff << 
            ", changing distanceOffset from " << distanceOffset_old << 
            " to " << distanceOffset);
        // update the distance offset
    }
    distances[CURRENT].distance += distanceOffset;
#endif

    // add information to the memory array
//    vel.data = 0.9*weightedVel + 0.1*velTmp;

    // on the very first time step, we ought not make a velocity
    if( distances[CURRENT-1].distance == 0 
        && distances[CURRENT].distance != 0)
    {
        ROS_INFO_STREAM("Setting velocity=0 for initial step steps.");
        vels[CURRENT]=0;
    }
    else
    {
        double deltaT=0.0;
#define FORCECONSTANTTIME 1
#if FORCECONSTANTTIME
        deltaT = (double)1.0/(double)75.0;
#else
        ros::Duration deltaT_ros = distances[CURRENT].stamp -
                                distances[CURRENT-1].stamp;
        deltaT = double(deltaT_ros.nsec)/1000000000.0;
        if( fabs(deltaT - 0) < 0.0001 ) 
        { 
            deltaT = (double)1.0/(double)75.0; 
        }
        else if( deltaT > 1.1*(1.0/75.0) )
        {
            ROS_INFO_STREAM("Aha! Found a timing problem (deltaT=" <<
                deltaT << ")");
            deltaT = (double)1.0/(double)75.0;
        }
#endif
        vels[CURRENT]=(distances[CURRENT].distance
                        -distances[CURRENT-1].distance)/deltaT;

    }

    // check on the piecewise nature of the signal, to see whether
    // we should set a new window width

    // contains index values where we have a piecewise jump
    std::vector<int> indices;
#ifndef USEOFFSET
    int count1=0;
    for( int jj=0; jj<LOOKBACK-1; jj++ )
    {
//        if( fabs(vels[CURRENT]) > alpha*75 )
        if( fabs(distances[jj+1].distance
                -distances[jj].distance) > alpha )
        {
            indices.push_back(jj+1);
        }
        count1++;
    }
    assert(count1 = 19);
#endif
    // if the indices vector is empty, we can take as long of a look
    // as we like at the data points
    if( indices.size() == 0 )
    {
        n=K; // we can take 20 values
    }
    else
    {
        // if there are 3 element in indices, then indices[2] will
        // hold the largest index
        int lastIndex=indices.size()-1;
        // and we want to be one index less than this in terms
        // of items that we will count; say that indices[2] holds
        // 15: we want to look at the last 3 pairwise items
        // dist[19]-dist[18]
        // dist[18]-dist[17]
        // dist[17]-dist[16]
        // dist[16]-dist[15] --> NO, dist[15] is erroneous [XXX]
        // so, K=20 (index 19), we want to look at the last 
        // 3 entries: (K-1)-15-1 = 3
        //               ^   ^ ^
        // 0 indexing in C   | |
        // indices[lastIndex]  |
        // 1 more offset to rule out [XXX] above
        n=(K-1)-indices[lastIndex]-1;
    }

    // if n > nMin, we can make a new data point
    // else, we multiply our velocity by a decay value
    velTmp=0.0;
    if( n > nMin )
    {
        int count=0;
        // we would like an even number if our horizon is
        // large enough
//        if( n % 2 == 1 && n > (LOOKBACK/2.0) )
//        {
//           n = n-1;
//        }
        if( n >= LOOKBACK-1 )
        {
           n = LOOKBACK-2;
        }
        // we need to go all the way back to 1st index, 
        // which give us 19 estimates (the 20 diffs)
        for( int kk=CURRENT; kk>=LOOKBACK-n; kk-- )
//        for( int kk=CURRENT; kk>LOOKBACK-n; kk-- )
        {
            count++;
            velTmp+=vels[kk];
//            vels[CURRENT]=vels[CURRENT]+vels[kk];
        }
        // now, divide through by the total, n
//        ROS_INFO_STREAM("count=="<<count<<",n="<<n);
        assert(count==(n));
        velTmp/=(double)n;
    }
    else
    {
        ROS_INFO_STREAM("Not enough data points for velocity sample (" 
            << "time=" << (ros::Time::now()-startUp)
            << ",n=" << n << ",lastVel=" <<velsOut[CURRENT-1] << ")" );
        ROS_INFO_STREAM("Last 20 vels:" << LASTTWENTY(velsOut)
            ); 
        ROS_INFO_STREAM("Last 20 dists:" << LASTTWENTYDISTANCE(distances)
            ); 
        // republish the data from last time
        velTmp = velsOut[CURRENT-1];
    }
    velsOut[CURRENT]=velTmp;
    vel.data=velTmp;

    newMessage = true;

    if( fabs(velsOut[CURRENT]-velsOut[CURRENT-1]) > 1 )
    {
            ROS_INFO_STREAM("Found an interesting point (" 
            << "time=" << (ros::Time::now()-startUp)
            << ",n=" << n << ",lastVelOut=" <<velsOut[CURRENT-1] << ")" );
        ROS_INFO_STREAM("Last 20 vels:" << LASTTWENTY(vels)
            ); 
        ROS_INFO_STREAM("Last 20 dists:" << LASTTWENTYDISTANCE(distances)
            ); 
        ROS_INFO_STREAM("Last 20 velsOut:" << LASTTWENTY(velsOut)
            ); 
        ROS_INFO_STREAM("Last 20 rawDists:" << LASTTWENTYDISTANCE(rawDistances)
            ); 

        if( indices.size() > 0 )
        {
            ROS_INFO_STREAM("indices.size()=" << indices.size() <<
                ",indices[indices.size()-1]=" << indices[indices.size()-1] );
        } 
        else 
        {
            ROS_INFO_STREAM("indices is empty");
        }
        // republish the data from last time
//        velTmp = velsOut[CURRENT-1];
    }

    // if vel is greater now than it was before, then
    // accel is greater (i.e., they are accel away from us)
    accel.data = (vel.data - vel_1.data)/0.02;
}

int main( int argc, char **argv )
{
    // initialize global vars
    dist.data = dist_1.data = dist_lastGood.data = 0.0;
    vel.data = vel_1.data = 0.0;
    accel.data = 0.0;
    weightedVel = 0.0;

    std::string dist_topic;
//    std::string angle_topic;
    std::string vel_topic;

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
	ros::init(argc, argv, "VelocityEstimator");

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

    // initialize the lookback arrays
    for( int i=0; i<LOOKBACK; i++ )
    {
        vels[i] = 0.0;
        velsOut[i] = 0.0;
        distances[i].distance=0.0;
        distances[i].stamp=ros::Time::now();
        rawDistances[i].distance=0.0;
        rawDistances[i].stamp=ros::Time::now();
    }

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
//	n.param("angle_topic", angle_topic, std::string("angle"));
	n.param("vel_topic", vel_topic, std::string("vel"));

    ROS_INFO_STREAM("Node namespace is " << ros::this_node::getNamespace());
    ROS_INFO_STREAM("Node name is " << ros::this_node::getName( ) );


  	ros::Publisher vel_pub = n.advertise<std_msgs::Float64>(vel_topic, 1);
//  	ros::Publisher accel_pub = n.advertise<std_msgs::Float2>("/azcar_sim/accel", 100);

  	// we also want to subscribe to the signaller
  	ros::Subscriber sub1 = n.subscribe(dist_topic, 1, &distCallback);
//  	ros::Subscriber sub2 = n.subscribe(angle_topic, 1, &angleCallback);

    ROS_INFO_STREAM("Looking for dist in topic " << dist_topic);
    ROS_INFO_STREAM("Publishing estimated velocity as " << ros::this_node::getName( ) << "/" << vel_topic);

    startUp = ros::Time::now();
    ROS_INFO_STREAM("Start up time is " << startUp );

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
            vel_pub.publish(vel);
            newMessage = false;
        }
//        vel_pub.publish(vel);
//        accel_pub.publish(accel);
		ros::spinOnce( );
		loop_rate.sleep( );

	}

	return EXIT_SUCCESS;
}

