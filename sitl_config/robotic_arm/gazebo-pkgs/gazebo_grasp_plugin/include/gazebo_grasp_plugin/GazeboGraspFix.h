#ifndef GAZEBO_GAZEBOGRASPFIX_H
#define GAZEBO_GAZEBOGRASPFIX_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <stdio.h>
#include <gazebo_grasp_plugin/GazeboGraspGripper.h>

namespace gazebo
{

/**
 * Inspired by gazebo::physics::Gripper, this plugin fixes an object which is grasped to the
 * robot hand to avoid problems with physics engines and to help the object staying in
 * the robot hand without slipping out.
 *
 * This is a *model* plugin, so you have to load the model plugin from the robot URDF:
 *
 * ```xml
 *   <gazebo>
 *       <plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">
 *          <arm>
 *              <arm_name>name-of-arm</arm_name>
 *              <palm_link> hand_link_name  </palm_link>
 *              <gripper_link> finger_index_link_1 </gripper_link>
 *              <gripper_link> finger_index_link_2 </gripper_link>
 *              <gripper_link> ... </gripper_link>
 *           </arm>
 *           <forces_angle_tolerance>100</forces_angle_tolerance>
 *           <update_rate>4</update_rate>
 *           <grip_count_threshold>4</grip_count_threshold>
 *           <max_grip_count>8</max_grip_count>
 *           <release_tolerance>0.005</release_tolerance>
 *           <disable_collisions_on_attach>false</disable_collisions_on_attach>
 *           <contact_topic>__default_topic__</contact_topic>
 *       </plugin>
 *   </gazebo>
 * ```
 *
 *    Description of the arguments:
 *
 *    - ``<arm>`` contains a collections of specification for each arm which can grasp one object (there may be several ``<arm>`` tags):
 *        - ``<arm_name>`` is the name of this arm. Has to be unique.
 *        - ``<palm_link>`` has to be the link to which the finger joints are attached.
 *        - ``<gripper_link>`` tags have to include -all- link names of the gripper/hand which are used to
 *              actively grasp objects (these are the links which determine whether a "grasp" exists according to
 *              above described criterion).
 *    - ``<update_rate>`` is the rate at which all contact points are checked against the "gripping criterion".
 *          Note that in-between such updates, existing contact points may be collected at
 *          a higher rate (the Gazebo world update rate). The ``update_rate`` is only the rate at
 *          which they are processed, which takes a bit of computation time, and therefore
 *          should be lower than the gazebo world update rate.
 *    - ``<forces_angle_tolerance>`` is the tolerance angle (in degrees) between two force vectors to be considered
 *           "opposing forces". If the angle is smaller than this, they are not opposing.
 *    - ``<grip_count_threshold>`` is number of times in the update loop (running at update_rate) that an object has
 *            to be detected as "gripped" in order to attach the object.
 *            Adjust this with the update rate.
 *    - ``<max_grip_count>`` is the maximum number of a counter:
 *            At each update iteration (running at update_rate), if the "gripping criterion" is
 *            met for an object, a counter for this object is increased. ``max_grip_count`` is
 *            the maximum number recorded for an object. As soon as the counter goes beyond this
 *            number, the counter is stopped. As soon as the "gripping criterion" does not
 *            hold any more, the number will start to decrease again, (by 1 each time the object
 *            is detected as "not grasped" in an update iteration). So this counter is
 *            like a "buffer" which, when it is full, maintains the state, and when it is empty,
 *            again, the object is released.
 *            This should be at least double of ``grip_count_threshold``.
 *    - ``<release_tolerance>`` is the distance which the gripper links are allowed to move away from the object
 *            during- a grasp without the object being detached, even if there are currently no
 *            actual contacts on the object. This condition can happen if the fingers "wobble"
 *            or move ever so slightly away from the object, and therefore the "gripping criterion"
 *            fails in a few subsequent update iterations. This setting is to make the behaviour more
 *            stable.
 *            Setting this number too high will also lead to the object not being detached even
 *            if the grippers have opened up to release it, so use this with care.
 *    - ``<disable_collisions_on_attach>`` can be used for the following:
 *            When an object is attached, collisions with it may be disabled, in case the
 *            robot still keeps wobbling.
 *    - ``<contact_topic>`` is the gazebo topic of contacts. Should normally be left at -\_\_default_topic\_\_-.
 *
 * Current limitations:
 *
 *  - Only one object can be attached per gripper.
 *  - Only partial support for an object cannot be gripped with two grippers (release condition may be
 *     triggered wrongly, or not at all, if two grippers are involved)
 *
 * \author Jennifer Buehler
 */
class GazeboGraspFix : public ModelPlugin
{
  public:
    GazeboGraspFix();
    GazeboGraspFix(physics::ModelPtr _model);
    virtual ~GazeboGraspFix();
  
  private:

    /**
     * Gets called just after the object has been attached to the palm link on \e armName
     */
    void OnAttach(const std::string &objectName,
                          const std::string &armName);
    /**
     * Gets called just after the object has been detached to the palm link on \e armName
     */
    void OnDetach(const std::string &objectName,
                          const std::string &armName);

    virtual void Init();
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    /**
     * Collects for each object all forces which are currently applied on it.
     * Then, for each object, checks whether of all the forces applied,
     * there are opposing forces.
     * This is done by calling CheckGrip() with the list of all forces applied.
     * If CheckGrip() returns true, the number of "grip counts"
     * is increased for the holding arm (but grip counts will never exceed \e max_grip_count).
     * If the number of grip counts for this object exceeds \e grip_count_threshold,
     * the object is attached by calling GazeboGraspGripper::HandleAttach(object-name),
     * setting \e attached and \e attachedObjName, and \e GazeboGraspGripper::attachGripContacts
     * is updated with the contact points currently existing for this object (current entry in \e contacts).
     *
     * Then, goes through all entries in \e gripCount, and unless it's an object
     * we just detected as "gripped", the counter is decreased.
     * If the counter is is smaller than \e grip_count_threshold, the object should
     * potentially be released, but this criterion happens too easily
     * (the fingers in gazebo may have started wobbling as the arm moves around, and although they are still
     * close to the object, the grip is not detected any more).
     * So to be sure, and additional criterion has to be satisfied before the object is released:
     * check that the collision point (the place on the link where the contact originally
     * was detected) has not moved too far from where it originally was, relative to the object.
     */
    void OnUpdate();

    void InitValues();


    /**
     * Gets called upon detection of contacts.
     * A list of contacts is passed in \_msg. One contact has two bodies, and only
     * the ones where one of the bodies is a gripper link are considered.
     * Each contact consists of a *list* of forces with their own origin/position each
     * (e.g. when the object and gripper are colliding at several places).
     * The averages of each contact's force vectors along with their origins is computed.
     * This "average contact force/origin" for each contact is then added to the \e this->contacts map.
     * If an entry for this object/link pair already exists, the average force (and its origin)
     * is *added* to the existing force/origin, and the average count is increased. This is to get
     * the average force application over time between link and object.
     */
    void OnContact(const ConstContactsPtr &ptr);

//    bool CheckGrip(const std::vector<GzVector3> &forces, float minAngleDiff,
//                   float lengthRatio);

    bool IsGripperLink(const std::string &linkName, std::string &gripperName) const;

    /**
     * return objects (key) and the gripper (value) to which it is attached
     */
    std::map<std::string, std::string> GetAttachedObjects() const;

    /**
     * Helper class to collect contact info per object.
     * Forward declaration here.
     */
    class ObjectContactInfo;

    /**
     * Helper function to determine if object attached to a gripper in ObjectContactInfo.
     */
    bool ObjectAttachedToGripper(const ObjectContactInfo &objContInfo,
                                 std::string &attachedToGripper) const;

    /**
     * Helper function to determine if object attached to this gripper
     */
    bool ObjectAttachedToGripper(const std::string &gripperName,
                                 std::string &attachedToGripper) const;


    // physics::ModelPtr model;
    // physics::PhysicsEnginePtr physics;
    physics::WorldPtr world;

    // sorted by their name, all grippers of the robot
    std::map<std::string, GazeboGraspGripper> grippers;

    event::ConnectionPtr update_connection;
    transport::NodePtr node;
    transport::PublisherPtr eventsPub;   // publisher of grasping events
    transport::SubscriberPtr contactSub; // subscriber to contact updates

    // tolerance (in degrees) between force vectors to
    // be considered "opposing"
    float forcesAngleTolerance;

    // when an object is attached, collisions with it may be disabled, in case the
    // robot still keeps wobbling.
    bool disableCollisionsOnAttach;

    // all collisions per gazebo collision link (each entry
    // belongs to a physics::CollisionPtr element). The key
    // is the collision link name, the value is the gripper name
    // this collision link belongs to.
    std::map<std::string, std::string> collisions;


    /**
     * Helper class to encapsulate a collision information. Forward declaration here.
     */
    class CollidingPoint;

    // Contact forces sorted by object name the gripper collides with (first key)
    // and the link colliding (second key).
    std::map<std::string, std::map<std::string, CollidingPoint> > contacts;
    boost::mutex mutexContacts; //mutex protects contacts

    // when an object was first attached, it had these colliding points.
    // First key is object name, second is the link colliding, as in \e contacts.
    // Only the links of *one* gripper are stored here. This indirectly imposes the
    // limitation that no two grippers can grasp the object (while it would be
    // possible, the release condition is tied to only one link, so the object may
    // not be released properly).
    std::map<std::string, std::map<std::string, CollidingPoint> >
    attachGripContacts;


    // Records how many subsequent update calls the grip on that object has been recorded
    // as "holding". Every loop, if a grip is not recorded, this number decreases.
    // When it reaches \e grip_count_threshold, it will be attached.
    // The number won't increase above max_grip_count once it has reached that number.
    std::map<std::string, int> gripCounts;

    // *maximum* number in \e gripCounts to be recorded.
    int maxGripCount;

    // number of recorded "grips" in the past (in gripCount) which, when it is exceeded, counts
    // as the object grasped, and when it is lower, as released.
    int gripCountThreshold;

    // once an object is gripped, the relative position of the collision link surface to the
    // object is remembered. As soon as this distance changes more than release_tolerance,
    // the object is released.
    float releaseTolerance;

    //nano seconds between two updates
    common::Time updateRate;

    //last time OnUpdate() was called
    common::Time prevUpdateTime;

    //ContactManager filter to be removed in destructor
    std::string filter_name;
};

}

#endif  // GAZEBO_GAZEBOGRASPFIX_H
