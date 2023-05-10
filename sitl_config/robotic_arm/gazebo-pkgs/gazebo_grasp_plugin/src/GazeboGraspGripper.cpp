#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include <gazebo_grasp_plugin/GazeboGraspGripper.h>
#include <gazebo_version_helpers/GazeboVersionHelpers.h>

using gazebo::GazeboGraspGripper;

#define DEFAULT_FORCES_ANGLE_TOLERANCE 120
#define DEFAULT_UPDATE_RATE 5
#define DEFAULT_MAX_GRIP_COUNT 10
#define DEFAULT_RELEASE_TOLERANCE 0.005
#define DEFAULT_DISABLE_COLLISIONS_ON_ATTACH false

///////////////////////////////////////////////////////////////////////////////
GazeboGraspGripper::GazeboGraspGripper():
  attached(false)
{
}

///////////////////////////////////////////////////////////////////////////////
GazeboGraspGripper::GazeboGraspGripper(const GazeboGraspGripper &o):
  model(o.model),
  gripperName(o.gripperName),
  linkNames(o.linkNames),
  collisionElems(o.collisionElems),
  fixedJoint(o.fixedJoint),
  palmLink(o.palmLink),
  disableCollisionsOnAttach(o.disableCollisionsOnAttach),
  attached(o.attached),
  attachedObjName(o.attachedObjName)
{}

///////////////////////////////////////////////////////////////////////////////
GazeboGraspGripper::~GazeboGraspGripper()
{
  this->model.reset();
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboGraspGripper::Init(physics::ModelPtr &_model,
                              const std::string &_gripperName,
                              const std::string &palmLinkName,
                              const std::vector<std::string> &fingerLinkNames,
                              bool _disableCollisionsOnAttach,
                              std::map<std::string, physics::CollisionPtr> &_collisionElems)
{
  this->gripperName = _gripperName;
  this->attached = false;
  this->disableCollisionsOnAttach = _disableCollisionsOnAttach;
  this->model = _model;
  physics::PhysicsEnginePtr physics =
    gazebo::GetPhysics(this->model->GetWorld());
  this->fixedJoint = physics->CreateJoint("revolute");

  this->palmLink = this->model->GetLink(palmLinkName);
  if (!this->palmLink)
  {
    gzerr << "GazeboGraspGripper: Palm link " << palmLinkName <<
          " not found. The gazebo grasp plugin will not work." << std::endl;
    return false;
  }
  for (std::vector<std::string>::const_iterator fingerIt =
         fingerLinkNames.begin();
       fingerIt != fingerLinkNames.end(); ++fingerIt)
  {
    physics::LinkPtr link = this->model->GetLink(*fingerIt);
    //gzmsg<<"Got link "<<fingerLinkElem->Get<std::string>()<<std::endl;
    if (!link.get())
    {
      gzerr << "GazeboGraspGripper ERROR: Link " << *fingerIt <<
            " can't be found in gazebo for GazeboGraspGripper model plugin. Skipping." <<
            std::endl;
      continue;
    }
    for (unsigned int j = 0; j < link->GetChildCount(); ++j)
    {
      physics::CollisionPtr collision = link->GetCollision(j);
      std::string collName = collision->GetScopedName();
      //collision->SetContactsEnabled(true);
      std::map<std::string, physics::CollisionPtr>::iterator collIter =
        collisionElems.find(collName);
      if (collIter !=
          this->collisionElems.end())   //this collision was already added before
      {
        gzwarn << "GazeboGraspGripper: Adding Gazebo collision link element " <<
               collName << " multiple times, the gazebo grasp handler may not work properly" <<
               std::endl;
        continue;
      }
      this->collisionElems[collName] = collision;
      _collisionElems[collName] = collision;
    }
  }
  return !this->collisionElems.empty();
}

///////////////////////////////////////////////////////////////////////////////
const std::string &GazeboGraspGripper::getGripperName() const
{
  return gripperName;
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboGraspGripper::hasLink(const std::string &linkName) const
{
  for (std::vector<std::string>::const_iterator it = linkNames.begin();
       it != linkNames.end(); ++it)
  {
    if (*it == linkName) return true;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////
bool GazeboGraspGripper::hasCollisionLink(const std::string &linkName) const
{
  return collisionElems.find(linkName) != collisionElems.end();
}


///////////////////////////////////////////////////////////////////////////////
bool GazeboGraspGripper::isObjectAttached() const
{
  return attached;
}

///////////////////////////////////////////////////////////////////////////////
const std::string &GazeboGraspGripper::attachedObject() const
{
  return attachedObjName;
}



///////////////////////////////////////////////////////////////////////////////
// #define USE_MODEL_ATTACH // this only works if the object is a model in itself, which is usually not
// the case. Leaving this in here anyway for documentation of what has been
// tried, and for and later re-evaluation.
bool GazeboGraspGripper::HandleAttach(const std::string &objName)
{
  if (!this->palmLink)
  {
    gzwarn << "GazeboGraspGripper: palm link not found, not enforcing grasp hack!\n"
           << std::endl;
    return false;
  }
  physics::WorldPtr world = this->model->GetWorld();
  if (!world.get())
  {
    gzerr << "GazeboGraspGripper: world is NULL" << std::endl;
    return false;
  }
#ifdef USE_MODEL_ATTACH
  physics::ModelPtr obj = world->GetModel(objName);
  if (!obj.get())
  {
    std::cerr << "ERROR: Object ModelPtr " << objName <<
              " not found in world, can't attach it" << std::endl;
    return false;
  }
  gazebo::math::Pose diff = obj->GetLink()->GetWorldPose() -
                            this->palmLink->GetWorldPose();
  this->palmLink->AttachStaticModel(obj, diff);
#else
  physics::CollisionPtr obj =
    boost::dynamic_pointer_cast<physics::Collision>(gazebo::GetEntityByName(world, objName));
  if (!obj.get())
  {
    std::cerr << "ERROR: Object " << objName <<
              " not found in world, can't attach it" << std::endl;
    return false;
  }
  gazebo::GzPose3 diff = gazebo::GetWorldPose(obj->GetLink()) -
                         gazebo::GetWorldPose(this->palmLink);
  this->fixedJoint->Load(this->palmLink, obj->GetLink(), diff);
  this->fixedJoint->Init();
#if GAZEBO_MAJOR_VERSION >= 8
  this->fixedJoint->SetUpperLimit(0, 0);
  this->fixedJoint->SetLowerLimit(0, 0);
#else
  this->fixedJoint->SetHighStop(0, 0);
  this->fixedJoint->SetLowStop(0, 0);
#endif

  if (this->disableCollisionsOnAttach)
  {
    // we can disable collisions of the grasped object, because when the fingers keep colliding with
    // it, the fingers keep wobbling, which can create difficulties when moving the arm.
    obj->GetLink()->SetCollideMode("none");
  }
#endif  // USE_MODEL_ATTACH
  this->attached = true;
  this->attachedObjName = objName;
  return true;
}

///////////////////////////////////////////////////////////////////////////////
void GazeboGraspGripper::HandleDetach(const std::string &objName)
{
  physics::WorldPtr world = this->model->GetWorld();
  if (!world.get())
  {
    gzerr << "GazeboGraspGripper: world is NULL" << std::endl << std::endl;
    return;
  }
#ifdef USE_MODEL_ATTACH
  physics::ModelPtr obj = world->GetModel(objName);
  if (!obj.get())
  {
    std::cerr << "ERROR: Object ModelPtr " << objName <<
              " not found in world, can't detach it" << std::endl;
    return;
  }
  this->palmLink->DetachStaticModel(objName);
#else
  physics::CollisionPtr obj = boost::dynamic_pointer_cast<physics::Collision>
                              (gazebo::GetEntityByName(world, objName));
  if (!obj.get())
  {
    std::cerr << "ERROR: Object " << objName <<
              " not found in world, can't attach it" << std::endl;
    return;
  }
  else if (this->disableCollisionsOnAttach)
  {
    obj->GetLink()->SetCollideMode("all");
  }

  // TODO: remove this test print, for issue #26 ------------------- 
#if 0
  if (obj && obj->GetLink())
  {
    auto linVel = GetWorldVelocity(obj->GetLink());
    gzmsg << "PRE-DETACH Velocity for link " << obj->GetLink()->GetName()
          << " (collision name " << objName << "): " << linVel
          << ", absolute val " << GetLength(linVel) << std::endl;
  }
#endif
  // ------------------- 

  this->fixedJoint->Detach();

  // TODO: remove this test print, for issue #26 ------------------- 
#if 0
  if (obj && obj->GetLink())
  {
    auto linVel = GetWorldVelocity(obj->GetLink());
    gzmsg << "POST-DETACH Velocity for link " << obj->GetLink()->GetName()
          << " (collision name " << objName << "): " << linVel
          << ", absolute val " << GetLength(linVel) << std::endl;

  }
#endif
  // ------------------- 

#endif  // USE_MODEL_ATTACH
  this->attached = false;
  this->attachedObjName = "";
}
