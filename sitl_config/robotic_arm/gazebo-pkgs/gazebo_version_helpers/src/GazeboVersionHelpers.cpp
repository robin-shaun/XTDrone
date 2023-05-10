#include <gazebo_version_helpers/GazeboVersionHelpers.h>
#include <gazebo/physics/physics.hh>

using gazebo::GzPose3;
using gazebo::GzVector3;
using gazebo::GzMatrix3;
using gazebo::GzMatrix4;

///////////////////////////////////////////////////////////////////////////////
GzPose3 gazebo::GetWorldPose(const gazebo::physics::LinkPtr &link)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return link->WorldPose();
#else 
  return link->GetWorldPose();
#endif 
}

///////////////////////////////////////////////////////////////////////////////
GzVector3 gazebo::GetWorldVelocity(const gazebo::physics::LinkPtr &link)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return link->WorldLinearVel();
#else 
  return link->GetWorldLinearVel();
#endif 
}
 

///////////////////////////////////////////////////////////////////////////////
GzMatrix4 gazebo::GetIdentity()
{
#if GAZEBO_MAJOR_VERSION >= 8
  return GzMatrix4::Identity;
#else 
  return GzMatrix4::IDENTITY;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
GzMatrix4 gazebo::GetMatrix(const GzPose3 &pose)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return GzMatrix4(pose);
#else 
  GzMatrix4 mat = pose.rot.GetAsMatrix4();
  mat.SetTranslate(pose.pos);
  return mat;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
GzMatrix4 gazebo::GetMatrix(const GzVector3 &pos)
{
  GzMatrix4 mat = GetIdentity();
#if GAZEBO_MAJOR_VERSION >= 8
  mat.SetTranslation(pos);
#else 
  mat.SetTranslate(pos);
#endif 
  return mat;
}

///////////////////////////////////////////////////////////////////////////////
double gazebo::GetLength(const GzVector3 &v)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return v.Length();
#else 
  return v.GetLength();
#endif 
}

///////////////////////////////////////////////////////////////////////////////
void gazebo::SetX(GzVector3 &v, const double val)
{
#if GAZEBO_MAJOR_VERSION >= 8
  v.X(val);
#else 
  v.x = val;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
void gazebo::SetY(GzVector3 &v, const double val)
{
#if GAZEBO_MAJOR_VERSION >= 8
  v.Y(val);
#else 
  v.y = val;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
void gazebo::SetZ(GzVector3 &v, const double val)
{
#if GAZEBO_MAJOR_VERSION >= 8
  v.Z(val);
#else 
  v.z = val;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
double gazebo::GetX(const GzVector3 &v)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return v.X();
#else 
  return v.x;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
double gazebo::GetY(const GzVector3 &v)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return v.Y();
#else 
  return v.y;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
double gazebo::GetZ(const GzVector3 &v)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return v.Z();
#else 
  return v.z;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
void gazebo::SetX(GzQuaternion &q, const double val)
{
#if GAZEBO_MAJOR_VERSION >= 8
  q.X(val);
#else 
  q.x = val;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
void gazebo::SetY(GzQuaternion &q, const double val)
{
#if GAZEBO_MAJOR_VERSION >= 8
  q.Y(val);
#else 
  q.y = val;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
void gazebo::SetZ(GzQuaternion &q, const double val)
{
#if GAZEBO_MAJOR_VERSION >= 8
  q.Z(val);
#else 
  q.z = val;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
void gazebo::SetW(GzQuaternion &q, const double val)
{
#if GAZEBO_MAJOR_VERSION >= 8
  q.W(val);
#else 
  q.w = val;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
double gazebo::GetX(const GzQuaternion &q)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return q.X();
#else 
  return q.x;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
double gazebo::GetY(const GzQuaternion &q)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return q.Y();
#else 
  return q.y;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
double gazebo::GetZ(const GzQuaternion &q)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return q.Z();
#else 
  return q.z;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
double gazebo::GetW(const GzQuaternion &q)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return q.W();
#else 
  return q.w;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
GzVector3 gazebo::GetVector(const double x, const double y, const double z)
{
#if GAZEBO_MAJOR_VERSION >= 8
  GzVector3 v(x, y, z);
//  v.X(x);
//  v.Y(y);
//  v.Z(z);
#else 
  GzVector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
#endif 
  return v;
}

///////////////////////////////////////////////////////////////////////////////
GzVector3 gazebo::GetPos(const GzPose3 &pose)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return pose.Pos();
#else 
  return pose.pos;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
gazebo::GzQuaternion gazebo::GetRot(const GzPose3 &pose)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return pose.Rot();
#else 
  return pose.rot;
#endif 
}

///////////////////////////////////////////////////////////////////////////////
GzVector3 gazebo::GetPos(const GzMatrix4 &mat)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return mat.Translation();
#else 
  return mat.GetTranslation();
#endif 
}

///////////////////////////////////////////////////////////////////////////////
gazebo::GzQuaternion gazebo::GetRot(const GzMatrix4 &mat)
{
#if GAZEBO_MAJOR_VERSION >= 8
  return mat.Rotation();
#else 
  return mat.GetRotation();
#endif 
}

///////////////////////////////////////////////////////////////////////////////
gazebo::physics::PhysicsEnginePtr gazebo::GetPhysics(
  const gazebo::physics::WorldPtr &world)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world->Physics();
#else
    return world->GetPhysicsEngine();
#endif
}

///////////////////////////////////////////////////////////////////////////////
gazebo::physics::EntityPtr gazebo::GetEntityByName(
  const gazebo::physics::WorldPtr &world, const std::string &name)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world->EntityByName(name);
#else
    return world->GetEntity(name);
#endif

}

///////////////////////////////////////////////////////////////////////////////
gazebo::physics::ModelPtr gazebo::GetModelByName(
  const gazebo::physics::WorldPtr &world, const std::string &name)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world->ModelByName(name);
#else
    return world->GetModel(name);
#endif
}

///////////////////////////////////////////////////////////////////////////////
gazebo::physics::Model_V gazebo::GetModels(
  const gazebo::physics::WorldPtr &world)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world->Models();
#else
    return world->GetModels();
#endif
}

///////////////////////////////////////////////////////////////////////////////
gazebo::GzVector3 gazebo::GetBoundingBoxDimensions(const GzBox &box)
{
#if GAZEBO_MAJOR_VERSION >= 11
    auto const size = box.Size();
    GzVector3 bb(size.X(), size.Y(), size.Z());
#elif GAZEBO_MAJOR_VERSION >= 8
    GzVector3 bb(box.XLength(), box.YLength(), box.ZLength());
#else
    GzVector3 bb(box.GetXLength(), box.GetYLength(), box.GetZLength());
#endif
    return bb;
}
