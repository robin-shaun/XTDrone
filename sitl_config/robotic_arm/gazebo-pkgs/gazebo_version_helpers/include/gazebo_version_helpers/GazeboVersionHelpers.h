#ifndef GAZEBO_VERSIONHELPERS_H
#define GAZEBO_VERSIONHELPERS_H

#include <gazebo/gazebo.hh>

namespace gazebo
{

// typedefs
#if GAZEBO_MAJOR_VERSION >= 11
namespace gz_math = ignition::math;
typedef gz_math::Pose3d GzPose3;
typedef gz_math::Vector3d GzVector3;
typedef gz_math::Quaterniond GzQuaternion;
typedef gz_math::Matrix4d GzMatrix4;
typedef gz_math::Matrix3d GzMatrix3;
typedef gz_math::AxisAlignedBox GzBox;
#elif GAZEBO_MAJOR_VERSION >= 8
namespace gz_math = ignition::math;
typedef gz_math::Pose3d GzPose3;
typedef gz_math::Vector3d GzVector3;
typedef gz_math::Quaterniond GzQuaternion;
typedef gz_math::Matrix4d GzMatrix4;
typedef gz_math::Matrix3d GzMatrix3;
typedef gz_math::Box GzBox;
#else
namespace gz_math = gazebo::math;
typedef gz_math::Pose GzPose3;
typedef gz_math::Vector3 GzVector3;
typedef gz_math::Quaternion GzQuaternion;
typedef gz_math::Matrix4 GzMatrix4;
typedef gz_math::Matrix3 GzMatrix3;
typedef gz_math::Box GzBox;
#endif


// Helper functions
// //////////////////////////

///////////////////////////////////////////////////////////////////////////////
GzPose3 GetWorldPose(const gazebo::physics::LinkPtr &link);

///////////////////////////////////////////////////////////////////////////////
GzVector3 GetWorldVelocity(const gazebo::physics::LinkPtr &link);

///////////////////////////////////////////////////////////////////////////////
GzMatrix4 GetIdentity();

///////////////////////////////////////////////////////////////////////////////
GzMatrix4 GetMatrix(const GzPose3 &pose);

///////////////////////////////////////////////////////////////////////////////
GzMatrix4 GetMatrix(const GzVector3 &pos);

///////////////////////////////////////////////////////////////////////////////
double GetLength(const GzVector3 &v);

///////////////////////////////////////////////////////////////////////////////
GzVector3 GetVector(const double x, const double y, const double z);

///////////////////////////////////////////////////////////////////////////////
void SetX(GzVector3 &v, const double val);
void SetY(GzVector3 &v, const double val);
void SetZ(GzVector3 &v, const double val);
double GetX(const GzVector3 &v);
double GetY(const GzVector3 &v);
double GetZ(const GzVector3 &v);


///////////////////////////////////////////////////////////////////////////////
void SetX(GzQuaternion &q, const double val);
void SetY(GzQuaternion &q, const double val);
void SetZ(GzQuaternion &q, const double val);
void SetW(GzQuaternion &q, const double val);
double GetX(const GzQuaternion &q);
double GetY(const GzQuaternion &q);
double GetZ(const GzQuaternion &q);
double GetW(const GzQuaternion &q);

///////////////////////////////////////////////////////////////////////////////
GzVector3 GetPos(const GzPose3 &pose);
///////////////////////////////////////////////////////////////////////////////
GzVector3 GetPos(const GzMatrix4 &mat);

///////////////////////////////////////////////////////////////////////////////
GzQuaternion GetRot(const GzPose3 &pose);
///////////////////////////////////////////////////////////////////////////////
GzQuaternion GetRot(const GzMatrix4 &mat);

///////////////////////////////////////////////////////////////////////////////
gazebo::physics::PhysicsEnginePtr GetPhysics(
  const gazebo::physics::WorldPtr &world);


///////////////////////////////////////////////////////////////////////////////
gazebo::physics::EntityPtr GetEntityByName(
  const gazebo::physics::WorldPtr &world, const std::string &name);

///////////////////////////////////////////////////////////////////////////////
gazebo::physics::ModelPtr GetModelByName(
  const gazebo::physics::WorldPtr &world, const std::string &name);

///////////////////////////////////////////////////////////////////////////////
gazebo::physics::Model_V GetModels(const gazebo::physics::WorldPtr &world);

///////////////////////////////////////////////////////////////////////////////
template<typename T>
GzVector3 GetSize3(const T& t)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return t.Size();
#else
    return t.GetSize();
#endif
}

///////////////////////////////////////////////////////////////////////////////
template<typename T>
std::string GetName(const T& t)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return t.Name();
#else
    return t.GetName();
#endif
}

///////////////////////////////////////////////////////////////////////////////
template<typename T>
GzBox GetBoundingBox(const T &t)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return t.BoundingBox();
#else
    return t.GetBoundingBox();
#endif
}

///////////////////////////////////////////////////////////////////////////////
GzVector3 GetBoundingBoxDimensions(const GzBox &box);

///////////////////////////////////////////////////////////////////////////////
template<typename T>
GzPose3 GetRelativePose(const T &t)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return t.RelativePose();
#else
    return t.GetRelativePose();
#endif
}



}  // namespace

#endif  //  GAZEBO_VERSIONHELPERS_H
