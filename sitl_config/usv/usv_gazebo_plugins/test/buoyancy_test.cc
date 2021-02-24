#include <gtest/gtest.h>
#include "usv_gazebo_plugins/polyhedron_volume.hh"
#include "usv_gazebo_plugins/shape_volume.hh"

using namespace buoyancy;

/////////////////////////////////////////////////
TEST(PolyhedronTest, CubeTotalVolume)
{
  auto cube = Polyhedron::makeCube(2,2,2);
  auto volume = cube.ComputeFullVolume();
  EXPECT_FLOAT_EQ(volume.volume, 8.0);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), 0.0);
}

/////////////////////////////////////////////////
TEST(PolyhedronTest, CylinderTotalVolume)
{
  auto cylinder = Polyhedron::makeCylinder(0.5,2,100);
  auto volume = cylinder.ComputeFullVolume();
  EXPECT_NEAR(volume.volume, 1.57, 0.01);
  EXPECT_NEAR(volume.centroid.X(), 0.0, 1e-10);
  EXPECT_NEAR(volume.centroid.Y(), 0.0, 1e-10);
  EXPECT_NEAR(volume.centroid.Z(), 0.0, 1e-10);
}

///////////////////////////////////////////////////
TEST(PolyhedronTest, CubeNotSubmerged)
{
  auto cube = Polyhedron::makeCube(1,1,1);
  // water surface at z = 0
  buoyancy::Plane waterSurface;

  auto volume = cube.SubmergedVolume(ignition::math::Vector3d{0,0,2},
      ignition::math::Quaterniond{1,0,0,0},
      waterSurface);

  EXPECT_FLOAT_EQ(volume.volume, 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), 0.0);
}

///////////////////////////////////////////////////
TEST(PolyhedronTest, CylinderNotSubmerged)
{
  auto cylinder = Polyhedron::makeCylinder(0.5,2,10);
  // water surface at z = 0
  buoyancy::Plane waterSurface;

  auto volume = cylinder.SubmergedVolume(ignition::math::Vector3d{0,0,2},
      ignition::math::Quaterniond{1,0,0,0},
      waterSurface);

  EXPECT_FLOAT_EQ(volume.volume, 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), 0.0);
}

///////////////////////////////////////////////////
TEST(PolyhedronTest, CubeSubmerged)
{
  auto cube = Polyhedron::makeCube(1,1,1);
  // water surface at z = 0
  buoyancy::Plane waterSurface;

  // half of the cube is submerged
  //        -----
  // -------|   |--------
  //        -----
  auto volume = cube.SubmergedVolume(ignition::math::Vector3d{0,0,0},
      ignition::math::Quaterniond{1,0,0,0},
      waterSurface);
  EXPECT_FLOAT_EQ(volume.volume, 0.5);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), -0.25);

  // cube is fully submerged
  // -------------------
  //        -----
  //        |   |
  //        -----
  volume = cube.SubmergedVolume(ignition::math::Vector3d{0,0,-2},
      ignition::math::Quaterniond{1,0,0,0},
      waterSurface);
  EXPECT_FLOAT_EQ(volume.volume, 1.0);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), -2.0);

  // cube is slightly submerged at a 45 degree angle in roll
  //       /\
  //      /  \
  // -----\  /---------
  //       \/
  volume = cube.SubmergedVolume(ignition::math::Vector3d{0,0,0.25},
      ignition::math::Quaterniond{0.9238795, 0.3826834, 0, 0},
      waterSurface);
  EXPECT_NEAR(volume.volume, 0.21, 0.01);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_NEAR(volume.centroid.Z(), -0.15, 0.01);
}

///////////////////////////////////////////////////
TEST(PolyhedronTest, CylinderSubmerged)
{
  auto cylinder = Polyhedron::makeCylinder(0.5, 2, 100);
  // water surface at z = 0
  buoyancy::Plane waterSurface;

  // half of the cylinder is submerged
  //        ---
  // -------| |--------
  //        ---
  auto volume = cylinder.SubmergedVolume(ignition::math::Vector3d{0,0,0.0},
      ignition::math::Quaterniond{1,0,0,0},
      waterSurface);
  EXPECT_NEAR(volume.volume, 0.785, 0.001);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), -0.5);

  // cylinder is fully submerged
  // -------------------
  //        ---
  //        | |
  //        ---
  volume = cylinder.SubmergedVolume(ignition::math::Vector3d{0,0,-4},
      ignition::math::Quaterniond{1,0,0,0},
      waterSurface);
  EXPECT_NEAR(volume.volume, 1.57, 0.01);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Z(), -4.0);

  // cube is half submerged at a 45 degree angle in roll
  //     --------
  // ----|      |------
  //     --------
  volume = cylinder.SubmergedVolume(ignition::math::Vector3d{0,0,0},
      ignition::math::Quaterniond{0.707,0.707,0,0},
      waterSurface);
  EXPECT_NEAR(volume.volume, 0.785, 0.001);
  EXPECT_FLOAT_EQ(volume.centroid.X(), 0.0);
  EXPECT_FLOAT_EQ(volume.centroid.Y(), 0.0);
  EXPECT_NEAR(volume.centroid.Z(), -0.21, 0.01);
}

/////////////////////////////////////////////////
sdf::ElementPtr generateGeometryElem(const std::string& str)
{
  std::ostringstream modelStr;
  modelStr  << "<sdf version='" << SDF_VERSION << "'>"
            << "<model name ='model'>"
            << "<link name ='link'>"
            << "  <visual name ='vis'>"
            << "    <geometry>"
            <<        str
            << "    </geometry>"
            << "  </visual>"
            << "</link>"
            << "</model>"
            << "</sdf>";
  sdf::SDF sdf;
  sdf.SetFromString(modelStr.str());
  return sdf.Root()->GetElement("model")->GetElement("link")->GetElement("visual")->GetElement("geometry");
}

/////////////////////////////////////////////////
TEST(ShapeVolumeTest, CubeNoRotation)
{
  sdf::ElementPtr boxElem = generateGeometryElem("<box><size>2 2 2</size></box>");
  ShapeVolumePtr box = std::move(ShapeVolume::makeShape(boxElem));
  const double fluidLevel = 0;
  const std::vector<std::vector<double>> poses = {{0,0,1.0},
                                                  {0,0,0.5},
                                                  {0,0,0.0},
                                                  {0,0,-0.5},
                                                  {0,0,-1.},
                                                  {0,0,-1.5}};
  const std::vector<std::vector<double>> expectedResult = {{0,0,0,0},
                                                           {2,0,0,-0.25},
                                                           {4,0,0,-0.5},
                                                           {6,0,0,-0.75},
                                                           {8,0,0,-1.0},
                                                           {8,0,0,-1.5}};
  ignition::math::Pose3d pose;
  for(size_t i=0; i<poses.size(); i++)
  {
    pose.Pos().X() = poses[i][0];
    pose.Pos().Y() = poses[i][1];
    pose.Pos().Z() = poses[i][2];
    auto vol = box->CalculateVolume(pose, fluidLevel);
    EXPECT_FLOAT_EQ(vol.volume, expectedResult[i][0]);
    EXPECT_FLOAT_EQ(vol.centroid.X(), expectedResult[i][1]);
    EXPECT_FLOAT_EQ(vol.centroid.Y(), expectedResult[i][2]);
    EXPECT_FLOAT_EQ(vol.centroid.Z(), expectedResult[i][3]);
  }
}

/////////////////////////////////////////////////
TEST(ShapeVolumeTest, CubeRotation)
{
  sdf::ElementPtr boxElem = generateGeometryElem("<box><size>2 2 2</size></box>");
  ShapeVolumePtr box = std::move(ShapeVolume::makeShape(boxElem));
  const double fluidLevel = 0;
  const std::vector<std::vector<double>> poses = {{0,0,1.5},
                                                  {0,0,1.0},
                                                  {0,0,0.5},
                                                  {0,0,0.0},
                                                  {0,0,-0.5},
                                                  {0,0,-1.},
                                                  {0,0,-1.5}};
  const std::vector<std::vector<double>> expectedResult = {{0,0,0,0},
                                                           {0.343,0,0,-0.138},
                                                           {1.672,0,0,-0.305},
                                                           {4.000,0,0,-0.471},
                                                           {6.328,0,0,-0.713},
                                                           {7.657,0,0,-1.051},
                                                           {8.000,0,0,-1.500}};
  ignition::math::Pose3d pose;
  pose.Rot() = {0.9238795, 0.3826834, 0, 0};
  for(size_t i=0; i<poses.size(); i++)
  {
    pose.Pos().X() = poses[i][0];
    pose.Pos().Y() = poses[i][1];
    pose.Pos().Z() = poses[i][2];
    auto vol = box->CalculateVolume(pose, fluidLevel);
    EXPECT_NEAR(vol.volume, expectedResult[i][0], 0.001);
    EXPECT_FLOAT_EQ(vol.centroid.X(), expectedResult[i][1]);
    EXPECT_FLOAT_EQ(vol.centroid.Y(), expectedResult[i][2]);
    EXPECT_NEAR(vol.centroid.Z(), expectedResult[i][3], 0.001);
  }
}

/////////////////////////////////////////////////
TEST(ShapeVolumeTest, CylinderNoRotation)
{
  sdf::ElementPtr cylinderElem = generateGeometryElem("<cylinder><radius>0.5</radius><length>2</length></cylinder>");
  ShapeVolumePtr cylinder = std::move(ShapeVolume::makeShape(cylinderElem));
  const double fluidLevel = 0;
  const std::vector<std::vector<double>> poses = {{0,0,1.0},
                                                  {0,0,0.5},
                                                  {0,0,0.0},
                                                  {0,0,-0.5},
                                                  {0,0,-1.},
                                                  {0,0,-1.5}};
  // note since we are approximating final volume is 1.55 instead of 1.57
  // (increase segments for precision)
  const std::vector<std::vector<double>> expectedResult = {{0,0,0,0},
                                                           {0.39,0,0,-0.25},
                                                           {0.77,0,0,-0.5},
                                                           {1.16,0,0,-0.75},
                                                           {1.55,0,0,-1.0},
                                                           {1.55,0,0,-1.5}};
  ignition::math::Pose3d pose;
  for(size_t i=0; i<poses.size(); i++)
  {
    pose.Pos().X() = poses[i][0];
    pose.Pos().Y() = poses[i][1];
    pose.Pos().Z() = poses[i][2];
    auto vol = cylinder->CalculateVolume(pose, fluidLevel);
    EXPECT_NEAR(vol.volume, expectedResult[i][0], 0.01);
    EXPECT_FLOAT_EQ(vol.centroid.X(), expectedResult[i][1]);
    EXPECT_FLOAT_EQ(vol.centroid.Y(), expectedResult[i][2]);
    EXPECT_FLOAT_EQ(vol.centroid.Z(), expectedResult[i][3]);
  }
}

/////////////////////////////////////////////////
TEST(ShapeVolumeTest, CylinderRotation)
{
  sdf::ElementPtr cylinderElem = generateGeometryElem("<cylinder><radius>0.5</radius><length>2</length></cylinder>");
  ShapeVolumePtr cylinder = std::move(ShapeVolume::makeShape(cylinderElem));
  const double fluidLevel = 0;
  const std::vector<std::vector<double>> poses = {{0,0,0.5},
                                                  {0,0,0.0},
                                                  {0,0,-0.5}};
  // note since we are approximating final volume is 1.55 instead of 1.57
  // (increase segments for precision)
  const std::vector<std::vector<double>> expectedResult = {{0,0,0,0},
                                                           {0.77,0,0,-0.21},
                                                           {1.55,0,0,-0.5},
                                                           {1.55,0,0,-1.0}};
  ignition::math::Pose3d pose;
  pose.Rot() = {0.707, 0.707, 0, 0};
  for(size_t i=0; i<poses.size(); i++)
  {
    pose.Pos().X() = poses[i][0];
    pose.Pos().Y() = poses[i][1];
    pose.Pos().Z() = poses[i][2];
    auto vol = cylinder->CalculateVolume(pose, fluidLevel);
    EXPECT_NEAR(vol.volume, expectedResult[i][0], 0.01);
    EXPECT_FLOAT_EQ(vol.centroid.X(), expectedResult[i][1]);
    EXPECT_FLOAT_EQ(vol.centroid.Y(), expectedResult[i][2]);
    EXPECT_NEAR(vol.centroid.Z(), expectedResult[i][3], 0.01);
  }
}

/////////////////////////////////////////////////
TEST(ShapeVolumeTest, Sphere) {
  sdf::ElementPtr sphereElem = generateGeometryElem("<sphere><radius>0.5</radius></sphere>");
  ShapeVolumePtr sphere = std::move(ShapeVolume::makeShape(sphereElem));
  const double fluidLevel = 0;
  const std::vector<std::vector<double>> poses = {{0, 0, 0.5},
                                                  {0, 0, 0.25},
                                                  {0, 0, 0.0},
                                                  {0, 0, -0.25},
                                                  {0, 0, -0.5},
                                                  {0, 0, -1.0}};

  const std::vector<std::vector<double>> expectedResult = {{0,     0, 0, 0},
                                                           {0.082, 0, 0, -0.088},
                                                           {0.262, 0, 0, -0.188},
                                                           {0.442, 0, 0, -0.313},
                                                           {0.523, 0, 0, -0.500},
                                                           {0.523, 0, 0, -1.000}};

  ignition::math::Pose3d pose;
  pose.Rot() = {0.9238795, 0.3826834, 0, 0}; // rotation has no impact
  for(size_t i=0; i<poses.size(); i++)
  {
    pose.Pos().X() = poses[i][0];
    pose.Pos().Y() = poses[i][1];
    pose.Pos().Z() = poses[i][2];
    auto vol = sphere->CalculateVolume(pose, fluidLevel);
    EXPECT_NEAR(vol.volume, expectedResult[i][0], 0.001);
    EXPECT_FLOAT_EQ(vol.centroid.X(), expectedResult[i][1]);
    EXPECT_FLOAT_EQ(vol.centroid.Y(), expectedResult[i][2]);
    EXPECT_NEAR(vol.centroid.Z(), expectedResult[i][3], 0.001);
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}