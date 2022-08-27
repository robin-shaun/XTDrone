/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/LensFlare.hh>
#include <gazebo/rendering/RenderingIface.hh>

#include "LensFlareVisualPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \class LensFlareVisualPlugin LensFlareVisualPlugin.hh
  /// \brief Private data for the LensFlareVisualPlugin class.
  class LensFlareVisualPluginPrivate
  {
    /// \brief Lens flare
    public: std::vector<rendering::LensFlarePtr> lensFlares;
  };
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(LensFlareVisualPlugin)

/////////////////////////////////////////////////
LensFlareVisualPlugin::LensFlareVisualPlugin()
    : dataPtr(new LensFlareVisualPluginPrivate)
{
}

/////////////////////////////////////////////////
LensFlareVisualPlugin::~LensFlareVisualPlugin()
{
}

/////////////////////////////////////////////////
void LensFlareVisualPlugin::Load(rendering::VisualPtr _visual,
    sdf::ElementPtr /*_sdf*/)
{
  if (!_visual)
    gzerr << "Invalid visual pointer." << std::endl;

  auto scene = _visual->GetScene();

  for (unsigned int i = 0; i < scene->CameraCount(); ++i)
  {
    this->AddLensFlare(scene->GetCamera(i));
  }
  for (unsigned int i = 0; i < scene->UserCameraCount(); ++i)
  {
    rendering::CameraPtr cam =
        boost::dynamic_pointer_cast<rendering::Camera>(
        scene->GetUserCamera(i));
    this->AddLensFlare(cam);
  }

  return;
}

/////////////////////////////////////////////////
void LensFlareVisualPlugin::AddLensFlare(rendering::CameraPtr _camera)
{
  if (!_camera)
    return;

  rendering::LensFlarePtr lensFlare;
  lensFlare.reset(new rendering::LensFlare);
  lensFlare->SetCamera(_camera);
  this->dataPtr->lensFlares.push_back(lensFlare);
}
