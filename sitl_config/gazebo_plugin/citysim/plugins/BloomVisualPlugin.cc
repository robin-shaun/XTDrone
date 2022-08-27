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
#include <gazebo/rendering/RenderingIface.hh>

#include "BloomVisualPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \class BloomVisualPlugin BloomVisualPlugin.hh
  /// \brief Private data for the BloomVisualPlugin class.
  class BloomVisualPluginPrivate
  {
  };
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(BloomVisualPlugin)

/////////////////////////////////////////////////
BloomVisualPlugin::BloomVisualPlugin()
    : dataPtr(new BloomVisualPluginPrivate)
{
}

/////////////////////////////////////////////////
BloomVisualPlugin::~BloomVisualPlugin()
{
}

/////////////////////////////////////////////////
void BloomVisualPlugin::Load(rendering::VisualPtr _visual,
    sdf::ElementPtr /*_sdf*/)
{
  if (!_visual)
    gzerr << "Invalid visual pointer." << std::endl;

  auto scene = _visual->GetScene();

  for (unsigned int i = 0; i < scene->CameraCount(); ++i)
  {
    this->AddBloom(scene->GetCamera(i));
  }
  for (unsigned int i = 0; i < scene->UserCameraCount(); ++i)
  {
    rendering::CameraPtr cam =
        boost::dynamic_pointer_cast<rendering::Camera>(
        scene->GetUserCamera(i));
    this->AddBloom(cam);
  }

  return;
}

/////////////////////////////////////////////////
void BloomVisualPlugin::AddBloom(rendering::CameraPtr _camera)
{
  if (!_camera)
    return;

  Ogre::CompositorInstance *bloomInstance;
  bloomInstance =
      Ogre::CompositorManager::getSingleton().addCompositor(
      _camera->OgreViewport(), "Bloom");

  bloomInstance->setEnabled(true);

}
