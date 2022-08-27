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
#ifndef GAZEBO_LENSFLAREVISUALPLUGIN_HH_
#define GAZEBO_LENSFLAREVISUALPLUGIN_HH_

#include <memory>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  // Forward declare private data class.
  class LensFlareVisualPluginPrivate;

  /// \brief Plugin that adds lens flare effect to a camera
  class LensFlareVisualPlugin : public VisualPlugin
  {
    /// \brief Constructor.
    public: LensFlareVisualPlugin();

    /// \brief Destructor.
    public: ~LensFlareVisualPlugin();

    // Documentation inherited
    public: virtual void Load(rendering::VisualPtr  _visual,
        sdf::ElementPtr _sdf);

    /// \brief Add lens flare effect to a camera
    private: void AddLensFlare(rendering::CameraPtr _camera);

    /// \internal
    /// \brief Private data pointer
    private: std::unique_ptr<LensFlareVisualPluginPrivate> dataPtr;
  };
}
#endif
