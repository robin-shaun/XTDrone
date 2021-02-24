/*
 * Copyright (C) 2019  Rhys Mainwaring
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

#include <memory>
#include <thread>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/UserCamera.hh"

#include "wave_gazebo_plugins/WavefieldVisualPlugin.hh"
#include "wave_gazebo_plugins/Gazebo.hh"
#include "wave_gazebo_plugins/Wavefield.hh"
#include "wave_gazebo_plugins/Utilities.hh"

using namespace gazebo;

namespace asv
{
  GZ_REGISTER_VISUAL_PLUGIN(WavefieldVisualPlugin)

///////////////////////////////////////////////////////////////////////////////
// Utilties

  /// \brief Convert a vector containing two doubles to an Ogre Vector2.
  ///
  /// \param[in] _v       A std::vector containing two entries.
  /// \param[out] _vout   The Ogre vector to be populated.
  void ToOgreVector2(const std::vector<double>& _v, Ogre::Vector2& _vout)
  {
    _vout = Ogre::Vector2::ZERO;
    if (_v.size() > 2)
    {
      gzerr << "Vector must have size 2 or less" << std::endl;
      return;
    }
    for (size_t i = 0; i < _v.size(); ++i)
    {
      _vout[i] = _v[i];
    }
  }

  /// \brief Convert a vector containing three doubles to an Ogre Vector3.
  ///
  /// \param[in] _v       A std::vector containing three entries.
  /// \param[out] _vout   The Ogre vector to be populated.
  void ToOgreVector3(const std::vector<double>& _v, Ogre::Vector3& _vout)
  {
    _vout = Ogre::Vector3::ZERO;
    if (_v.size() > 3)
    {
      gzerr << "Vector must have size 3 or less" << std::endl;
      return;
    }
    for (size_t i = 0; i < _v.size(); ++i)
    {
      _vout[i] = _v[i];
    }
  }

  /// \brief Convert an ignition Vector2 to an Ogre Vector2.
  ///
  /// \param[in] _v       An ignition vector.
  /// \param[out] _vout   The Ogre vector to be populated.
  void ToOgreVector2(const ignition::math::Vector2d& _v, Ogre::Vector2& _vout)
  {
    _vout.x = _v.X();
    _vout.y = _v.Y();
  }

  /// \brief Convert an ignition Vector3 to an Ogre Vector3.
  ///
  /// \param[in] _v       An ignition vector.
  /// \param[out] _vout   The Ogre vector to be populated.
  void ToOgreVector3(const ignition::math::Vector3d& _v, Ogre::Vector3& _vout)
  {
    _vout.x = _v.X();
    _vout.y = _v.Y();
    _vout.z = _v.Z();
  }

  void ToOgreVector2(
    const std::vector<ignition::math::Vector2d>& _v,
    Ogre::Vector2& _vout0,
    Ogre::Vector2& _vout1,
    Ogre::Vector2& _vout2
  )
  {
    _vout0 = Ogre::Vector2::ZERO;
    _vout1 = Ogre::Vector2::ZERO;
    _vout2 = Ogre::Vector2::ZERO;

    if (_v.size() > 3)
    {
      gzerr << "Vector must have size 3 or less" << std::endl;
      return;
    }
    if (_v.size() > 0)
      ToOgreVector2(_v[0], _vout0);
    if (_v.size() > 1)
      ToOgreVector2(_v[1], _vout1);
    if (_v.size() > 2)
      ToOgreVector2(_v[2], _vout2);
  }

  void ToOgreVector3(
    const std::vector<ignition::math::Vector3d>& _v,
    Ogre::Vector3& _vout0,
    Ogre::Vector3& _vout1,
    Ogre::Vector3& _vout2
  )
  {
    _vout0 = Ogre::Vector3::ZERO;
    _vout1 = Ogre::Vector3::ZERO;
    _vout2 = Ogre::Vector3::ZERO;

    if (_v.size() > 3)
    {
      gzerr << "Vector must have size 3 or less" << std::endl;
      return;
    }
    if (_v.size() > 0)
      ToOgreVector3(_v[0], _vout0);
    if (_v.size() > 1)
      ToOgreVector3(_v[1], _vout1);
    if (_v.size() > 2)
      ToOgreVector3(_v[2], _vout2);
  }

///////////////////////////////////////////////////////////////////////////////
// WavefieldVisualPluginPrivate

  /// \internal
  /// \brief Private data for the WavefieldVisualPlugin
  class WavefieldVisualPluginPrivate
  {
    public: WavefieldVisualPluginPrivate() :
            planeUp("planeUp"),
            planeDown("planeDown")
            {}

    /// \brief The visual containing this plugin.
    public: rendering::VisualPtr visual;

    /// \brief The visual's name
    public: std::string visualName;

    /// \brief The wavefield visual plugin SDF.
    public: sdf::ElementPtr sdf;

    /// \brief The wavefield parameters.
    public: std::shared_ptr<WaveParameters> waveParams;

    /// \brief Do not update visual if 'true', [false].
    public: bool isStatic = false;

    /// \brief Enable rtts for reflection refraction, [true].
    public: bool enableRtt = true;

    /// \brief Ratio between shallow water color and refraction color to use
    ///        In [0, 1], where 0 is no refraction and 1 is maximum refraction
    public: double refractOpacity = 0;

    /// \brief Ratio between environment color and reflection color to use
    ///        In [0, 1], where 0 is no reflection and 1 is maximum reflection
    public: double reflectOpacity = 0;

    /// \brief Noise scale in rtt. Create distortion in reflection/refraction
    public: double rttNoise = 0;

    /// \brief World stats.
    public: double simTime = 0;

    /// \brief Prevent multiple calls to Init loading visuals twice...
    public: bool isInitialised = false;

    // OGRE objects for reflection/refraction
    public: gazebo::rendering::ScenePtr scene;
    public: Ogre::Entity* oceanEntity = nullptr;
    public: Ogre::MovablePlane planeUp;
    public: Ogre::MovablePlane planeDown;
    public: Ogre::TextureUnitState *reflectTex = nullptr;
    public: Ogre::TextureUnitState *refractTex = nullptr;

    // Vectors of OGRE objects
    public: std::vector<Ogre::Camera*> cameras;
    public: std::vector<Ogre::RenderTarget*> reflectionRts;
    public: std::vector<Ogre::RenderTarget*> refractionRts;

    /// \brief Event based connections.
    public: event::ConnectionPtr preRenderConnection;
    public: event::ConnectionPtr cameraPreRenderConnection;
  };

///////////////////////////////////////////////////////////////////////////////
// WavefieldVisualPlugin

  WavefieldVisualPlugin::~WavefieldVisualPlugin()
  {
    // Clean up.
    this->data->waveParams.reset();

    // Reset connections and transport.
    this->data->preRenderConnection.reset();
    this->data->cameraPreRenderConnection.reset();
  }

  WavefieldVisualPlugin::WavefieldVisualPlugin() :
    VisualPlugin(),
    RenderTargetListener(),
    data(new WavefieldVisualPluginPrivate)
  {
    this->data->isInitialised = false;
  }

  void WavefieldVisualPlugin::Load(
    rendering::VisualPtr _visual,
    sdf::ElementPtr _sdf)
  {
    // @DEBUG_INFO
    // std::thread::id threadId = std::this_thread::get_id();
    // gzmsg << "Load WavefieldVisualPlugin [thread: "
    //       << threadId << "]" << std::endl;

    // Capture visual and plugin SDF
    GZ_ASSERT(_visual != nullptr, "Visual must not be null");
    GZ_ASSERT(_sdf != nullptr, "SDF Element must not be null");

    // Capture the visual and sdf.
    this->data->visual = _visual;
    this->data->sdf = _sdf;

    // Process SDF Parameters
    #if GAZEBO_MAJOR_VERSION >= 8
      this->data->visualName = _visual->Name();
    #else
      this->data->visualName = _visual->GetName();
    #endif

    gzmsg << "WavefieldVisualPlugin <" << this->data->visualName
          << ">: Loading WaveParamaters from SDF" <<  std::endl;

    this->data->isStatic = Utilities::SdfParamBool(*_sdf, "static", false);

#if (GAZEBO_MAJOR_VERSION == 7 && GAZEBO_MINOR_VERSION >= 16) || \
    (GAZEBO_MAJOR_VERSION == 9 && GAZEBO_MINOR_VERSION >= 11)
    // Check if reflection/refracion rtts enabled
    // Only available in Gazebo Version >=7.16.0 || >=9.11.0
    this->data->enableRtt = Utilities::SdfParamBool(*_sdf, "enableRtt", true);
#else
    this->data->enableRtt = false;
#endif

    // Read refraction and reflection ratios and noise
    this->data->refractOpacity =
      Utilities::SdfParamDouble(*_sdf, "refractOpacity", 0.2);
    this->data->reflectOpacity =
      Utilities::SdfParamDouble(*_sdf, "reflectOpacity", 0.2);
    this->data->rttNoise =
      Utilities::SdfParamDouble(*_sdf, "rttNoise", 0.1);

    this->data->waveParams.reset(new WaveParameters());
    if (_sdf->HasElement("wave"))
    {
      gzmsg << "Found <wave> tag" << std::endl;
      sdf::ElementPtr sdfWave = _sdf->GetElement("wave");
      this->data->waveParams->SetFromSDF(*sdfWave);
    }
    else
    {
      gzerr << "Missing <wave> tag" << std::endl;
    }

    // @DEBUG_INFO
    this->data->waveParams->DebugPrint();

    // Setup oceanEntity
    Ogre::SceneNode *ogreNode = this->data->visual->GetSceneNode();
    this->data->oceanEntity =
        dynamic_cast<Ogre::Entity *>(ogreNode->getAttachedObject(0));
    if (!this->data->oceanEntity)
    {
      gzerr << "No ocean entity found" << std::endl;
      return;
    }

    // Render water later for proper rendering of propeller
    this->data->oceanEntity->setRenderQueueGroup(this->data->oceanEntity->
                                                 getRenderQueueGroup()+1);

    // Setup reflection refraction
    if (this->data->enableRtt)
      this->SetupReflectionRefraction();

    // Bind the update method to ConnectPreRender events
    this->data->preRenderConnection = event::Events::ConnectPreRender(
        std::bind(&WavefieldVisualPlugin::OnPreRender, this));
  }

  void WavefieldVisualPlugin::Init()
  {
    // @DEBUG_INFO
    // std::thread::id threadId = std::this_thread::get_id();
    // gzmsg << "Init WavefieldVisualPlugin [thread: "
    //       << threadId << "]" << std::endl;

    if (!this->data->isInitialised)
    {
      // Initialise vertex shader
      std::string shaderType = "vertex";
#if 0
      this->data->visual->SetMaterialShaderParam(
        "time", shaderType, std::to_string(0.0));
#else
      rendering::SetMaterialShaderParam(*this->data->visual,
        "time", shaderType, std::to_string(0.0));
#endif
      this->SetShaderParams();

      this->data->isInitialised = true;
    }
  }

  void WavefieldVisualPlugin::Reset()
  {
    // @DEBUG_INFO
    // gzmsg << "Reset WavefieldVisualPlugin" << std::endl;
  }

  void WavefieldVisualPlugin::OnPreRender()
  {
    if (this->data->enableRtt)
    {
      // Update reflection/refraction clip plane pose (in case the ocean moves)
      this->UpdateClipPlanes();

      // Continuously look for new cameras for reflection/refraction setup
      this->AddNewCamerasForReflectionRefraction();
    }

    // Create moving ocean waves
    if (!this->data->isStatic)
    {
#if 0
      this->data->visual->SetMaterialShaderParam(
        "time", shaderType, std::to_string(simTime));
#else
      auto simTime = this->data->visual->GetScene()->SimTime();
      rendering::SetMaterialShaderParam(*this->data->visual,
        "time", "vertex",
        std::to_string(static_cast<float>(simTime.Double())));
#endif
    }
  }

///////////////////////////////////////////////////////////////////////////////
// Reflection/Refraction setup and texture creation

  void WavefieldVisualPlugin::SetupReflectionRefraction()
  {
    // OGRE setup
    this->data->scene = this->data->visual->GetScene();

    // Create clipping planes to hide objects for making rtts, in default pose
    this->data->planeUp = Ogre::MovablePlane(Ogre::Vector3::UNIT_Z,
                                             Ogre::Vector3::ZERO);
    this->data->planeDown = Ogre::MovablePlane(-Ogre::Vector3::UNIT_Z,
                                               Ogre::Vector3::ZERO);

    // Get texture unit states to update with rtts
    Ogre::MaterialPtr material =
      Ogre::MaterialManager::getSingleton().getByName(this->data->visual->
                                                      GetMaterialName());
    this->data->reflectTex = (material->getTechnique(0)->getPass(0)->
                              getTextureUnitState(2));

    this->data->refractTex = (material->getTechnique(0)->getPass(0)->
                              getTextureUnitState(3));

    // Set reflection/refraction parameters
    rendering::SetMaterialShaderParam(*this->data->visual,
      "refractOpacity", "fragment",
      std::to_string(static_cast<float>(this->data->refractOpacity)));
    rendering::SetMaterialShaderParam(*this->data->visual,
      "reflectOpacity", "fragment",
      std::to_string(static_cast<float>(this->data->reflectOpacity)));
    rendering::SetMaterialShaderParam(*this->data->visual,
      "rttNoise", "fragment",
      std::to_string(static_cast<float>(this->data->rttNoise)));

    // Temp fix for camera sensors rendering upsidedown, only needed on server
    if (this->data->scene->UserCameraCount() > 0)
    {
      rendering::SetMaterialShaderParam(*this->data->visual,
        "flipAcrossY", "fragment",
        std::to_string(0));
    }
    else
    {
      rendering::SetMaterialShaderParam(*this->data->visual,
        "flipAcrossY", "fragment",
        std::to_string(1));
    }

#if (GAZEBO_MAJOR_VERSION == 7 && GAZEBO_MINOR_VERSION >= 16) || \
    (GAZEBO_MAJOR_VERSION == 9 && GAZEBO_MINOR_VERSION >= 11)
    // Bind the update method to ConnectCameraPreRender events
    // Only in Gazebo Version >=7.19.0 || >=9.11.0
    this->data->cameraPreRenderConnection =
      rendering::Events::ConnectCameraPreRender(
        std::bind(&WavefieldVisualPlugin::OnCameraPreRender,
                  this, std::placeholders::_1));
#endif
  }

  void WavefieldVisualPlugin::UpdateClipPlanes()
  {
    #if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Pose3d pose = this->data->visual->WorldPose();
      Ogre::Vector3 oceanPosition(pose.Pos().X(),
                                  pose.Pos().Y(),
                                  pose.Pos().Z());

      Ogre::Quaternion oceanRotation(pose.Rot().W(),
                                     pose.Rot().X(),
                                     pose.Rot().Y(),
                                     pose.Rot().Z());
    #else
      math::Pose pose = this->data->visual->GetWorldPose();
      Ogre::Vector3 oceanPosition(pose.pos.x,
                                  pose.pos.y,
                                  pose.pos.z);

      Ogre::Quaternion oceanRotation(pose.rot.w,
                                     pose.rot.x,
                                     pose.rot.y,
                                     pose.rot.z);
    #endif
    Ogre::Vector3 oceanNormal = oceanRotation * Ogre::Vector3::UNIT_Z;
    this->data->planeUp.redefine(oceanNormal, oceanPosition);
    this->data->planeDown.redefine(-oceanNormal, oceanPosition);
  }

  void WavefieldVisualPlugin::AddNewCamerasForReflectionRefraction()
  {
    // User cam setup in gzclient
    if (this->data->scene->UserCameraCount() > 0)
    {
      // Get user cam
      rendering::UserCameraPtr userCamera = this->data->scene->GetUserCamera(0);

      // If user cam not already in cameras, create its rtts
      if (std::find(this->data->cameras.begin(), this->data->cameras.end(),
                    userCamera->OgreCamera()) == this->data->cameras.end())
      {
        this->CreateRtts(userCamera->OgreCamera());
      }
    }

    // Camera sensor setup in gzserver
    else
    {
      // Get new cameras, create their rtts
      std::vector<rendering::CameraPtr> newCameras = this->NewCameras();
      for (rendering::CameraPtr c : newCameras)
      {
        this->CreateRtts(c->OgreCamera());
      }
    }
  }

  void WavefieldVisualPlugin::CreateRtts(Ogre::Camera* _camera)
  {
    // Preserve the camera aspect ratio in the texture.
    const double kScale = 0.25;
    const int kWidth    = _camera->getViewport()->getActualWidth() * kScale;
    const int kHeight   = _camera->getViewport()->getActualHeight() * kScale;

    // Create reflection texture
    Ogre::TexturePtr rttReflectionTexture =
      Ogre::TextureManager::getSingleton().createManual(
        this->data->visualName + "_" + _camera->getName() + "_reflection",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        kWidth, kHeight,
        0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);

    // Create refraction texture
    Ogre::TexturePtr rttRefractionTexture =
      Ogre::TextureManager::getSingleton().createManual(
        this->data->visualName + "_" + _camera->getName() + "_refraction",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        kWidth, kHeight,
        0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET);

    Ogre::ColourValue backgroundColor =
        rendering::Conversions::Convert(this->data->scene->BackgroundColor());

    // Setup reflection render target
    Ogre::RenderTarget* reflectionRt =
        rttReflectionTexture->getBuffer()->getRenderTarget();
    reflectionRt->setAutoUpdated(false);
    Ogre::Viewport *reflVp =
        reflectionRt->addViewport(_camera);
    reflVp->setClearEveryFrame(true);
    reflVp->setOverlaysEnabled(false);
    reflVp->setBackgroundColour(backgroundColor);
    reflVp->setVisibilityMask(GZ_VISIBILITY_ALL &
        ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
    reflectionRt->addListener(this);

    // Setup refraction render target
    Ogre::RenderTarget* refractionRt =
        rttRefractionTexture->getBuffer()->getRenderTarget();
    refractionRt->setAutoUpdated(false);
    Ogre::Viewport *refrVp =
        refractionRt->addViewport(_camera);
    refrVp->setClearEveryFrame(true);
    refrVp->setOverlaysEnabled(false);
    refrVp->setBackgroundColour(backgroundColor);
    refrVp->setVisibilityMask(GZ_VISIBILITY_ALL &
        ~(GZ_VISIBILITY_GUI | GZ_VISIBILITY_SELECTABLE));
    refractionRt->addListener(this);

    // Store camera and rtts
    this->data->cameras.push_back(_camera);
    this->data->reflectionRts.push_back(reflectionRt);
    this->data->refractionRts.push_back(refractionRt);

    // Add frame to texture units
    this->data->reflectTex->addFrameTextureName(rttReflectionTexture
                                                ->getName());
    this->data->refractTex->addFrameTextureName(rttRefractionTexture
                                                ->getName());
  }

  std::vector<rendering::CameraPtr> WavefieldVisualPlugin::NewCameras()
  {
    std::vector<rendering::CameraPtr> retVal;

    auto sensors = sensors::SensorManager::Instance()->GetSensors();
    for (auto sensor : sensors)
    {
      if (sensor->Type() == "camera")
      {
        rendering::CameraPtr c = this->data->scene->GetCamera(
            this->data->scene->StripSceneName(sensor->ScopedName()));

        if (c && std::find(this->data->cameras.begin(),
            this->data->cameras.end(),
            c->OgreCamera()) == this->data->cameras.end())
        {
          retVal.push_back(c);
        }
      }
    }
    return retVal;
  }

///////////////////////////////////////////////////////////////////////////////
// Shader Params Wave Generation

  void WavefieldVisualPlugin::SetShaderParams()
  {
    const std::string shaderType = "vertex";

    // Parameters (to load from SDF)
    Ogre::Vector3 amplitude = Ogre::Vector3::ZERO;
    Ogre::Vector3 wavenumber = Ogre::Vector3::ZERO;
    Ogre::Vector3 omega = Ogre::Vector3::ZERO;
    Ogre::Vector3 steepness = Ogre::Vector3::ZERO;
    Ogre::Vector2 dir0 = Ogre::Vector2::ZERO;
    Ogre::Vector2 dir1 = Ogre::Vector2::ZERO;
    Ogre::Vector2 dir2 = Ogre::Vector2::ZERO;

    ToOgreVector3(this->data->waveParams->Amplitude_V(), amplitude);
    ToOgreVector3(this->data->waveParams->Wavenumber_V(), wavenumber);
    ToOgreVector3(this->data->waveParams->AngularFrequency_V(), omega);
    ToOgreVector3(this->data->waveParams->Steepness_V(), steepness);
    ToOgreVector2(this->data->waveParams->Direction_V(), dir0, dir1, dir2);

    // These parameters are updated on initialisation
    auto& visual = *this->data->visual;
#if 0
    visual.SetMaterialShaderParam(
      "amplitude", shaderType, Ogre::StringConverter::toString(amplitude));
    visual.SetMaterialShaderParam(
      "wavenumber", shaderType, Ogre::StringConverter::toString(wavenumber));
    visual.SetMaterialShaderParam(
      "omega", shaderType, Ogre::StringConverter::toString(omega));
    visual.SetMaterialShaderParam(
      "steepness", shaderType, Ogre::StringConverter::toString(steepness));
    visual.SetMaterialShaderParam(
      "dir0", shaderType, Ogre::StringConverter::toString(dir0));
    visual.SetMaterialShaderParam(
      "dir1", shaderType, Ogre::StringConverter::toString(dir1));
    visual.SetMaterialShaderParam(
      "dir2", shaderType, Ogre::StringConverter::toString(dir2));
#else
    rendering::SetMaterialShaderParam(visual,
      "Nwaves", shaderType, std::to_string(this->data->waveParams->Number()));
    rendering::SetMaterialShaderParam(visual,
      "amplitude", shaderType, Ogre::StringConverter::toString(amplitude));
    rendering::SetMaterialShaderParam(visual,
      "wavenumber", shaderType, Ogre::StringConverter::toString(wavenumber));
    rendering::SetMaterialShaderParam(visual,
      "omega", shaderType, Ogre::StringConverter::toString(omega));
    rendering::SetMaterialShaderParam(visual,
      "steepness", shaderType, Ogre::StringConverter::toString(steepness));
    rendering::SetMaterialShaderParam(visual,
      "dir0", shaderType, Ogre::StringConverter::toString(dir0));
    rendering::SetMaterialShaderParam(visual,
      "dir1", shaderType, Ogre::StringConverter::toString(dir1));
    rendering::SetMaterialShaderParam(visual,
      "dir2", shaderType, Ogre::StringConverter::toString(dir2));
    float tau = this->data->waveParams->Tau();
    rendering::SetMaterialShaderParam(visual,
      "tau", shaderType, Ogre::StringConverter::toString(tau));
#endif
  }

///////////////////////////////////////////////////////////////////////////////
// OnCameraPreRender: How to update rtt before camera

  void WavefieldVisualPlugin::OnCameraPreRender(const std::string &_camera)
  {
    // Get appropriate camera source
    rendering::CameraPtr camSource;
    if (this->data->scene->UserCameraCount() > 0)
      camSource = this->data->scene->GetUserCamera(0);
    else
      camSource = this->data->scene->GetCamera(_camera);

    // Update rtts first before updating camera
    for (unsigned int i = 0; i < this->data->cameras.size(); ++i)
    {
      if (camSource->OgreCamera() == this->data->cameras.at(i))
      {
        this->data->reflectionRts.at(i)->update();
        this->data->refractionRts.at(i)->update();
        return;
      }
    }
  }

///////////////////////////////////////////////////////////////////////////////
// pre and post RenderTargetUpdate(): RenderTargetListener methods

  void WavefieldVisualPlugin::preRenderTargetUpdate(
      const Ogre::RenderTargetEvent& rte)
  {
    // Hide ocean for creating reflection/refraction textures
    if (this->data->oceanEntity)
    {
      this->data->oceanEntity->setVisible(false);
    }

    // Reflection: hide entities below, reflect, and set the right frame
    for (unsigned int i = 0; i < this->data->reflectionRts.size(); ++i)
    {
      Ogre::RenderTarget* rt = this->data->reflectionRts.at(i);
      if (rte.source == rt)
      {
        this->data->cameras.at(i)->enableReflection(this->data->planeUp);
        this->data->cameras.at(i)->enableCustomNearClipPlane(this->data->
                                                             planeUp);
        this->data->reflectTex->setCurrentFrame(i);
        return;
      }
    }

    // Refraction: hide entities above and set the right frame
    for (unsigned int i = 0; i < this->data->refractionRts.size(); ++i)
    {
      Ogre::RenderTarget* rt = this->data->refractionRts.at(i);
      if (rte.source == rt)
      {
        this->data->cameras.at(i)->enableCustomNearClipPlane(this->data->
                                                             planeDown);
        this->data->refractTex->setCurrentFrame(i);
        return;
      }
    }
  }

  void WavefieldVisualPlugin::postRenderTargetUpdate(
      const Ogre::RenderTargetEvent& rte)
  {
    // Show ocean after creating reflection/refraction textures
    if (this->data->oceanEntity)
    {
      this->data->oceanEntity->setVisible(true);
    }

    // Reflection: reshow all entities
    for (unsigned int i = 0; i < this->data->reflectionRts.size(); ++i)
    {
      Ogre::RenderTarget* rt = this->data->reflectionRts.at(i);
      if (rte.source == rt)
      {
        this->data->cameras.at(i)->disableReflection();
        this->data->cameras.at(i)->disableCustomNearClipPlane();
        return;
      }
    }

    // Refraction: reshow all entities
    for (unsigned int i = 0; i < this->data->refractionRts.size(); ++i)
    {
      Ogre::RenderTarget* rt = this->data->refractionRts.at(i);
      if (rte.source == rt)
      {
        this->data->cameras.at(i)->disableCustomNearClipPlane();
        return;
      }
    }
  }
}
