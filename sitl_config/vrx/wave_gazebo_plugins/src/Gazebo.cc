/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <array>
#include <iostream>
#include <iterator>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>

#include "wave_gazebo_plugins/Gazebo.hh"

namespace gazebo
{
  namespace rendering
  {
    void SetMaterialShaderParam(
      Visual& _visual,
      const std::string &_paramName,
      const std::string &_shaderType,
      const std::string &_value)
    {
      // currently only vertex and fragment shaders are supported
      if (_shaderType != "vertex" && _shaderType != "fragment")
      {
        gzerr << "Shader type: '" << _shaderType << "' is not supported"
              << std::endl;
        return;
      }

      // set the parameter based name and type defined in material script
      // and shaders
      auto setNamedParam = [](Ogre::GpuProgramParametersSharedPtr _params,
          const std::string &_name, const std::string &_v)
      {
        auto paramDef = _params->_findNamedConstantDefinition(_name);
        if (!paramDef)
          return;

        switch (paramDef->constType)
        {
          case Ogre::GCT_INT1:
          {
            int value = Ogre::StringConverter::parseInt(_v);
            _params->setNamedConstant(_name, value);
            break;
          }
          case Ogre::GCT_FLOAT1:
          {
            Ogre::Real value = Ogre::StringConverter::parseReal(_v);
            _params->setNamedConstant(_name, value);
            break;
          }
    #if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
          case Ogre::GCT_INT2:
          case Ogre::GCT_FLOAT2:
          {
            Ogre::Vector2 value = Ogre::StringConverter::parseVector2(_v);
            _params->setNamedConstant(_name, value);
            break;
          }
    #endif
          case Ogre::GCT_INT3:
          case Ogre::GCT_FLOAT3:
          {
            Ogre::Vector3 value = Ogre::StringConverter::parseVector3(_v);
            _params->setNamedConstant(_name, value);
            break;
          }
          case Ogre::GCT_INT4:
          case Ogre::GCT_FLOAT4:
          {
            Ogre::Vector4 value = Ogre::StringConverter::parseVector4(_v);
            _params->setNamedConstant(_name, value);
            break;
          }
          case Ogre::GCT_MATRIX_4X4:
          {
            Ogre::Matrix4 value = Ogre::StringConverter::parseMatrix4(_v);
            _params->setNamedConstant(_name, value);
            break;
          }
          default:
            break;
        }
      };

      // loop through material techniques and passes to find the param
      Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName(
          _visual.GetMaterialName());
      if (mat.isNull())
      {
        gzerr << "Failed to find material: '" << _visual.GetMaterialName()
              << std::endl;
        return;
      }
      for (unsigned int i = 0; i < mat->getNumTechniques(); ++i)
      {
        Ogre::Technique *technique = mat->getTechnique(i);
        if (!technique)
          continue;
        for (unsigned int j = 0; j < technique->getNumPasses(); ++j)
        {
          Ogre::Pass *pass = technique->getPass(j);
          if (!pass)
            continue;

          // check if pass is programmable, ie if they are using shaders
          if (!pass->isProgrammable())
            continue;

          if (_shaderType == "vertex" && pass->hasVertexProgram())
          {
            setNamedParam(pass->getVertexProgramParameters(), \
                          _paramName, _value);
          }
          else if (_shaderType == "fragment" && pass->hasFragmentProgram())
          {
            setNamedParam(pass->getFragmentProgramParameters(), \
                          _paramName, _value);
          }
          else
          {
            gzerr << "Failed to retrieve shaders for material: '"
                  << _visual.GetMaterialName() << "', technique: '"
                  << technique->getName() << "', pass: '"
                  << pass->getName() << "'"
                  << std::endl;
            continue;
          }
        }
      }
    }
  }
}
