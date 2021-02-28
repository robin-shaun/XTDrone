/**
Copyright (c) 2014, Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
    in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include <roboticsgroup_gazebo_plugins/disable_link_plugin.h>

namespace gazebo {

    DisableLinkPlugin::DisableLinkPlugin()
    {
        kill_sim = false;

        link_.reset();
    }

    DisableLinkPlugin::~DisableLinkPlugin()
    {
        kill_sim = true;
    }

    void DisableLinkPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        model_ = _parent;
        world_ = model_->GetWorld();

        // Check for link element
        if (!_sdf->HasElement("link")) {
            ROS_ERROR("No link element present. DisableLinkPlugin could not be loaded.");
            return;
        }

        link_name_ = _sdf->GetElement("link")->Get<std::string>();

        // Get pointers to joints
        link_ = model_->GetLink(link_name_);
        if (link_) {
            link_->SetEnabled(false);
            // Output some confirmation
            ROS_INFO_STREAM("DisableLinkPlugin loaded! Link: \"" << link_name_);
        }
        else
            ROS_ERROR_STREAM("Link" << link_name_ << " not found! DisableLinkPlugin could not be loaded.");
    }

    GZ_REGISTER_MODEL_PLUGIN(DisableLinkPlugin);
}
