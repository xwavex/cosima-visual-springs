/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2018 by Dennis Leroy Wigand <dwigand at cor-lab dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <iostream>
#include <vector>
#include <cstdint>
#include <mutex>
#include <string>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/MovableText.hh>
#include <gazebo/rendering/LinkFrameVisual.hh>

using namespace gazebo;

namespace cosima
{

class VirtualConstraintVisual : public VisualPlugin
{
  public:
    ~VirtualConstraintVisual()
    {
        event::Events::DisconnectPreRender(this->updateConnection);
        // this->infoSub.reset();
        // if (this->node)
        //     this->node->Fini();
    }

    void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
    {
        // firstReceived = false;

        if (!_visual || !_sdf)
        {
            gzerr << "[VirtualConstraintVisual] No visual or SDF element specified. Plugin won't load." << std::endl;
            return;
        }
        this->visual = _visual;

        // Connect to the world update signal
        this->updateConnection = event::Events::ConnectPreRender(std::bind(&VirtualConstraintVisual::Update, this));

        // this->node = transport::NodePtr(new transport::Node());
        // this->node->Init();

        // FOR SINGLE SUBS!
        // this->infoSub = this->node->Subscribe("/gazebo/" + this->visual->GetName() + "/spring", &VirtualConstraintVisual::OnInfo, this);

        // Setup rendering components
        // TODO

        // gzdbg << "[VirtualConstraintVisual] " << this->visual->GetName() << " subscribed to " << this->infoSub->GetTopic() << std::endl;
        gzdbg << "[VirtualConstraintVisual] " << this->visual->GetName() << " successfully loaded!" << std::endl;
    }

  private:
    void Update()
    {
        if (!this->visual)
        {
            gzerr << "[VirtualConstraintVisual] The visual is null." << std::endl;
            return;
        }
        // if (!firstReceived)
        // {
        //     return;
        // }

        // std::lock_guard<std::mutex> lock(this->mutex);
    }

    /// \brief Visual whose color will be changed.
    rendering::VisualPtr visual;

    /// \brief Connects to rendering update event.
    event::ConnectionPtr updateConnection;

    // /// \brief Node used for communication.
    // transport::NodePtr node;

    // /// \brief Node used for communication.
    // std::mutex mutex;

    // /// \brief Subscriber to spring msg.
    // transport::SubscriberPtr infoSub;

    // /// \brief Check if spring msg was received..
    // bool firstReceived;
};

// Register this plugin with the client
GZ_REGISTER_VISUAL_PLUGIN(VirtualConstraintVisual)

} // namespace cosima