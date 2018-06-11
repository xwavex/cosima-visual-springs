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

#include "constraint.pb.h"

using namespace gazebo;

namespace cosima
{

class VirtualConstraintVisual : public VisualPlugin
{
  public:
    ~VirtualConstraintVisual()
    {
        event::Events::DisconnectPreRender(this->updateConnection);
        this->infoSub.reset();
        if (this->node)
            this->node->Fini();
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
        this->visual->SetVisible(true);

        globalMaterialName = "Constraint/global";

        this->lineLength = 0.4;

        this->in_world_frame = true;

        // this->scene = rendering::get_scene();
        this->visual_draw = gazebo::rendering::VisualPtr(new gazebo::rendering::Visual(this->visual->GetName() + "_draw", this->visual));
        this->visual_draw->Load();
        this->visual_draw->SetVisible(true);

        // Setup rendering components
        this->line = visual_draw->CreateDynamicLine(gazebo::rendering::RENDERING_LINE_STRIP);
        this->line->setMaterial("Gazebo/White");
        this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);

        this->inWorldFrameText = new rendering::MovableText();
        this->inWorldFrameText->Load(this->visual_draw->GetName() + "__inworldframe_TEXT__", "Damping", "Arial", 0.03, common::Color::Red);
        this->inWorldFrameText->SetShowOnTop(true);
        this->inWorldFrameTextSceneNode = this->visual_draw->GetSceneNode()->createChildSceneNode(this->visual_draw->GetName() + "__inworldframe_TEXT_NODE__");
        this->inWorldFrameTextSceneNode->attachObject(inWorldFrameText);
        this->inWorldFrameTextSceneNode->setInheritScale(false);

        // Connect to the world update signal
        this->updateConnection = event::Events::ConnectPreRender(std::bind(&VirtualConstraintVisual::Update, this));

        this->node = transport::NodePtr(new transport::Node());
        this->node->Init();

        // FOR SINGLE SUBS!
        this->infoSub = this->node->Subscribe("/gazebo/" + this->visual->GetName() + "/constraint", &VirtualConstraintVisual::OnInfo, this);

        globalMaterial = Ogre::MaterialManager::getSingleton().getByName(globalMaterialName);
        if (globalMaterial.isNull())
        {
            globalMaterial = Ogre::MaterialManager::getSingleton().create(globalMaterialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            globalMaterial->setAmbient(0.94, 0.27, 0.38);
            globalMaterial->setDiffuse(0.94, 0.27, 0.38, 1);
            globalMaterial->setSpecular(0.1, 0.1, 0.1, 1);
        }
        this->line->setMaterial(globalMaterialName);

        localMaterial = Ogre::MaterialManager::getSingleton().getByName(localMaterialName);
        if (localMaterial.isNull())
        {
            localMaterial = Ogre::MaterialManager::getSingleton().create(localMaterialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            localMaterial->setAmbient(0.48, 0.33, 0.63);
            localMaterial->setDiffuse(0.48, 0.33, 0.63, 1);
            localMaterial->setSpecular(0.1, 0.1, 0.1, 1);
        }

        // material Gazebo / Blue
        // {
        //     technique
        //     {
        //         pass ambient
        //         {
        //             ambient 0 0 1 diffuse 0 0 1 specular 0.1 0.1 0.1 1 1
        //         }
        //     }
        // }

        gzdbg
            << "[VirtualConstraintVisual] " << this->visual->GetName() << " subscribed to " << this->infoSub->GetTopic() << " and is successfully loaded!" << std::endl;
    }

  private:
    void Update()
    {
        if (!this->visual || !this->visual_draw)
        {
            gzerr << "[VirtualConstraintVisual] The visual is null." << std::endl;
            return;
        }

        if (!this->line)
        {
            gzerr << "[VirtualConstraintVisual] The line to draw is null." << std::endl;
            return;
        }

        if (!line->isVisible())
        {
            line->setVisible(true);
        }

        std::lock_guard<std::mutex> lock(this->mutex);

        line->Clear();

        gazebo::common::Color color;
        if (firstReceived && !in_world_frame)
        {
            color = common::Color(0.48, 0.33, 0.63, 1.0);
            this->line->setMaterial(localMaterialName);
        }
        else
        {
            color = common::Color(0.94, 0.27, 0.38, 1.0);
            this->line->setMaterial(globalMaterialName);
        }

        line->AddPoint(0, 0, 0, color);
        line->AddPoint(lineLength, 0, 0, color);
        // cross
        double crossLength = lineLength * 0.2;
        line->AddPoint(lineLength, crossLength, crossLength, color);
        line->AddPoint(lineLength, -crossLength, -crossLength, color);
        line->AddPoint(lineLength, 0, 0, color);
        line->AddPoint(lineLength, crossLength, -crossLength, color);
        line->AddPoint(lineLength, -crossLength, crossLength, color);

        math::Vector3 vec(1, 0, 0);
        math::Vector3 finalVec = this->visual_draw->GetWorldPose().rot.RotateVector(vec);
        inWorldFrameText->SetText("(" + std::to_string(finalVec.x) + ", " + std::to_string(finalVec.y) + ", " + std::to_string(finalVec.z) + ")");
        inWorldFrameText->SetColor(color);
        inWorldFrameTextSceneNode->setPosition(0, 0, 0);
    }

    void OnInfo(const boost::shared_ptr<cosima_gazebo_virtual_elements::msgs::Constraint const> &_msg)
    {
        std::lock_guard<std::mutex> lock(this->mutex);
        // check for matching name
        std::string visualMsgName = _msg->name() + "::link::" + _msg->name();
        if (this->visual->GetName().compare(visualMsgName) != 0)
        {
            // Message is not meant for me, so skipping!
            return;
        }
        if (!firstReceived)
        {
            firstReceived = true;
        }
        in_world_frame = _msg->in_world_frame();
    }

    /// \brief Visual whose color will be changed.
    rendering::VisualPtr visual;
    rendering::VisualPtr visual_draw;

    /// \brief length of line.
    double lineLength;

    /// \brief Connects to rendering update event.
    event::ConnectionPtr updateConnection;

    /// \brief MovableText for daming.
    rendering::MovableText *inWorldFrameText;
    /// \brief SceneNode for daming.
    Ogre::SceneNode *inWorldFrameTextSceneNode;

    /// \brief Node used for communication.
    transport::NodePtr node;

    /// \brief Node used for communication.
    std::mutex mutex;

    /// \brief Subscriber to constraint msg.
    transport::SubscriberPtr infoSub;

    /// \brief Triangle line.
    rendering::DynamicLines *line;

    /// \brief Check if constraint msg was received.
    bool firstReceived;

    /// \brief Check if constraint is in world frame.
    bool in_world_frame;

    /// \brief Custom Ogre Material.
    Ogre::MaterialPtr globalMaterial;

    /// \brief Custom Ogre Material name.
    std::string globalMaterialName;

    /// \brief Custom Ogre Material.
    Ogre::MaterialPtr localMaterial;

    /// \brief Custom Ogre Material name.
    std::string localMaterialName;
};

// Register this plugin with the client
GZ_REGISTER_VISUAL_PLUGIN(VirtualConstraintVisual)

} // namespace cosima