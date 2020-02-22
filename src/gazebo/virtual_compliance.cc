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

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>

#include "constraint.pb.h"
// #include "spring_compliance_mapping.hh"

using namespace gazebo;

namespace cosima
{

class VirtualCompliance : public ModelPlugin
{

    ~VirtualCompliance()
    {
        event::Events::DisconnectWorldUpdateBegin(this->update_connection);
        this->factoryPub.reset();
        if (this->node)
            this->node->Fini();
        // if (this->compliance)
        // {
        //     SpringComplianceMapping::Get().Remove(target_link_name, this->compliance);
        // }
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        if (!_parent || !_sdf)
        {
            gzerr << "No model or SDF element specified. Plugin won't load." << std::endl;
            return;
        }

        firstIteration = true;

        // Store the pointer to the model
        this->model = _parent;
        this->world = _parent->GetWorld();
        this->anchor_type = "link";
        gzdbg
            << "Loading VirtualCompliance plugin" << std::endl;

        // load parameters
        if (!_sdf->HasElement("anchor_type"))
        {
            gzerr << "VirtualCompliance plugin missing <anchor_type>, cannot proceed" << std::endl;
            return;
        }
        else
        {
            this->anchor_type = _sdf->GetElement("anchor_type")->GetValue()->GetAsString();
            if (this->anchor_type.empty())
            {
                gzerr << "VirtualCompliance plugin's anchor_type is empty, cannot proceed" << std::endl;
                return;
            }
            else if (this->anchor_type.compare("link") != 0 && this->anchor_type.compare("direction") != 0 && this->anchor_type.compare("compliance_direction") != 0)
            {
                gzerr << "VirtualCompliance plugin's anchor_type is unknown, cannot proceed" << std::endl;
                return;
            }
        }

        if (!_sdf->HasElement("anchor"))
        {
            //<!-- link, direction, compliance_direction -->
            if (this->anchor_type.compare("link") == 0)
            {
                gzerr << "VirtualCompliance plugin missing <anchor> while link is the anchor_type, cannot proceed" << std::endl;
                return;
            }
        }
        else
        {
            this->anchor_model_name = _sdf->GetElement("anchor")->GetAttribute("model")->GetAsString();
            if (this->anchor_model_name.empty())
            {
                gzerr << "VirtualCompliance plugin's anchor_model_name is empty, cannot proceed" << std::endl;
                return;
            }

            this->anchor_link_name = _sdf->GetElement("anchor")->GetAttribute("link")->GetAsString();
            if (this->anchor_link_name.empty())
            {
                gzerr << "VirtualCompliance plugin's anchor_link_name is empty, cannot proceed" << std::endl;
                return;
            }
        }

        if (!_sdf->HasElement("direction"))
        {
            //<!-- link, direction, compliance_direction -->
            if (this->anchor_type.compare("direction") == 0)
            {
                gzerr << "VirtualCompliance plugin missing <direction> since direction is the anchor_type, cannot proceed" << std::endl;
                return;
            }
        }
        else
        {
            this->global_anchor_as_target_offset = _sdf->Get<math::Pose>("direction");
            if (this->global_anchor_as_target_offset == math::Pose::Zero)
            {
                gzerr << "VirtualCompliance plugin's direction vector is Zero, cannot proceed" << std::endl;
                return;
            }
            isGlobal = true;
            _sdf->GetElement("direction")->GetAttribute("in_world_frame")->Get(this->isGlobal);
        }

        if (!_sdf->HasElement("target"))
        {
            gzerr << "VirtualCompliance plugin missing <target>, cannot proceed" << std::endl;
            return;
        }
        else
        {
            this->target_model_name = _sdf->GetElement("target")->GetAttribute("model")->GetAsString();
            if (this->target_model_name.empty())
            {
                gzerr << "VirtualCompliance plugin's target_model_name is empty, cannot proceed" << std::endl;
                return;
            }

            this->target_link_name = _sdf->GetElement("target")->GetAttribute("link")->GetAsString();
            if (this->target_link_name.empty())
            {
                gzerr << "VirtualCompliance plugin's target_link_name is empty, cannot proceed" << std::endl;
                return;
            }
        }

        // get world pointer
        // world = this->model->GetWorld();
        if (!world)
        {
            gzerr << "VirtualCompliance plugin cannot retrieve world pointer, cannot proceed" << std::endl;
            return;
        }

        // get access to models using polling in the callback!

        // this->link = this->model->GetLink(this->link_name);
        // if (!this->link)
        // {
        //     gzerr << "Error: link named " << this->link_name << " does not exist" << std::endl;
        //     return;
        // }

        //     // Custom Callback Queue
        //     this->callback_queue_thread = boost::thread(boost::bind(&VirtualCompliance::QueueThread, this));
        // }

        gzmsg << "Loaded CoSiMA Virtual Compliance plugin on " << this->model->GetName() << std::endl;

        anchor_model = 0;
        target_model = 0;

        node = transport::NodePtr(new transport::Node());
        node->Init(this->model->GetName());
        this->sendSecondsElapsed = 1.0;
        // TOPIC PER SPRING
        factoryPub = node->Advertise<cosima_gazebo_virtual_elements::msgs::Constraint>("/gazebo/" + this->model->GetName() + "::link::" + this->model->GetName() + "/compliance");

        complianceMsg.set_id(this->model->GetId());
        complianceMsg.set_name(this->model->GetName());

        // // add compliance to storage
        // this->compliance = std::make_shared<SpringComplianceMapping::Constraint>();
        // this->compliance->name = this->model->GetName();

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&VirtualCompliance::OnUpdate, this, _1));
        // this->after_connection = event::Events::ConnectWorldUpdateEnd(std::bind(&VirtualCompliance::OnUpdateEnd, this));
        std::cout
            << "Plugin Pubs to /gazebo/" << this->model->GetName() << "::link::" << this->model->GetName() << "/compliance" << std::endl;
    }

    // void VirtualCompliance::Connect()
    // {
    //     this->connect_count++;
    // }

    // void VirtualCompliance::Disconnect()
    // {
    //     this->connect_count--;
    // }

    // void VirtualCompliance::QueueThread()
    // {
    //     static const double timeout = 0.01;

    //     while (this->rosnode->ok())
    //     {
    //         this->queue.callAvailable(ros::WallDuration(timeout));
    //     }
    // }

    // void OnUpdateEnd()
    // {
    //     if (!anchor_link || !target_link)
    //     {
    //         return;
    //     }
    //     math::Pose aLinkWorldPose = this->anchor_link->GetWorldPose();
    //     math::Pose tLinkWorldPose = this->target_link->GetWorldPose();

    //     if (aLinkWorldPose.pos == anchor_old && tLinkWorldPose.pos == target_old && aLinkWorldPose == reference_old)
    //     {
    //         // Skip sending if everything is ok!
    //         return;
    //     }

    //     anchor_old = aLinkWorldPose.pos;
    //     target_old = tLinkWorldPose.pos;
    //     reference_old = aLinkWorldPose;
    // }

    void retrieveAnchorWrtType(std::string anchor_type)
    {
        // anchor type LINK
        if (anchor_type.compare("link") == 0)
        {
            if (!anchor_model)
            {
                // trying to retrieve the anchor model
                anchor_model = world->GetModel(anchor_model_name);
            }

            if (anchor_model && !anchor_link)
            {
                // trying to retrieve the anchor link
                anchor_link = anchor_model->GetLink(anchor_link_name);
            }
        }
        // // anchor type DIRECTION
        // else if (anchor_type.compare("direction") == 0)
        // {

        // }
    }

    bool allInformationAvailableToProceed(std::string anchor_type)
    {
        // anchor type LINK
        if (anchor_type.compare("link") == 0)
        {
            if (this->anchor_model && this->anchor_link && this->target_model && this->target_link)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        // anchor type DIRECTION
        else if (anchor_type.compare("direction") == 0)
        {
            if (this->target_model && this->target_link)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            // TODO add constrained
            return false;
        }
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        if (!world)
        {
            return;
        }

        // retrieve anchor
        retrieveAnchorWrtType(this->anchor_type);

        if (!target_model)
        {
            // trying to retrieve the target model
            target_model = world->GetModel(target_model_name);
        }

        if (target_model && !target_link)
        {
            // trying to retrieve the target link
            target_link = target_model->GetLink(target_link_name);
        }

        if (allInformationAvailableToProceed(this->anchor_type))
        {
            // if (firstIteration)
            // {
            //     // Only add each compliance once!
            //     SpringConstraintMapping::Get().Add(target_link_name, this->compliance);
            //     firstIteration = false;
            // }
            // get pose of links
            math::Pose aLinkWorldPose;
            math::Vector3 lookAtVector;
            math::Pose tLinkWorldPose = this->target_link->GetWorldPose();

            if (anchor_type.compare("link") == 0)
            {
                aLinkWorldPose = this->anchor_link->GetWorldPose();
                lookAtVector = aLinkWorldPose.pos;
            }
            else
            {
                if (isGlobal)
                {
                    lookAtVector = (tLinkWorldPose.pos + this->global_anchor_as_target_offset.pos);
                }
                else
                {
                    lookAtVector = tLinkWorldPose.pos + tLinkWorldPose.rot.RotateVector(this->global_anchor_as_target_offset.pos);
                }
            }

            auto eye = ignition::math::Vector3d(tLinkWorldPose.pos.x, tLinkWorldPose.pos.y, tLinkWorldPose.pos.z);
            auto target = ignition::math::Vector3d(lookAtVector.x, lookAtVector.y, lookAtVector.z);
            auto up = ignition::math::Vector3d(0, 0, 1);
            auto lookat = ignition::math::Matrix4d::LookAt(eye, target, up).Pose();
            this->model->SetWorldPose(lookat);

            math::Vector3 notGlobalCaseTranslationComplianceVec = tLinkWorldPose.rot.RotateVector(this->global_anchor_as_target_offset.pos);

            gazebo::common::Time gz_walltime = common::Time::GetWallTime();
            if (factoryPub->HasConnections() && (gz_walltime.sec - old_wall_time.sec > this->sendSecondsElapsed))
            {
                complianceMsg.set_anchor_type(this->anchor_type);

                complianceMsg.mutable_anchor()->mutable_orientation()->set_w(this->global_anchor_as_target_offset.rot.w);
                complianceMsg.mutable_anchor()->mutable_orientation()->set_x(this->global_anchor_as_target_offset.rot.x);
                complianceMsg.mutable_anchor()->mutable_orientation()->set_y(this->global_anchor_as_target_offset.rot.y);
                complianceMsg.mutable_anchor()->mutable_orientation()->set_z(this->global_anchor_as_target_offset.rot.z);
                if (this->isGlobal)
                {
                    complianceMsg.mutable_anchor()->mutable_position()->set_x(this->global_anchor_as_target_offset.pos.x);
                    complianceMsg.mutable_anchor()->mutable_position()->set_y(this->global_anchor_as_target_offset.pos.y);
                    complianceMsg.mutable_anchor()->mutable_position()->set_z(this->global_anchor_as_target_offset.pos.z);
                }
                else
                {
                    complianceMsg.mutable_anchor()->mutable_position()->set_x(notGlobalCaseTranslationComplianceVec.x);
                    complianceMsg.mutable_anchor()->mutable_position()->set_y(notGlobalCaseTranslationComplianceVec.y);
                    complianceMsg.mutable_anchor()->mutable_position()->set_z(notGlobalCaseTranslationComplianceVec.z);
                }
                complianceMsg.set_in_world_frame(this->isGlobal);

                gazebo::common::Time gz_time = gazebo::physics::get_world()->GetSimTime();
                complianceMsg.mutable_time()->set_nsec(gz_time.nsec);
                complianceMsg.mutable_time()->set_sec(gz_time.sec);

                factoryPub->Publish(complianceMsg);
                old_wall_time = gz_walltime;
            }

            // // update global storage
            // {
            //     std::lock_guard<std::mutex> lock(this->compliance->compliance_mutex);
            //     this->compliance->in_world_frame = isGlobal;
            //     if (isGlobal)
            //     {
            //         this->compliance->compliancePose = this->global_anchor_as_target_offset;
            //     }
            //     else
            //     {
            //         // TODO not sure if this is correct?
            //         this->compliance->compliancePose.pos = notGlobalCaseTranslationComplianceVec;
            //         this->compliance->compliancePose.rot = this->global_anchor_as_target_offset.rot;
            //     }
            // }
        }
    }

  private:
    // Pointer to the update event connection
    event::ConnectionPtr update_connection;

    physics::ModelPtr model;
    physics::LinkPtr link;
    physics::WorldPtr world;

    std::string anchor_model_name;
    std::string anchor_link_name;
    std::string target_model_name;
    std::string target_link_name;
    std::string anchor_type;

    math::Vector3 stiffness;
    math::Vector3 stiffnessOrient;
    math::Vector3 dampingOrient;
    math::Vector3 damping;

    physics::ModelPtr anchor_model;
    physics::ModelPtr target_model;
    physics::LinkPtr anchor_link;
    physics::LinkPtr target_link;

    math::Vector3 anchor_old;
    math::Vector3 target_old;
    math::Pose reference_old;

    bool isGlobal;

    double sendSecondsElapsed;
    gazebo::common::Time old_wall_time;

    math::Pose global_anchor_as_target_offset;

    transport::NodePtr node;
    transport::PublisherPtr factoryPub;

    cosima_gazebo_virtual_elements::msgs::Constraint complianceMsg;

    // Storage container
    // std::shared_ptr<SpringComplianceMapping::Constraint> compliance;

    bool firstIteration;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(VirtualCompliance)

} // namespace cosima