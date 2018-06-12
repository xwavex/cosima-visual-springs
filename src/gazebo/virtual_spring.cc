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

#include "spring.pb.h"
#include "spring_constraint_mapping.hh"

using namespace gazebo;

namespace cosima
{

class VirtualSpring : public ModelPlugin
{

    ~VirtualSpring()
    {
        event::Events::DisconnectWorldUpdateBegin(this->update_connection);
        event::Events::DisconnectWorldUpdateEnd(this->after_connection);
        this->factoryPub.reset();
        if (this->node)
            this->node->Fini();
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        if (!_parent || !_sdf)
        {
            gzerr << "No model or SDF element specified. Plugin won't load." << std::endl;
            return;
        }

        // Store the pointer to the model
        this->model = _parent;
        this->world = _parent->GetWorld();
        gzdbg << "Loading VirtualSpring plugin" << std::endl;

        // load parameters

        if (!_sdf->HasElement("anchor"))
        {
            gzerr << "VirtualSpring plugin missing <anchor>, cannot proceed" << std::endl;
            return;
        }
        else
        {
            this->anchor_model_name = _sdf->GetElement("anchor")->GetAttribute("model")->GetAsString();
            this->anchor_link_name = _sdf->GetElement("anchor")->GetAttribute("link")->GetAsString();
        }

        if (!_sdf->HasElement("target"))
        {
            gzerr << "VirtualSpring plugin missing <target>, cannot proceed" << std::endl;
            return;
        }
        else
        {
            this->target_model_name = _sdf->GetElement("target")->GetAttribute("model")->GetAsString();
            this->target_link_name = _sdf->GetElement("target")->GetAttribute("link")->GetAsString();
        }

        if (!_sdf->HasElement("stiffness"))
        {
            gzerr << "VirtualSpring plugin missing <stiffness>, cannot proceed" << std::endl;
            return;
        }
        else
        {
            this->stiffness = _sdf->Get<math::Vector3>("stiffness");
        }

        if (!_sdf->HasElement("damping"))
        {
            gzerr << "VirtualSpring plugin missing <damping>, cannot proceed" << std::endl;
            return;
        }
        else
        {
            this->damping = _sdf->Get<math::Vector3>("damping");
        }

        if (!_sdf->HasElement("stiffness_orient"))
        {
            gzerr << "VirtualSpring plugin missing <stiffness_orient>, cannot proceed" << std::endl;
            return;
        }
        else
        {
            this->stiffnessOrient = _sdf->Get<math::Vector3>("stiffness_orient");
        }

        if (!_sdf->HasElement("damping_orient"))
        {
            gzerr << "VirtualSpring plugin missing <damping_orient>, cannot proceed" << std::endl;
            return;
        }
        else
        {
            this->dampingOrient = _sdf->Get<math::Vector3>("damping_orient");
        }

        // get world pointer
        // world = this->model->GetWorld();
        if (!world)
        {
            gzerr << "VirtualSpring plugin cannot retrieve world pointer, cannot proceed" << std::endl;
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
        //     this->callback_queue_thread = boost::thread(boost::bind(&VirtualSpring::QueueThread, this));
        // }

        gzmsg << "Loaded CoSiMA Virtual Spring plugin on " << this->model->GetName() << std::endl;

        anchor_model = 0;
        target_model = 0;

        // Register communication
        node = transport::NodePtr(new transport::Node());
        node->Init(this->model->GetName());
        // ALL ON THE SAME TOPIC
        // factoryPub = node->Advertise<cosima_gazebo_virtual_elements::msgs::Spring>("/gazebo/" + world->GetName() + "/springs");
        this->sendSecondsElapsed = 1.0;
        // TOPIC PER SPRING
        factoryPub = node->Advertise<cosima_gazebo_virtual_elements::msgs::Spring>("/gazebo/" + this->model->GetName() + "::link::" + this->model->GetName() + "/spring");
        std::cout << "Plugin Pubs to /gazebo/" << this->model->GetName() << "::link::" << this->model->GetName() << "/spring" << std::endl;
        // std::string modelNameId = this->model->GetName() + std::to_string(this->model->GetId());
        springMsg.set_id(this->model->GetId());
        springMsg.set_name(this->model->GetName());

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&VirtualSpring::OnUpdate, this, _1));
        this->after_connection = event::Events::ConnectWorldUpdateEnd(std::bind(&VirtualSpring::OnUpdateEnd, this));
    }

    // void VirtualSpring::Connect()
    // {
    //     this->connect_count++;
    // }

    // void VirtualSpring::Disconnect()
    // {
    //     this->connect_count--;
    // }

    // void VirtualSpring::QueueThread()
    // {
    //     static const double timeout = 0.01;

    //     while (this->rosnode->ok())
    //     {
    //         this->queue.callAvailable(ros::WallDuration(timeout));
    //     }
    // }

    void OnUpdateEnd()
    {
        if (!anchor_link || !target_link)
        {
            return;
        }
        math::Pose aLinkWorldPose = this->anchor_link->GetWorldPose();
        math::Pose tLinkWorldPose = this->target_link->GetWorldPose();

        gazebo::common::Time gz_walltime = common::Time::GetWallTime();

        // last publish time is not yet over 1 sec
        if (gz_walltime.sec - old_wall_time.sec <= this->sendSecondsElapsed)
        {
            // and position is completely the same
            if (aLinkWorldPose.pos == anchor_old && tLinkWorldPose.pos == target_old && aLinkWorldPose == reference_old)
            {
                // do not send in this case to save processing power
                return;
            }
        }

        // Setup message
        springMsg.mutable_anchor()->set_x(aLinkWorldPose.pos.x);
        springMsg.mutable_anchor()->set_y(aLinkWorldPose.pos.y);
        springMsg.mutable_anchor()->set_z(aLinkWorldPose.pos.z);

        springMsg.mutable_target()->set_x(tLinkWorldPose.pos.x);
        springMsg.mutable_target()->set_y(tLinkWorldPose.pos.y);
        springMsg.mutable_target()->set_z(tLinkWorldPose.pos.z);

        // TODO plus offset!
        springMsg.mutable_reference()->mutable_position()->set_x(aLinkWorldPose.pos.x);
        springMsg.mutable_reference()->mutable_position()->set_y(aLinkWorldPose.pos.y);
        springMsg.mutable_reference()->mutable_position()->set_z(aLinkWorldPose.pos.z);
        springMsg.mutable_reference()->mutable_orientation()->set_w(aLinkWorldPose.rot.w);
        springMsg.mutable_reference()->mutable_orientation()->set_x(aLinkWorldPose.rot.x);
        springMsg.mutable_reference()->mutable_orientation()->set_y(aLinkWorldPose.rot.y);
        springMsg.mutable_reference()->mutable_orientation()->set_z(aLinkWorldPose.rot.z);

        gazebo::common::Time gz_time = gazebo::physics::get_world()->GetSimTime();
        springMsg.mutable_time()->set_nsec(gz_time.nsec);
        springMsg.mutable_time()->set_sec(gz_time.sec);

        springMsg.mutable_stiffness()->set_x(stiffness.x);
        springMsg.mutable_stiffness()->set_y(stiffness.y);
        springMsg.mutable_stiffness()->set_z(stiffness.z);

        springMsg.mutable_damping()->set_x(damping.x);
        springMsg.mutable_damping()->set_y(damping.y);
        springMsg.mutable_damping()->set_z(damping.z);

        springMsg.mutable_stiffness_orient()->set_x(stiffnessOrient.x);
        springMsg.mutable_stiffness_orient()->set_y(stiffnessOrient.y);
        springMsg.mutable_stiffness_orient()->set_z(stiffnessOrient.z);

        springMsg.mutable_damping_orient()->set_x(dampingOrient.x);
        springMsg.mutable_damping_orient()->set_y(dampingOrient.y);
        springMsg.mutable_damping_orient()->set_z(dampingOrient.z);

        springMsg.set_active(true);

        factoryPub->Publish(springMsg);

        anchor_old = aLinkWorldPose.pos;
        target_old = tLinkWorldPose.pos;
        reference_old = aLinkWorldPose;
        old_wall_time = gz_walltime;
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        if (!world)
        {
            return;
        }
        // else
        // {
        //     gzmsg << "world here" << std::endl;
        // }

        if (!anchor_model)
        {
            // trying to retrieve the anchor model
            anchor_model = world->GetModel(anchor_model_name);
        }
        // else
        // {
        //     gzmsg << "anchor_model_name here " << anchor_model_name << std::endl;
        // }
        if (anchor_model && !anchor_link)
        {
            // trying to retrieve the anchor link
            anchor_link = anchor_model->GetLink(anchor_link_name);
        }
        // else
        // {
        //     gzmsg << "anchor_link_name here " << anchor_link_name << std::endl;
        // }

        if (!target_model)
        {
            // trying to retrieve the target model
            target_model = world->GetModel(target_model_name);
        }
        // else
        // {
        //     gzmsg << "target_model_name here " << target_model_name << std::endl;
        // }
        if (target_model && !target_link)
        {
            // trying to retrieve the target link
            target_link = target_model->GetLink(target_link_name);
        }
        // else
        // {
        //     gzmsg << "target_link_name here " << target_link_name << std::endl;
        // }

        if (anchor_model && anchor_link && target_model && target_link)
        {
            // get pose of links
            math::Pose aLinkWorldPose = this->anchor_link->GetWorldPose();
            math::Pose tLinkWorldPose = this->target_link->GetWorldPose();

            // Adjust my visual
            // math::Vector3 midPoint = tLinkWorldPose.pos + aLinkWorldPose.pos;
            // auto eye = ignition::math::Vector3d(midPoint.x * 0.5, midPoint.y * 0.5, midPoint.z * 0.5);
            // auto target = ignition::math::Vector3d(tLinkWorldPose.pos.x, tLinkWorldPose.pos.y, tLinkWorldPose.pos.z);
            // auto up = ignition::math::Vector3d(0, 0, 1);
            // auto lookat = ignition::math::Matrix4d::LookAt(eye, target, up).Pose();
            // this->model->SetWorldPose(lookat);

            math::Pose accumulatedConstrainedPose;
            accumulatedConstrainedPose = math::Pose::Zero;

            // get available constraints
            SpringConstraintMapping &scm = SpringConstraintMapping::Get();
            if (scm.LinkContained(target_link_name))
            {
                SpringConstraintMapping::ConstraintPtrVec scmap = scm.GetConstraintVec(target_link_name);
                for (SpringConstraintMapping::ConstraintPtrVec::iterator it = scmap.begin(); it < scmap.end(); it++)
                {
                    std::shared_ptr<SpringConstraintMapping::Constraint> constraint_other = *it;
                    std::lock_guard<std::mutex> lock(constraint_other->constraint_mutex);
                    // TODO this is WRONG I guess!
                    accumulatedConstrainedPose += constraint_other->constraintPose;
                }
            }

            math::Vector3 translationConstrained = math::Vector3::One - accumulatedConstrainedPose.pos;
            math::Vector3 rotationConstrained = math::Vector3::One - accumulatedConstrainedPose.rot.GetAsEuler();

            // get force on links
            math::Vector3 tLinkWorldLinVelocity = this->target_link->GetWorldLinearVel();
            math::Vector3 tLinkWorldAngVelocity = this->target_link->GetWorldAngularVel();

            // spring equation
            math::Vector3 resultingForceOnTargetForce = -damping * tLinkWorldLinVelocity + stiffness * (aLinkWorldPose.pos - tLinkWorldPose.pos);

            math::Quaternion tmpQuat = aLinkWorldPose.rot - tLinkWorldPose.rot;
            math::Vector3 resultingForceOnTargetTorque = -dampingOrient * tLinkWorldAngVelocity + stiffnessOrient * tmpQuat.GetAsEuler();

            // TODO use the accumulatedConstrainedPose to prject everything to Zero!

            // apply force and torque to the target link

            // std::cout << "translationConstrained " << translationConstrained << std::endl;
            // distinguish between global and non global?
            this->target_link->AddForce(resultingForceOnTargetForce * translationConstrained);
            this->target_link->AddTorque(resultingForceOnTargetTorque * rotationConstrained);

            // std::cout << "resultingforce " << resultingForceOnTargetForce * translationConstrained << std::endl;
        }
    }

  private:
    // Pointer to the update event connection
    event::ConnectionPtr update_connection;
    event::ConnectionPtr after_connection;

    physics::ModelPtr model;
    physics::LinkPtr link;
    physics::WorldPtr world;

    std::string anchor_model_name;
    std::string anchor_link_name;
    std::string target_model_name;
    std::string target_link_name;

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

    double sendSecondsElapsed;
    gazebo::common::Time old_wall_time;

    transport::NodePtr node;
    transport::PublisherPtr factoryPub;
    cosima_gazebo_virtual_elements::msgs::Spring springMsg;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(VirtualSpring)

} // namespace cosima