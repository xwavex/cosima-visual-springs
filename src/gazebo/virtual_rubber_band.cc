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
#include "spring_constraint_mapping.hh"

using namespace gazebo;

namespace cosima
{

class VirtualRubberBand : public ModelPlugin
{

    ~VirtualRubberBand()
    {
        event::Events::DisconnectWorldUpdateBegin(this->update_connection);
        event::Events::DisconnectWorldUpdateBegin(this->after_connection);
        // this->factoryPub.reset();
        // if (this->node)
        //     this->node->Fini();
        // if (this->constraint)
        // {
        //     SpringConstraintMapping::Get().Remove(target_link_name, this->constraint);
        // }
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
        // this->world = _parent->GetWorld();
        gzdbg
            << "Loading VirtualRubberBand plugin on " << this->model->GetName() << std::endl;

        // Listen to the update event. This event is broadcast every simulation iteration.
        this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&VirtualRubberBand::OnUpdate, this, _1));
        this->after_connection = event::Events::ConnectWorldUpdateEnd(std::bind(&VirtualRubberBand::OnUpdateEnd, this));
        // std::cout
        //     << "Plugin Pubs to /gazebo/" << this->model->GetName() << "::link::" << this->model->GetName() << "/constraint" << std::endl;
    }

    void OnUpdateEnd()
    {
        gazebo::physics::Joint_V jointList = this->model->GetJoints();
        for (uint i = 0; i < jointList.size(); i++)
        {

            gazebo::physics::JointPtr tmp = jointList[i];
            // gzwarn << "check " << i << ", " << tmp->GetType() << std::endl;
            if (tmp->GetType() == 1088)
            {
                gazebo::math::Angle ang = tmp->GetAngle(0);
                double rad = ang.Radian();
                // gzwarn << "joint " << i << ", rad " << rad << ", f " << tmp->GetForceTorque(0).body1Force << ", v " << tmp->GetVelocity(0) << std::endl;

                // double kp = 0.7;
                // double kd = 0.1;

                // tmp->SetForce(0, kp * (-rad) + kd * -tmp->GetVelocity(0));
            }
        }
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        // if (!world)
        // {
        //     return;
        // }

        // TODO do get links only once!
        // gazebo::physics::Joint_V jointList = this->model->GetJoints();
        // for (uint i = 0; i < jointList.size(); i++)
        // {

        //     gazebo::physics::JointPtr tmp = jointList[i];
        //     // gzwarn << "check " << i << ", " << tmp->GetType() << std::endl;
        //     if (tmp->GetType() == 1088)
        //     {
        //         gazebo::math::Angle ang = tmp->GetAngle(0);
        //         double rad = ang.Radian();
        //         // gzwarn << "rad " << i << ", " << rad << std::endl;

        //         tmp->SetForce(0, -0.5);
        //     }
        // }

        // // retrieve anchor
        // retrieveAnchorWrtType(this->anchor_type);

        // if (!target_model)
        // {
        //     // trying to retrieve the target model
        //     target_model = world->GetModel(target_model_name);
        // }

        // if (target_model && !target_link)
        // {
        //     // trying to retrieve the target link
        //     target_link = target_model->GetLink(target_link_name);
        // }
    }

  private:
    // Pointer to the update event connection
    event::ConnectionPtr update_connection, after_connection;

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

    gazebo::common::Time old_wall_time;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(VirtualRubberBand)

} // namespace cosima