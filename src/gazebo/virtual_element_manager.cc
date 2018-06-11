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

#include <ignition/math/Pose3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

using namespace gazebo;

namespace cosima
{
class VirtualElementManager : public WorldPlugin
{
public:
  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    // Option 1: Insert model from file via function call.
    // The filename must be in the GAZEBO_MODEL_PATH environment variable.
    // _parent->InsertModelFile("model://box");

    // Option 2: Insert model from string via function call.
    // Insert a sphere model from string
    world = _parent;
    worldSDF = _sdf;

    sdf::SDF sphereSDF;
    sphereSDF.SetFromString(
        "<sdf version ='1.4'>\
          <model name ='spring0'>\
            <pose>2 0 0 0 0 0</pose>\
            <link name ='link'>\
              <pose>0 0 0 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <box><size>0.5 0.5 0.5</size></box>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <box><size>0.5 0.5 0.5</size></box>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>");
    // Demonstrate using a custom model name.
    sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
    model->GetAttribute("name")->SetFromString("spring0");
    _parent->InsertModelSDF(sphereSDF);

    sdf::SDF sphereSDF2;
    sphereSDF2.SetFromString(
        "<sdf version ='1.4'>\
          <model name ='spring0'>\
            <pose>0 0 0 0 0 0</pose>\
            <link name ='link'>\
              <pose>0 0 0 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <box><size>0.5 0.5 0.5</size></box>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <box><size>0.5 0.5 0.5</size></box>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>");
    // Demonstrate using a custom model name.
    sdf::ElementPtr model2 = sphereSDF2.Root()->GetElement("model");
    model2->GetAttribute("name")->SetFromString("spring1");
    _parent->InsertModelSDF(sphereSDF2);

    // // Option 3: Insert model from file via message passing.
    // {
    //   // Create a new transport node
    //   transport::NodePtr node(new transport::Node());

    //   // Initialize the node with the world name
    //   node->Init(_parent->GetName());

    //   // Create a publisher on the ~/factory topic
    //   transport::PublisherPtr factoryPub =
    //   node->Advertise<msgs::Factory>("~/factory");

    //   // Create the message
    //   msgs::Factory msg;

    //   // Model file to load
    //   msg.set_sdf_filename("model://cylinder");

    //   // Pose to initialize the model to
    //   msgs::Set(msg.mutable_pose(),
    //       ignition::math::Pose3d(
    //         ignition::math::Vector3d(1, -2, 0),
    //         ignition::math::Quaterniond(0, 0, 0)));

    //   // Send the message
    //   factoryPub->Publish(msg);
    // }

    first = true;
    newJoint = 0;
    this->connectionWorldUpdateBegin = event::Events::ConnectWorldUpdateBegin(std::bind(&VirtualElementManager::update, this));
    this->connectionAddEntity = event::Events::ConnectAddEntity(boost::bind(&VirtualElementManager::addEntity, this, _1));
  }

  bool first;
  sdf::ElementPtr worldSDF;
  physics::JointPtr newJoint;
  physics::LinkPtr ls0;
  physics::LinkPtr ls1;

  void update()
  {
    if (world && first)
    {
      if (world->GetModelCount() >= 2)
      {
        physics::ModelPtr s0 = world->GetModel("spring0");
        physics::ModelPtr s1 = world->GetModel("spring1");
        if (s0 && s1)
        {
          // std::cout << "s0 and s1 added" << std::endl;

          ls0 = s0->GetLink("link");
          ls1 = s1->GetLink("link");

          if (ls0 && ls1)
          {
            std::cout << "ls0 and ls1 added" << std::endl;
            // delete joint is already there... etc.
            // newJoint = world->GetPhysicsEngine()->CreateJoint("prismatic", s0);
            // newJoint->Load(ls0, ls1, math::Pose());
            // newJoint->Init();
            // newJoint->SetHighStop(0, 0);
            // newJoint->SetLowStop(0, 0);

            //insert model with view save in list!
            sdf::SDF sphereSDF3;
            sphereSDF3.SetFromString(
                "<sdf version ='1.4'>\
                    <model name ='name_placeholder'>\
                      <static>true</static>\
                      <allow_auto_disable>true</allow_auto_disable>\
                      <pose>0 0 0 0 0 0</pose>\
                      <link name ='link'>\
                        <gravity>false</gravity>\
                        <pose>0 0 0 0 0 0</pose>\
                        <visual name ='name_placeholder'>\
                          <geometry>\
                            <empty/>\
                          </geometry>\
                          <plugin name='virtual_spring_visual' filename='libcosima_gazebo_virtual_spring_visual.so'>\
                            <topic>blaaa?</topic>\
                         </plugin>\
                        </visual>\
                      </link>\
                      <plugin name='virtual_spring' filename='libcosima_gazebo_virtual_spring.so'>\
                          <anchor model='spring0' link='link' />\
                          <target model='spring1' link='link' />\
                          <stiffness>5.5 5.5 5.5</stiffness>\
                          <damping>0 0 0</damping>\
                          <stiffness_orient>0.0 0.0 0.0</stiffness_orient>\
                          <damping_orient>0 0 0</damping_orient>\
                          <!-- link, direction, constraint_direction -->\
                          <anchor_type>link</anchor_type>\
                      </plugin>\
                    </model>\
                  </sdf>");
            // Demonstrate using a custom model name.
            sdf::ElementPtr model3 = sphereSDF3.Root()->GetElement("model");

            // TODO take care of unique names!
            model3->GetAttribute("name")->SetFromString("springGhost");
            model3->GetElement("link")->GetElement("visual")->GetAttribute("name")->SetFromString("springGhost");
            world->InsertModelSDF(sphereSDF3);

            std::cout << "inserted spring model with plugin" << std::endl;

            // TODO constraint which dynamic links
            sdf::SDF sphereSDF4;
            sphereSDF4.SetFromString(
                "<sdf version ='1.4'>\
                    <model name ='name_placeholder'>\
                      <static>true</static>\
                      <allow_auto_disable>true</allow_auto_disable>\
                      <pose>0 0 0 0 0 0</pose>\
                      <link name ='link'>\
                        <gravity>false</gravity>\
                        <pose>0 0 0 0 0 0</pose>\
                        <visual name ='name_placeholder'>\
                          <geometry>\
                            <empty/>\
                          </geometry>\
                          <plugin name='virtual_constraint_visual' filename='libcosima_gazebo_virtual_constraint_visual.so'>\
                            <topic>blaaa?</topic>\
                         </plugin>\
                        </visual>\
                      </link>\
                      <plugin name='virtual_constraint' filename='libcosima_gazebo_virtual_constraint.so'>\
                          <anchor model='spring0' link='link'>\
                            <offset>0 0 0 0 0 0</offset>\
                            <!-- hier kann auch ohne model and link, <frame> eine konstante definieren -->\
                          </anchor>\
                          <target model='spring1' link='link' />\
                          <!-- link, direction, constraint_direction -->\
                          <anchor_type>link</anchor_type>\
                      </plugin>\
                    </model>\
                  </sdf>");
            // Demonstrate using a custom model name.
            sdf::ElementPtr model4 = sphereSDF4.Root()->GetElement("model");

            // TODO take care of unique names!
            model4->GetAttribute("name")->SetFromString("constraintGhost");
            model4->GetElement("link")->GetElement("visual")->GetAttribute("name")->SetFromString("constraintGhost");
            // TODO DLW
            // world->InsertModelSDF(sphereSDF4);

            // TODO constraint which dynamic links
            sdf::SDF sphereSDF5;
            sphereSDF5.SetFromString(
                "<sdf version ='1.4'>\
                    <model name ='name_placeholder'>\
                      <static>true</static>\
                      <allow_auto_disable>true</allow_auto_disable>\
                      <pose>0 0 0 0 0 0</pose>\
                      <link name ='link'>\
                        <gravity>false</gravity>\
                        <pose>0 0 0 0 0 0</pose>\
                        <visual name ='name_placeholder'>\
                          <geometry>\
                            <empty/>\
                          </geometry>\
                          <plugin name='virtual_constraint_visual' filename='libcosima_gazebo_virtual_constraint_visual.so'>\
                            <topic>blaaa</topic>\
                         </plugin>\
                        </visual>\
                      </link>\
                      <plugin name='virtual_constraint' filename='libcosima_gazebo_virtual_constraint.so'>\
                          <!-- Direction hat hier nicht die semantik eines fixen ankers sondern des direction vectors -->\
                          <direction in_world_frame='true'>1 1 0</direction>\
                          <target model='spring1' link='link' />\
                          <!-- link, direction, constraint_direction -->\
                          <anchor_type>direction</anchor_type>\
                      </plugin>\
                    </model>\
                  </sdf>");
            // Demonstrate using a custom model name.
            sdf::ElementPtr model5 = sphereSDF5.Root()->GetElement("model");

            // TODO take care of unique names!
            model5->GetAttribute("name")->SetFromString("constraintGhostFixedGlobal");
            model5->GetElement("link")->GetElement("visual")->GetAttribute("name")->SetFromString("constraintGhostFixedGlobal");
            world->InsertModelSDF(sphereSDF5);

            std::cout << "inserted constraint model with plugin" << std::endl;

            first = false;
          }
        }
      }
    }
    // else if (world)
    // {
    //   physics::ModelPtr springGhost = world->GetModel("springGhost");
    //   if (springGhost)
    //   {
    //     if (newJoint && ls0 && ls1)
    //     {
    //       math::Vector3 v(1, 0.1, 0.1);
    //       // springGhost->SetScale(v);
    //       springGhost->SetWorldPose(newJoint->GetParent()->GetWorldPose() + (newJoint->GetParent()->GetWorldPose() - newJoint->GetChild()->GetWorldPose()));
    //     }
    //   }
    // }
  }

  void addEntity(const std::string str)
  {
    std::cout << "ADD ENTITY: " << str << std::endl;
    if (world)
    {
      std::cout << "World is here " << world->GetModelCount() << std::endl;
    }
    else
    {
      return;
    }
  }

  ~VirtualElementManager()
  {
    event::Events::DisconnectWorldUpdateBegin(this->connectionWorldUpdateBegin);
    event::Events::DisconnectAddEntity(this->connectionAddEntity);
  }

  physics::WorldPtr world;

  event::ConnectionPtr connectionWorldUpdateBegin;
  event::ConnectionPtr connectionAddEntity;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(VirtualElementManager)
} // namespace cosima