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

#include "spring.pb.h"

using namespace gazebo;

namespace cosima
{

class VirtualSpringVisual : public VisualPlugin
{
  public:
    ~VirtualSpringVisual()
    {
        event::Events::DisconnectPreRender(this->updateConnection);
        this->infoSub.reset();
        if (this->node)
            this->node->Fini();
    }

    void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
    {
        firstReceived = false;
        amountOfSegments = 20;
        amplitude = 0.05;

        if (!_visual || !_sdf)
        {
            gzerr << "[VirtualSpringVisual] No visual or SDF element specified. Plugin won't load." << std::endl;
            return;
        }
        this->visual = _visual;
        this->scene = rendering::get_scene();
        this->visual_draw = gazebo::rendering::VisualPtr(new gazebo::rendering::Visual(this->visual->GetName() + "_draw", scene));
        this->visual_draw->Load();
        this->visual_draw->SetVisible(true);

        // Connect to the world update signal
        this->updateConnection = event::Events::ConnectPreRender(std::bind(&VirtualSpringVisual::Update, this));

        this->node = transport::NodePtr(new transport::Node());
        this->node->Init();

        // FOR SINGLE SUBS!
        this->infoSub = this->node->Subscribe("/gazebo/" + this->visual->GetName() + "/spring", &VirtualSpringVisual::OnInfo, this);

        // Setup rendering components
        line = visual_draw->CreateDynamicLine(gazebo::rendering::RENDERING_LINE_STRIP);
        line->setMaterial("Gazebo/Turquoise");
        line->setVisibilityFlags(GZ_VISIBILITY_GUI);

        stiffnessText = new rendering::MovableText();
        stiffnessText->Load(this->visual_draw->GetName() + "__STIFFNESS_TEXT__", "Stiffness", "Arial", 0.03);
        stiffnessText->SetShowOnTop(true);
        // stiffnessText->MovableObject::getUserObjectBindings().setUserAny(
        //     Ogre::Any(std::string(this->visual_draw->GetName())));
        stiffnessTextSceneNode = this->visual_draw->GetSceneNode()->createChildSceneNode(this->visual_draw->GetName() + "__STIFFNESS_TEXT_NODE__");
        stiffnessTextSceneNode->attachObject(stiffnessText);
        stiffnessTextSceneNode->setInheritScale(false);

        dampingText = new rendering::MovableText();
        dampingText->Load(this->visual_draw->GetName() + "__damping_TEXT__", "Damping", "Arial", 0.03);
        dampingText->SetShowOnTop(true);
        // dampingText->MovableObject::getUserObjectBindings().setUserAny(
        //     Ogre::Any(std::string(this->visual_draw->GetName())));
        dampingTextSceneNode = this->visual_draw->GetSceneNode()->createChildSceneNode(this->visual_draw->GetName() + "__damping_TEXT_NODE__");
        dampingTextSceneNode->attachObject(dampingText);
        dampingTextSceneNode->setInheritScale(false);

        stiffness_orientText = new rendering::MovableText();
        stiffness_orientText->Load(this->visual_draw->GetName() + "__stiffness_orient_TEXT__", "stiffness_orient", "Arial", 0.03);
        stiffness_orientText->SetShowOnTop(true);
        // stiffness_orientText->MovableObject::getUserObjectBindings().setUserAny(
        //     Ogre::Any(std::string(this->visual_draw->GetName())));
        stiffness_orientTextSceneNode = this->visual_draw->GetSceneNode()->createChildSceneNode(this->visual_draw->GetName() + "__stiffness_orient_TEXT_NODE__");
        stiffness_orientTextSceneNode->attachObject(stiffness_orientText);
        stiffness_orientTextSceneNode->setInheritScale(false);

        damping_orientText = new rendering::MovableText();
        damping_orientText->Load(this->visual_draw->GetName() + "__damping_orient_TEXT__", "damping_orient", "Arial", 0.03);
        damping_orientText->SetShowOnTop(true);
        // damping_orientText->MovableObject::getUserObjectBindings().setUserAny(
        //     Ogre::Any(std::string(this->visual_draw->GetName())));
        damping_orientTextSceneNode = this->visual_draw->GetSceneNode()->createChildSceneNode(this->visual_draw->GetName() + "__damping_orient_TEXT_NODE__");
        damping_orientTextSceneNode->attachObject(damping_orientText);
        damping_orientTextSceneNode->setInheritScale(false);

        // reference visual
        referenceFrameVis = gazebo::rendering::LinkFrameVisualPtr(new gazebo::rendering::LinkFrameVisual("_LINK_FRAME_VISUAL_", this->visual_draw));
        referenceFrameVis->Load();
        referenceFrameVis->SetVisible(false);
        referenceFrameVis->SetAxisMaterial(0, "Gazebo/Red");
        referenceFrameVis->SetAxisMaterial(1, "Gazebo/Green");
        referenceFrameVis->SetAxisMaterial(2, "Gazebo/Blue");

        gzdbg
            << "[VirtualSpringVisual] " << this->visual->GetName() << " subscribed to " << this->infoSub->GetTopic() << std::endl;
    }

  private:
    void Update()
    {
        if (!this->visual)
        {
            gzerr << "[VirtualSpringVisual] The visual is null." << std::endl;
            return;
        }
        if (!firstReceived)
        {
            return;
        }
        if (anchor_old == anchor && target_old == target && reference_old == reference)
        {
            // Skip recalculations is nothing is changing!
            return;
        }

        std::lock_guard<std::mutex> lock(this->mutex);

        line->Clear();
        double dist = anchor.Distance(target);

        math::Vector3 diffVec = anchor - target;

        // beginning point
        line->AddPoint(target.x, target.y, target.z, common::Color::White);

        // middle points
        double steps = 1 / amountOfSegments;
        for (unsigned int i = 1; i < amountOfSegments; i++)
        {
            double stepSize = i * steps;

            // TODO in the future project triangle to actual diffVec
            double modZ = 0;
            if (i % 2 == 0)
            {
                modZ = -amplitude;
            }
            else
            {
                modZ = amplitude;
            }
            if (i == 1 || i == (amountOfSegments - 1))
            {
                modZ = 0;
            }
            line->AddPoint(target.x + diffVec.x * stepSize, target.y + diffVec.y * stepSize, target.z + diffVec.z * stepSize + modZ, common::Color::White);
        }

        // last point
        line->AddPoint(anchor.x, anchor.y, anchor.z, common::Color::White);

        math::Vector3 middleVec = target + diffVec * 0.5;
        // set stiffness
        stiffnessText->SetText("Stiff.L.: " + std::to_string(stiffness.x) + ", " + std::to_string(stiffness.y) + ", " + std::to_string(stiffness.z));
        stiffnessTextSceneNode->setPosition(middleVec.x, middleVec.y, middleVec.z);

        // set damping
        dampingText->SetText("Damp.L.: " + std::to_string(damping.x) + ", " + std::to_string(damping.y) + ", " + std::to_string(damping.z));
        dampingTextSceneNode->setPosition(middleVec.x, middleVec.y, middleVec.z - 0.03);

        // set stiffness_orient
        stiffness_orientText->SetText("Stiff.A.: " + std::to_string(stiffness_orient.x) + ", " + std::to_string(stiffness_orient.y) + ", " + std::to_string(stiffness_orient.z));
        stiffness_orientTextSceneNode->setPosition(middleVec.x, middleVec.y, middleVec.z - 0.06);

        // set damping_orient
        damping_orientText->SetText("Damp.A.: " + std::to_string(damping_orient.x) + ", " + std::to_string(damping_orient.y) + ", " + std::to_string(damping_orient.z));
        damping_orientTextSceneNode->setPosition(middleVec.x, middleVec.y, middleVec.z - 0.09);

        if (!referenceFrameVis->GetVisible())
        {
            referenceFrameVis->SetVisible(true);
        }

        referenceFrameVis->SetPose(reference);

        // save old values
        anchor_old = anchor;
        target_old = target;
        reference_old = reference;
    }

    void OnInfo(const boost::shared_ptr<cosima_gazebo_virtual_spring::msgs::Spring const> &_msg)
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

        // get Anchor Vector
        anchor.x = _msg->anchor().x();
        anchor.y = _msg->anchor().y();
        anchor.z = _msg->anchor().z();

        // get Target Vector
        target.x = _msg->target().x();
        target.y = _msg->target().y();
        target.z = _msg->target().z();

        // get Reference Vector
        reference.pos.x = _msg->reference().position().x();
        reference.pos.y = _msg->reference().position().y();
        reference.pos.z = _msg->reference().position().z();
        reference.rot.w = _msg->reference().orientation().w();
        reference.rot.x = _msg->reference().orientation().x();
        reference.rot.y = _msg->reference().orientation().y();
        reference.rot.z = _msg->reference().orientation().z();

        // get stiffness Vector
        stiffness.x = _msg->stiffness().x();
        stiffness.y = _msg->stiffness().y();
        stiffness.z = _msg->stiffness().z();

        // get damping Vector
        damping.x = _msg->damping().x();
        damping.y = _msg->damping().y();
        damping.z = _msg->damping().z();

        // get stiffness_orient Vector
        stiffness_orient.x = _msg->stiffness_orient().x();
        stiffness_orient.y = _msg->stiffness_orient().y();
        stiffness_orient.z = _msg->stiffness_orient().z();

        // get damping_orient Vector
        damping_orient.x = _msg->damping_orient().x();
        damping_orient.y = _msg->damping_orient().y();
        damping_orient.z = _msg->damping_orient().z();
    }

    /// \brief Visual whose color will be changed.
    rendering::VisualPtr visual;
    rendering::VisualPtr visual_draw;

    /// \brief Connects to rendering update event.
    event::ConnectionPtr updateConnection;

    /// \brief Node used for communication.
    transport::NodePtr node;

    /// \brief Node used for communication.
    std::mutex mutex;

    /// \brief Subscriber to spring msg.
    transport::SubscriberPtr infoSub;

    /// \brief Store for anchor vector.
    math::Vector3 anchor;

    /// \brief Store for target vector.
    math::Vector3 target;

    /// \brief Store for reference vector.
    math::Pose reference;

    /// \brief OLD Store for anchor vector.
    math::Vector3 anchor_old;

    /// \brief OLD Store for target vector.
    math::Vector3 target_old;

    /// \brief OLD Store for reference vector.
    math::Pose reference_old;

    /// \brief Store for stiffness vector.
    math::Vector3 stiffness;

    /// \brief Store for damping vector.
    math::Vector3 damping;

    /// \brief Store for stiffness_orient vector.
    math::Vector3 stiffness_orient;

    /// \brief Store for damping_orient vector.
    math::Vector3 damping_orient;

    /// \brief Check if spring msg was received..
    bool firstReceived;

    /// \brief Amount of triangle segments to render.
    double amountOfSegments;

    /// \brief Amplitude for the triangle to render..
    double amplitude;

    /// \brief Triangle line.
    rendering::DynamicLines *line;

    /// \brief Pointer to scene.
    rendering::ScenePtr scene;

    /// \brief MovableText for stiffness.
    rendering::MovableText *stiffnessText;
    /// \brief SceneNode for stiffness.
    Ogre::SceneNode *stiffnessTextSceneNode;

    /// \brief MovableText for daming.
    rendering::MovableText *dampingText;
    /// \brief SceneNode for daming.
    Ogre::SceneNode *dampingTextSceneNode;

    /// \brief MovableText for stiffness_orient.
    rendering::MovableText *stiffness_orientText;
    /// \brief SceneNode for stiffness_orient.
    Ogre::SceneNode *stiffness_orientTextSceneNode;

    /// \brief MovableText for damping_orient.
    rendering::MovableText *damping_orientText;
    /// \brief SceneNode for damping_orient.
    Ogre::SceneNode *damping_orientTextSceneNode;

    /// \brief Linkframe visual for reference frame.
    gazebo::rendering::LinkFrameVisualPtr referenceFrameVis;
};

// Register this plugin with the client
GZ_REGISTER_VISUAL_PLUGIN(VirtualSpringVisual)

} // namespace cosima