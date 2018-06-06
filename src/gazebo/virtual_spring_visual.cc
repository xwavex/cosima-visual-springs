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

        if (!_visual || !_sdf)
        {
            gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
            std::cout << "no visual" << std::endl;
            return;
        }
        this->visual = _visual;
        this->scene = rendering::get_scene();
        this->visual_draw = gazebo::rendering::VisualPtr(new gazebo::rendering::Visual("bla", scene));
        this->visual_draw->Load();
        this->visual_draw->SetVisible(true);

        // Connect to the world update signal
        this->updateConnection = event::Events::ConnectPreRender(std::bind(&VirtualSpringVisual::Update, this));

        this->node = transport::NodePtr(new transport::Node());
        this->node->Init();

        // FOR SINGLE SUBS!
        this->infoSub = this->node->Subscribe("/gazebo/" + this->visual->GetName() + "/spring", &VirtualSpringVisual::OnInfo, this);

        line = visual_draw->CreateDynamicLine(gazebo::rendering::RENDERING_LINE_STRIP);
        line->setMaterial("Gazebo/Turquoise");
        line->setVisibilityFlags(GZ_VISIBILITY_GUI);

        gzerr << "VisualPlugin subs to " << this->infoSub->GetTopic() << std::endl;
        std::cout << "VisualPlugin subs to " << this->infoSub->GetTopic() << std::endl;
    }

  private:
    void Update()
    {
        if (!this->visual)
        {
            gzerr << "The visual is null." << std::endl;
            return;
        }
        if (!firstReceived)
        {
            return;
        }

        std::lock_guard<std::mutex> lock(this->mutex);

        line->Clear();
        double dist = anchor.Distance(target);

        double amountOfSegments = 20;
        math::Vector3 diffVec = anchor - target;

        // beginning point
        line->AddPoint(target.x, target.y, target.z, common::Color::White);
        // post beginning point
        // line->AddPoint(target.x + diffVec.x * 0.1, target.y + diffVec.y * 0.1, target.z + diffVec.z * 0.1, common::Color::White);
        double steps = 1 / amountOfSegments;
        // middle points
        for (unsigned int i = 1; i < amountOfSegments; i++)
        {
            double stepSize = i * steps;
            double modZ = 0;
            double amplitude = 0.05;
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
            // std::cout << "stepSize " << stepSize << std::endl;
            line->AddPoint(target.x + diffVec.x * stepSize, target.y + diffVec.y * stepSize, target.z + diffVec.z * stepSize + modZ, common::Color::White);
        }

        // line->AddPoint(target.x + diffVec.x, target.y + diffVec.y, target.z + diffVec.z, common::Color::White);

        // pre last point
        // line->AddPoint(target.x + diffVec.x * 0.9, target.y + diffVec.y * 0.9, target.z + diffVec.z * 0.9, common::Color::White);

        // last point
        line->AddPoint(anchor.x, anchor.y, anchor.z, common::Color::White);
    }

    void OnInfo(const boost::shared_ptr<cosima_gazebo_virtual_spring::msgs::Spring const> &_msg)
    {
        std::lock_guard<std::mutex> lock(this->mutex);
        // this->dataPtr->currentSimTime = msgs::Convert(_msg->time());

        // check for matching name

        std::string visualMsgName = _msg->name() + "::link::" + _msg->name();
        // std::cout << "compare " << this->visual->GetName() << " whith " << visualMsgName << std::endl;
        if (this->visual->GetName().compare(visualMsgName) != 0)
        {
            // Message is not meant for me, so skipping!
            return;
        }
        if (!firstReceived)
        {
            firstReceived = true;
        }

        // std::cout << "got message " << std::endl;

        // get Anchor Vector
        anchor.x = _msg->anchor().x();
        anchor.y = _msg->anchor().y();
        anchor.z = _msg->anchor().z();

        // get Target Vector
        target.x = _msg->target().x();
        target.y = _msg->target().y();
        target.z = _msg->target().z();

        // get Reference Vector
        reference.x = _msg->reference().x();
        reference.y = _msg->reference().y();
        reference.z = _msg->reference().z();
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

    math::Vector3 anchor;
    math::Vector3 target;
    math::Vector3 reference;

    bool firstReceived;

    rendering::DynamicLines *line;

    rendering::ScenePtr scene;
};

// Register this plugin with the client
GZ_REGISTER_VISUAL_PLUGIN(VirtualSpringVisual)

} // namespace cosima