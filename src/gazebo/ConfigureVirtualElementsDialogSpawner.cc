/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include <sstream>
// #include <gazebo/msgs/msgs.hh>
#include "ConfigureVirtualElementsDialogSpawner.hh"
#include <gazebo/rendering/rendering.hh>

using namespace gazebo;
using namespace cosima;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(cosima::ConfigureVirtualElementsDialogSpawner)

/////////////////////////////////////////////////
ConfigureVirtualElementsDialogSpawner::ConfigureVirtualElementsDialogSpawner()
    : GUIPlugin()
{
    // this->counter = 0;
    // Set the frame background and foreground colors
    this->setStyleSheet(
        "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    // TODO perhaps wrong?
    ptr = std::shared_ptr<gazebo::gui::ConfigureVirtualElementsDialog>(new gazebo::gui::ConfigureVirtualElementsDialog(this));

    // Create the layout that sits inside the frame
    QVBoxLayout *frameLayout = new QVBoxLayout();

    // Create a push button, and connect it to the OnButton function
    button = new QPushButton(tr("CoSiMA Virtual Elements"));
    connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));
    button->setVisible(false);

    // Add the button to the frame's layout
    frameLayout->addWidget(button);

    // Add frameLayout to the frame
    mainFrame->setLayout(frameLayout);

    // Add the frame to the main layout
    mainLayout->addWidget(mainFrame);

    // Remove margins to reduce space
    frameLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);

    // Position and resize this widget
    this->move(10, 10);
    this->resize(200, 40);

    firstPreRenderer = true;
    this->connectionPreRender = gazebo::event::Events::ConnectPreRender(std::bind(&ConfigureVirtualElementsDialogSpawner::OnPreRender, this));
}

void ConfigureVirtualElementsDialogSpawner::OnPreRender()
{
    if (firstPreRenderer)
    {
        this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
        this->node->Init();
        this->requestPub = this->node->Advertise<msgs::Request>("~/request");
        this->responseSub = this->node->Subscribe("~/response", &ConfigureVirtualElementsDialogSpawner::OnResponse, this, true);
        firstPreRenderer = false;
        button->setVisible(true);
        // disconnect!
        gazebo::event::Events::DisconnectPreRender(this->connectionPreRender);
    }
}

void ConfigureVirtualElementsDialogSpawner::Load(sdf::ElementPtr _elem)
{
    std::cout << "Gui PLugin start!" << std::endl;
}

/////////////////////////////////////////////////
void ConfigureVirtualElementsDialogSpawner::OnResponse(ConstResponsePtr &_msg)
{
    if (!this->requestMsg || _msg->id() != this->requestMsg->id())
    {
        return;
    }

    gazebo::msgs::Scene sceneMsg;
    if (_msg->has_type() && _msg->type() == sceneMsg.GetTypeName())
    {
        sceneMsg.ParseFromString(_msg->serialized_data());

        for (int i = 0; i < sceneMsg.model_size(); i++)
        {
            // this->entities[sceneMsg.model(i).name()] = sceneMsg.model(i).id();

            for (int j = 0; j < sceneMsg.model(i).link_size(); j++)
            {
                std::cout << sceneMsg.model(i).name() << ", link " << sceneMsg.model(i).link(j).name() << std::endl;
                // this->entities[sceneMsg.model(i).link(j).name()] =
                //     sceneMsg.model(i).link(j).id();

                // for (int k = 0; k < sceneMsg.model(i).link(j).collision_size(); ++k)
                // {
                //     this->entities[sceneMsg.model(i).link(j).collision(k).name()] =
                //         sceneMsg.model(i).link(j).collision(k).id();
                // }
            }
            // gui::Events::modelUpdate(sceneMsg.model(i));
        }
    }
    // delete this->requestMsg;
    // this->requestMsg = NULL;
}

void ConfigureVirtualElementsDialogSpawner::OnRequest(ConstRequestPtr &_msg)
{
    // boost::mutex::scoped_lock lock(*this->receiveMutex);
    // this->requestMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
void ConfigureVirtualElementsDialogSpawner::OnModelMsg(ConstModelPtr &_msg)
{
    // boost::mutex::scoped_lock lock(*this->receiveMutex);
    // this->modelMsgs.push_back(_msg);
    std::cout << _msg->DebugString();
}

/////////////////////////////////////////////////
ConfigureVirtualElementsDialogSpawner::~ConfigureVirtualElementsDialogSpawner()
{
    delete this->requestMsg;
    delete this->button;
}

///////////////////////////////////////////////
void ConfigureVirtualElementsDialogSpawner::OnButton()
{
    // msgs::Model model;
    // model.set_name("plugin_unit_sphere_" + std::to_string(this->counter++));
    // msgs::Set(model.mutable_pose(), ignition::math::Pose3d(0, 0, 1.5, 0, 0, 0));
    // const double mass = 1.0;
    // const double radius = 0.5;
    // msgs::AddSphereLink(model, mass, radius);

    // std::ostringstream newModelStr;
    // newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    //             << msgs::ModelToSDF(model)->ToString("")
    //             << "</sdf>";

    // // Send the model to the gazebo server
    // msgs::Factory msg;
    // msg.set_sdf(newModelStr.str());
    // this->factoryPub->Publish(msg);
    std::cout << "Clickeddd!" << std::endl;

    // TODO
    this->requestPub->WaitForConnection();
    this->requestMsg = gazebo::msgs::CreateRequest("scene_info");
    this->requestPub->Publish(*this->requestMsg);

    ptr->Init();
}