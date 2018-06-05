#include <gazebo/rendering/rendering.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/gui/GuiEvents.hh>
// #include <ignition/msgs.hh>
// #include <ignition/transport.hh>
// #include <ignition/math.hh>

using namespace gazebo;

namespace cosima
{
class VirtualSpringVisualSystemPlugin : public SystemPlugin
{
  private:
	rendering::ScenePtr scene;

  private:
	rendering::VisualPtr visual;

	/////////////////////////////////////////////
	/// \brief Destructor
  public:
	virtual ~VirtualSpringVisualSystemPlugin() {}

	/////////////////////////////////////////////
	/// \brief Called after the plugin has been constructed.
  public:
	void Load(int /*_argc*/, char ** /*_argv*/)
	{
		world = 0;
		firstWorld = false;
		this->connections.push_back(
			rendering::Events::ConnectCreateScene(
				boost::bind(&VirtualSpringVisualSystemPlugin::InitScene, this)));

		connectModelUpdate = gui::Events::ConnectModelUpdate(boost::bind(&VirtualSpringVisualSystemPlugin::InitWorldRef, this, _1));
//std::placeholders::_1
		// this->connections.push_back(event::Events::ConnectPreRender(std::bind(&VirtualSpringVisualSystemPlugin::Update, this)));
	}

	/////////////////////////////////////////////
	// \brief Called once after Load
  private:
	void Init() {}

	gazebo::physics::WorldPtr world;
	bool firstWorld;

	void InitWorldRef(const msgs::Model &_msg) {
		if (!world) {
			std::cout << "World not found" << std::endl;
			world = gazebo::physics::get_world("default");
		}
		gui::Events::DisconnectModelUpdate(connectModelUpdate);
	}

	/////////////////////////////////////////////
	/// \brief Called every PreRender event. See the Load function.
	/// Checks flags for operations
	void Update()
	{

		// rendering::VisualPtr vis = scene->

		/** TODO change this to somehow receive always the right name...? */
		
		if (!world) {
			std::cout << "World not found" << std::endl;
			world = gazebo::physics::get_world("default");
		}
		
		if (world && !firstWorld) {
			std::cout << "World found" << std::endl;
			firstWorld = true;
		}
		if (world && world->GetModelCount() >= 2) {
			// physics::ModelPtr s0 = world->GetModel("spring0");
			// physics::ModelPtr s1 = world->GetModel("spring1");
			// if (s0 && s1) {
			// 	// std::cout << "s0 and s1 added" << std::endl;
				
			// 	// physics::LinkPtr ls0 = s0->GetLink("link");
			// 	// physics::LinkPtr ls1 = s1->GetLink("link");
			// 	if (s0->GetJointCount() > 0) {
			// 		std::vector<physics::JointPtr> s0Joints = s0->GetJoints();
			// 		// TODO
			// 		physics::JointPtr springJoint = s0Joints[0];
			// 		if (springJoint) {
			// 			// yay joint!
			// 			gazebo::rendering::VisualPtr v = gazebo::rendering::VisualPtr(new gazebo::rendering::Visual("v", scene));
			// 			v->Load();
			// 			v->SetVisible(true);

			// 			rendering::MovableText *text;
			// 			text->Load("__TEXT_OBJECT__", "text displayed", "Arial", 0.1);
			// 			Ogre::SceneNode *textNode = v->GetSceneNode()->createChildSceneNode("__TEXT_NODE__");
			// 			textNode->attachObject(text);

			// 		}
			// 	} else {
			// 		std::cout << "Not joints :(" << std::endl;
			// 	}

				
			// }
		}

		// for (auto &object : trajectoryObjects)
		// {

		// 		if((object.second).clear)
		// 		{
		// 			clearHandler(object.second);
		// 		}

		// 		else if((object.second).draw)
		// 		{

		// 			drawHandler(object.second,object.first);
		// 		}
		// 		else if((object.second).lifecycle)
		// 		{
		// 			cycleHandler(object.second,object.first);
		// 		}

		// 		if((object.second).del)
		// 		{
		// 			delHandler(object.second,object.first);
		// 		}
		// }
	}

	// /////////////////////////////////////////////
	// /// Clears the line and his clearFlag.
	// private: void clearHandler(struct lineObject &object)
	// {
	// 	(object).line->Clear();
	// 	(object).line->Update();
	// 	(object).clear = 0;
	// }

	// /////////////////////////////////////////////
	// /// Gets the coordinates of the visual and adds it to line object.
	// private: void drawHandler(struct lineObject &object,std::string name)
	// {
	// 	rendering::VisualPtr vis = scene->GetVisual(name);
	// 	if(vis!=nullptr)
	// 	{
	// 		math::Pose pose = vis->GetWorldPose();
	// 		math::Vector3 vec = pose.pos;
	// 		pointAdd((object).line,vec.x,vec.y,vec.z);
	// 	}
	// }

	/////////////////////////////////////////////
	///Gets the scene and creates new visual.
  private:
	void InitScene()
	{
		scene = rendering::get_scene();
		visual = gazebo::rendering::VisualPtr(new gazebo::rendering::Visual("springs", scene));
		visual->Load();
		visual->SetVisible(true);
	}

	// /////////////////////////////////////////////
	// ///Checks if visual is existend.If so, generates new trajectory object.
	// private: void newTrajectory(const ignition::msgs::StringMsg &_req)
	// {
	// 	rendering::VisualPtr vis = scene->GetVisual(_req.data());

	// 	if(vis == nullptr)
	// 	{
	// 		std::cerr << "Error " << _req.data() << " not found" << std::endl;
	// 		return;
	// 	}

	// 	if(trajectoryObjects.find(_req.data()) == trajectoryObjects.end())
	// 	{
	// 		trajectoryObjects[_req.data()] = {getLine(), 1, 0, 0, 1,lcLength,0};
	// 	}
	// }

	// /////////////////////////////////////////////
	// ///Creates new DynamicLines and initializes it.
	// private: rendering::DynamicLines* getLine()
	// {
	// 	rendering::DynamicLines* line = visual->CreateDynamicLine(gazebo::rendering::RENDERING_LINE_STRIP);
	// 	line->setMaterial(color[colorIndex]);
	// 	incColorIndex();
	// 	line->setVisibilityFlags(GZ_VISIBILITY_GUI);
	// 	return line;
	// }

	/// All the event connections.
  private:
	std::vector<event::ConnectionPtr> connections;
	event::ConnectionPtr connectModelUpdate;
};
// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(VirtualSpringVisualSystemPlugin)
} // namespace cosima
