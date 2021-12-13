#include "world_step_control/world_step_control.h"

#include "world_step_control/world_step_control_plugin.h"

#include <assert.h>
#include <gazebo/physics/World.hh>

static constexpr auto GzWorldCtrlTopic = "~/world_control";

WorldStepControl::~WorldStepControl()
{
	this->WorldStepControl::Shutdown();
}

void WorldStepControl::Init()
{	
	// Setup ROS Node
	this->_rosNode.reset(new ros::NodeHandle(world_step_control::GazeboROSNodeName));
	this->_rosNode->setCallbackQueue(&this->_rQueue);

	// Create ROS service to step simulation
	this->_rosStepSrv = this->_rosNode->advertiseService("/gazebo/world_control", &WorldStepControl::HandleStepService, this);

	// Setup Gazebo communication
	this->_gzNode.reset(new gazebo::transport::Node());
	this->_gzNode->Init();

	this->_gzWCtrlPub = this->_gzNode->Advertise<gazebo::msgs::WorldControl>(GzWorldCtrlTopic);

	// Make sure simulation is paused before startup
	//this->PauseSim();
}

void WorldStepControl::Shutdown()
{
	this->_gzWCtrlPub->Fini();
	this->_gzNode->Fini();

	this->_rosNode->shutdown();
	this->_rosNode.reset(nullptr);
}

void WorldStepControl::StepSimulation(uint32_t steps)
{
	assert(this->_gzNode != nullptr);

	gazebo::msgs::WorldControl wCtrl;
	wCtrl.set_multi_step(steps);

	this->_gzNode->Publish(GzWorldCtrlTopic, wCtrl);
}

void WorldStepControl::PauseSim(bool set_paused)
{
	assert(this->_gzNode != nullptr);

	gazebo::msgs::WorldControl msg;
	msg.set_pause(true);

	this->_gzWCtrlPub->Publish(msg);
}

bool WorldStepControl::HandleStepService(world_step_control::WorldSteps::Request &req, world_step_control::WorldSteps::Response &resp)
{
	try
	{
		this->StepSimulation(req.steps);
	}
	catch(const std::exception &e)
	{
		gzerr << "Failed to step simulation: " << e.what() << std::endl;
		return 0;
	}

	resp.executed_steps = req.steps;

	return true;
}
