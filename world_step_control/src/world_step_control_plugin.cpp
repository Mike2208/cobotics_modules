#include "world_step_control/world_step_control_plugin.h"

#include <assert.h>
#include <gazebo/physics/physics.hh>
#include <rosgraph_msgs/Clock.h>

// Register System plugin
GZ_REGISTER_SYSTEM_PLUGIN(WorldStepControlPlugin);

void WorldStepControlPlugin::Load(int _argc, char **_argv)
{
	this->_conWorldCreated = gazebo::event::Events::ConnectWorldCreated(  std::bind(&WorldStepControlPlugin::OnWorldCreated, this, std::placeholders::_1));
	this->_conWorldStepEnd = gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&WorldStepControlPlugin::OnWorldStepEnd, this));
}

void WorldStepControlPlugin::Init()
{
	// Initialize ros, if it has not already been initialized.
	// Taken from http://gazebosim.org/tutorials?tut=guided_i6&cat=
	if (!ros::isInitialized())
	{
		int ros_argc = 0;
		char **ros_argv = NULL;
		ros::init(ros_argc, ros_argv, GazeboROSNodeName,
		          ros::init_options::NoSigintHandler);
	}

	// Get list of modules that must be retrieved before starting
	auto rn = ros::NodeHandle(WorldStepControlPlugin::GazeboROSNodeName);
	rn.param("modules", this->_modules, decltype(this->_modules)());

	this->_rworldClkPub = rn.advertise<rosgraph_msgs::Clock>("/clock", 1);

	this->WorldStepControl::Init();
	this->ModuleControl::Init();
}

void WorldStepControlPlugin::Reset()
{
	this->ModuleControl::Shutdown();
	this->WorldStepControl::Shutdown();

	this->WorldStepControl::Init();
	this->ModuleControl::Init();
}

void WorldStepControlPlugin::OnWorldCreated(std::string worldName)
{
	this->_world = gazebo::physics::get_world(worldName);

	// Wait for modules to register
	const auto &missingModules = this->ModuleControl::WaitForModules(std::vector(this->_modules),
	                                                                 &(this->_rworldClkPub),
	                                                                 WorldStepControlPlugin::GetClkTime(),
	                                                                 std::chrono::duration<double>::max());
	if(!missingModules.empty())
	{
		ROS_WARN("Not all modules were initialized:");
		for(const auto missingModuleName : missingModules)
		{
			ROS_WARN("Failed to initialize module \"%s\"", missingModuleName.c_str());
		}

		throw std::runtime_error("Failed to initialize all modules");
	}

	ROS_ERROR("All Modules initialized");
}

void WorldStepControlPlugin::OnWorldStepEnd()
{
	const rosgraph_msgs::Clock clkMsg = WorldStepControlPlugin::GetClkTime();

	//this->_rworldClkPub.publish(clkMsg);

	this->ModuleControl::UpdateModules(clkMsg.clock, &(this->_rworldClkPub));
}

rosgraph_msgs::Clock WorldStepControlPlugin::GetClkTime()
{
	rosgraph_msgs::Clock clkMsg;
	{
		const auto gzTime = this->_world->SimTime();
		clkMsg.clock.sec = gzTime.sec;
		clkMsg.clock.nsec = gzTime.nsec;
	}

	return clkMsg;
}
