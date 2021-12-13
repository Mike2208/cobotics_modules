#include "world_step_control/module_control.h"

#include "world_step_control/constants.h"

#include <assert.h>
#include <chrono>
#include <mutex>

static constexpr std::chrono::duration WaitTime = std::chrono::minutes(10);

ModuleControl::ModuleExecutionData::ModuleExecutionData(const ros::Time &startTime)
    : StartTime(startTime)
{}

ModuleControl::ModuleData::ModuleData(const std::string &name, const std::string &service, const ros::Time &endTime)
    : Name(name),
      Service(service),
      EndTime(endTime)
{}

void ModuleControl::Init()
{
	this->_rCbQueue.clear();
	this->_rNode.reset(new ros::NodeHandle(world_step_control::GazeboROSNodeName));
	this->_rNode->setCallbackQueue(&this->_rCbQueue);

	this->_rExecResults = this->_rNode->subscribe(ModuleControl::ExecTopicName, 100, &ModuleControl::HandleExecResult, this);

	this->_rUnregisterSrv = this->_rNode->advertiseService(ModuleControl::UnregisterServiceName, &ModuleControl::HandleUnregisterModule, this);

	// Only allow registration once system is waiting for modules
	//this->_rRegisterSrv = this->_rNode->advertiseService(ModuleControl::RegisterServiceName, &ModuleControl::HandleRegisterModule, this);
}

void ModuleControl::Shutdown()
{
	this->_rExecResults.shutdown();
	this->_rRegisterSrv.shutdown();
	this->_rUnregisterSrv.shutdown();

	this->_rNode.reset();

	this->_queue.clear();
	this->_runningModules.clear();
	this->_modules.clear();
}

const std::vector<std::string> &ModuleControl::WaitForModules(std::vector<std::string> &&modules,
                                                              ros::Publisher *clockPub, const rosgraph_msgs::Clock &curTime,
                                                              const std::chrono::duration<double> &waitTime)
{
	this->_waitModules = std::move(modules);

	this->_rRegisterSrv = this->_rNode->advertiseService(ModuleControl::RegisterServiceName, &ModuleControl::HandleRegisterModule, this);

	const auto startTime = std::chrono::steady_clock().now();
	do
	{
		// Handle incoming registration messages. Will update _waitModules
		this->_rCbQueue.callAvailable(ros::WallDuration(0.04));
		//ros::spinOnce();

		// Publish current clock time
		if(clockPub)
			clockPub->publish(curTime);

		// Sleep for 100ms
		usleep(1000*100);
	}
	while(!this->_waitModules.empty() &&
	      std::chrono::steady_clock().now() - startTime < waitTime &&
	      !ros::isShuttingDown());

	return this->_waitModules;
}

void ModuleControl::UpdateModules(const ros::Time &curTime, ros::Publisher *clockPub)
{
	// Wait for running modules
	{
		const auto realTimeWaitEnd = std::chrono::system_clock::now() + WaitTime;

		auto runModIt = this->_runningModules.begin();
		while(runModIt != this->_runningModules.end())
		{
			if(runModIt->first->EndTime <= curTime)
			{
				// Wait for module to complete
				while(!runModIt->second.Completed &&
				      std::chrono::system_clock::now() <= realTimeWaitEnd &&
				      !ros::isShuttingDown())
				{
					if(clockPub)
					{
						rosgraph_msgs::Clock clk;
						clk.clock = curTime;
						clockPub->publish(clk);
					}

					this->_rCbQueue.callAvailable(ros::WallDuration(0.04));
					//ros::spinOnce();
					usleep(100);
				}

				if(!runModIt->second.Completed)
				{
					const auto errMsg = "Module \"" + runModIt->first->Name + "\" failed to complete within alloted time";
					throw std::runtime_error(errMsg);
				}

				runModIt = this->_runningModules.erase(runModIt);
			}
			else
				++runModIt;
		}
	}

	// Order execution of modules in queue
	{
		std::scoped_lock lockMem(this->_lock);

		auto queueIt = this->_queue.begin();
		while(queueIt != this->_queue.end()
		      && queueIt->first <= curTime)
		{
			auto runMod = decltype(this->_runningModules)::value_type(queueIt->second, ModuleExecutionData(curTime));
			const auto res = this->_runningModules.insert(runMod);
			if(!res.second)
			{
				if(!res.first->second.Completed)
				{
					const auto errMsg = "Module \"" + res.first->first->Name + "\" did not complete even though new execution is schedules";
					throw std::runtime_error(errMsg);
				}

				res.first->second = runMod.second;
			}

			// Send execution command to module
			ros::ServiceClient srvClient = this->_rNode->serviceClient<world_step_control::ModuleExecuteStep>(queueIt->second->Service);
			world_step_control::ModuleExecuteStep srv;
			srv.request.StartTime = res.first->second.StartTime;

			if(!srvClient.call(srv))
			{
				ROS_ERROR("Failed to contact module \"%s\" at service \"%s\"", queueIt->second->Name.c_str(), queueIt->second->Service.c_str());
				throw std::runtime_error("Failed to contact module");
			}

			// Adjust execution period if actual start time does not match recorded one
			res.first->first->EndTime += (curTime - queueIt->first);

			queueIt = this->_queue.erase(queueIt);
		}
	}
}

void ModuleControl::RunModule(const ModuleData *pModule, const ros::Time &startTime)
{
	ros::ServiceClient execClient = this->_rNode->serviceClient<world_step_control::ModuleExecuteStep>(pModule->Service);

	world_step_control::ModuleExecuteStep req;
	req.request.StartTime = startTime;

	if(!execClient.call(req))
	{
		const auto errMsg = "Could not execute module \"" + pModule->Name + "\" with service \"" + pModule->Service + "\"";
		//gzerr << errMsg;
		throw std::runtime_error(errMsg);
	}
}

ModuleControl::ModuleData *ModuleControl::FindModule(const std::string &name)
{	return const_cast<ModuleData*>(const_cast<const ModuleControl*>(this)->FindModule(name));	}

const ModuleControl::ModuleData *ModuleControl::FindModule(const std::string &name) const
{
	for(auto &module : this->_modules)
	{
		if(module.Name == name)
			return &module;
	}

	return nullptr;
}

void ModuleControl::RemoveModuleFromQueue(const ModuleControl::ModuleData *const pData)
{
	auto moduleIt = this->_queue.begin();
	while(moduleIt != this->_queue.end())
	{
		if(moduleIt->second == pData)
			moduleIt = this->_queue.erase(moduleIt);
		else
			++moduleIt;
	}
}

void ModuleControl::RemoveModuleFromRunning(const ModuleControl::ModuleData *const pData)
{
	this->_runningModules.erase(const_cast<ModuleControl::ModuleData *const>(pData));
}

bool ModuleControl::HandleRegisterModule(world_step_control::RegisterModuleRequest &req, world_step_control::RegisterModuleResponse &resp)
{
	std::scoped_lock lockMem(this->_lock);

	// If req.StartTime lies in the past, set StartTime to now
	ros::Time insertTime = ros::Time::now();
	if(insertTime <= req.StartTime)
		insertTime = req.StartTime;

	// Check if module with existing name exists
	ModuleData *pData = this->FindModule(req.Name);
	if(pData != nullptr)
	{
		//gzwarn << "Module with name \"" << pData->Name << "\" already registered (Topic: \"" << pData->Service << "\"). Overrriding...\n";

		// Erase stored start times from queue for old module with same name
		this->RemoveModuleFromQueue(pData);
		*pData = ModuleData(req.Name, req.Topic, insertTime + req.ExecutionTime);
	}
	else
	{
		this->_modules.push_back(ModuleData(req.Name, req.Topic, insertTime + req.ExecutionTime));
		pData = &(this->_modules.back());
	}

	// Check if new module is in list of modules to wait for
	{
		auto waitModuleIt = std::find(this->_waitModules.begin(), this->_waitModules.end(), req.Name);
		if(waitModuleIt != this->_waitModules.end())
			this->_waitModules.erase(waitModuleIt);
	}

	// Insert new module into queue at correct position
	const auto res = this->_queue.emplace(insertTime, pData);
	assert(res->second);

	resp.RegisteredStartTime = insertTime;

	return true;
}

bool ModuleControl::HandleUnregisterModule(world_step_control::UnregisterModuleRequest &req, world_step_control::UnregisterModuleResponse &resp)
{
	std::scoped_lock lockMem(this->_lock);

	for(auto moduleIt = this->_modules.begin(); moduleIt != this->_modules.end(); ++moduleIt)
	{
		if(moduleIt->Name == req.Name)
		{
			// Erase module from _queue, _runningThreads, and _modules
			this->RemoveModuleFromQueue(&(*moduleIt));
			this->RemoveModuleFromRunning(&(*moduleIt));
			this->_modules.erase(moduleIt);

			resp.ModuleFound = true;
			return true;
		}
	}

	resp.ModuleFound = false;
	return true;
}

void ModuleControl::HandleExecResult(const world_step_control::ModuleExecutionResult &msg)
{
	std::scoped_lock lockMem(this->_lock);

	const auto runModuleIt = this->_runningModules.find(this->FindModule(msg.ModuleName));
	if(runModuleIt == this->_runningModules.end())
	{
		//gzwarn << "Received result for module \"" << msg.ModuleName << "\", which is not supposed to be running\n";
		return;
	}

	// Update start and end times of current module
	auto nextStartTime = runModuleIt->first->EndTime + msg.PauseTime;
	if(nextStartTime < ros::Time::now())
		nextStartTime = ros::Time::now();

	runModuleIt->first->EndTime = nextStartTime + msg.ExecutionTime;

	const auto res = this->_queue.emplace(nextStartTime, runModuleIt->first);
	assert(res->second);

	runModuleIt->second.Completed = true;
}
