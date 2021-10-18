#include "world_step_control/module.h"

#include <ros/console.h>

StepModule::StepModule(const std::string &name)
    : _rNode(),
      _moduleName(name),
      _shutdownRosOnDestroy(true)
{
	this->initServices(name);
}

StepModule::StepModule(const std::string &name, int argc, char *argv[], uint32_t rosOptions)
    : _rNode(StepModule::initRos(name, argc, argv, rosOptions)),
      _moduleName(name),
      _shutdownRosOnDestroy(true)
{
	this->initServices(name);
}

StepModule::~StepModule()
{
	try
	{
		ros::ServiceClient unregSrv = this->_rNode.serviceClient<world_step_control::UnregisterModule>(this->_unregisterSrv);

		world_step_control::UnregisterModule srv;
		srv.request.Name = this->_moduleName;

		unregSrv.call(srv);

		if(this->_shutdownRosOnDestroy && !ros::isShuttingDown())
			ros::shutdown();
	}
	catch(std::exception &e)
	{
		ROS_ERROR("%s", e.what());
	}
}

double StepModule::ParamExecutionTime() const
{
	return this->_paramExecutionTime;
}

void StepModule::Run()
{
	while(!this->_shutdown &&
	      !ros::isShuttingDown())
	{
		this->RunOnce();
	}

	this->_shutdown = false;
}

void StepModule::RunOnce()
{
	// Receive service requests
	ros::spinOnce();

	if(this->_execStep
	        && ros::Time::now() >= this->_execReqData.StartTime)
	{
		//ros::Duration(ros::Time::now() - this->_execReqData.StartTime).sleep();

		auto res = this->ExecuteStep(this->_execReqData.StartTime);
		this->_execStep = false;

		res.ModuleName = this->_moduleName;
		this->_rResultPub.publish(res);
	}
}

bool StepModule::HandleExecutionCall(world_step_control::ModuleExecuteStepRequest &req, world_step_control::ModuleExecuteStepResponse &)
{
	if(this->_execStep)
		ROS_WARN("Module \"%s\" received execution request while an old request was not yet fulfilled", this->_moduleName.c_str());

	this->_execReqData = req;
	this->_execStep = true;

	return true;
}

void StepModule::initServices(const std::string &name)
{
	// Check params
	std::string execSrvName, execResultTopic, regiserServiceName;
	this->_rNode.param<std::string>("exec_srv_name", execSrvName, ExecServicePath + name);
	this->_rNode.param<std::string>("exec_res_topic", execResultTopic, ExecTopicName);
	this->_rNode.param<std::string>("module_register_srv", regiserServiceName, RegisterServiceName);
	this->_rNode.param<std::string>("module_unregister_srv", this->_unregisterSrv, UnregisterServiceName);

	this->_rNode.param<double>("exec_time", this->_paramExecutionTime, 0.1);

	// Make topcis absolute
	if(!execSrvName.empty() && execSrvName.front() != '/')
		execSrvName = this->_rNode.getNamespace() + "/" + execSrvName;

	if(!execResultTopic.empty() && execResultTopic.front() != '/')
		execResultTopic = this->_rNode.getNamespace() + "/" + execResultTopic;

	this->_rResultPub = this->_rNode.advertise<world_step_control::ModuleExecutionResult>(execResultTopic, 10, true);

	this->_rExecCall = this->_rNode.advertiseService(execSrvName, &StepModule::HandleExecutionCall, this);

	ros::ServiceClient regSrv = this->_rNode.serviceClient<world_step_control::RegisterModule>(regiserServiceName);
	if(!regSrv.waitForExistence(ros::Duration(10)))
	{
		const auto errMsg = "Module Control Service not found";
		ROS_ERROR(errMsg);
		throw std::runtime_error(errMsg);
	}

	world_step_control::RegisterModule srv;
	srv.request.Name = name;
	srv.request.Topic = execSrvName;
	srv.request.StartTime = ros::Time(0);
	srv.request.ExecutionTime = ros::Duration(this->_paramExecutionTime);

	if(!regSrv.call(srv))
	{
		const auto errMsg = "Failed to register module \"" + srv.request.Name + "\"";
		ROS_ERROR(errMsg.c_str());
		throw std::runtime_error(errMsg);
	}
}

ros::NodeHandle StepModule::initRos(const std::string &name, int argc, char *argv[], uint32_t options)
{
	if(!ros::isInitialized())
	{
		ros::init(argc, argv, name, options);
	}

	std::cout.flush();

	return ros::NodeHandle();
}
