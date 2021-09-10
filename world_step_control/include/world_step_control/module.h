#ifndef MODULE_H
#define MODULE_H

#include <ros/ros.h>

#include "world_step_control/module_control.h"
#include "world_step_control/ModuleExecuteStep.h"
#include "world_step_control/ModuleExecutionResult.h"

/*!
 * \brief Controls module stepping
 */
class StepModule
{
	public:
		using ModuleExecutionResult = world_step_control::ModuleExecutionResult;

		static constexpr auto ExecServicePath = "exec";
		static constexpr auto ExecTopicName   = ModuleControl::ExecTopicName;
		static constexpr auto RegisterServiceName = ModuleControl::RegisterServiceName;
		static constexpr auto UnregisterServiceName = ModuleControl::UnregisterServiceName;

		StepModule(const std::string &name);
		StepModule(const std::string &name, int argc, char *argv[], uint32_t rosOptions);
		virtual ~StepModule();

		double ParamExecutionTime() const;

		/*!
		 * \brief Wait for step cmds
		 */
		void Run();

		/*!
		 * \brief Run ros::spinOnce and handle incoming messages
		 */
		void RunOnce();

		/*!
		 * \brief User function that is defined at each step
		 * \param time Start time
		 * \return Returns execution time of next step
		 */
		virtual ModuleExecutionResult ExecuteStep(const ros::Time &time) = 0;

	private:
		/*!
		 * \brief ROS Node Handle
		 */
		ros::NodeHandle _rNode;

		/*!
		 * \brief ROS Publisher for execution results
		 */
		ros::Publisher _rResultPub;

		/*!
		 * \brief ROS Service Server. Receives execution cmds
		 */
		ros::ServiceServer _rExecCall;

		/*!
		 * \brief Name of module
		 */
		std::string _moduleName;

		/*!
		 * \brief Topic to which to publish execution results
		 */
		std::string _resultPubTopic;

		/*!
		 * \brief Name of unregistration service
		 */
		std::string _unregisterSrv;

		/*!
		 * \brief Execution Time set by ROS params
		 */
		double _paramExecutionTime;

		/*!
		 * \brief Flag. Is set to true when an service msg is reveived by _rExecCall
		 */
		volatile bool _execStep = false;

		/*!
		 * \brief Should ROS be shutdown on class destroy
		 */
		bool _shutdownRosOnDestroy = false;

		/*!
		 * \brief Last Execution Request data
		 */
		world_step_control::ModuleExecuteStepRequest _execReqData;

		/*!
		 * \brief Flag. Set to true to stop potentially running run() cmd
		 */
		volatile bool _shutdown = false;

		bool HandleExecutionCall(world_step_control::ModuleExecuteStepRequest  &req,
		                         world_step_control::ModuleExecuteStepResponse &res);

		void initServices(const std::string &name);

		static ros::NodeHandle initRos(const std::string &name, int argc, char *argv[], uint32_t options = 0);
};

#endif // MODULE_H
