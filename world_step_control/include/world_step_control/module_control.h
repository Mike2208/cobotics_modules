#ifndef MODULE_CONTROL_H
#define MODULE_CONTROL_H

#include <future>
#include <list>
#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <rosgraph_msgs/Clock.h>

#include "world_step_control/ModuleExecutionResult.h"
#include "world_step_control/ModuleExecuteStep.h"
#include "world_step_control/RegisterModule.h"
#include "world_step_control/UnregisterModule.h"

/*!
 * \brief Control Module execution and wait
 */
class ModuleControl
{
		using exec_thread_t = std::future<world_step_control::ModuleExecuteStepResponse>;

		/*!
		 * \brief Struct containing data for running thread
		 */
		struct ModuleExecutionData
		{
			/*!
			 * \brief Time when module execution was initialized
			 */
			ros::Time StartTime;

			/*!
			 * \brief Has execution finished?
			 */
			bool Completed = false;

			ModuleExecutionData(const ros::Time &startTime);
		};

		/*!
		 * \brief Data for individual modules
		 */
		struct ModuleData
		{
			/*!
			 * \brief Module Name
			 */
			std::string Name;

			/*!
			 * \brief Topic to send execution requests to
			 */
			std::string Service;

			/*!
			 * \brief Time when module should finish
			 */
			ros::Time EndTime;

			ModuleData(const std::string &name, const std::string &service, const ros::Time &endTime);
		};

	public:

		static constexpr auto ExecTopicName = "/gazebo/module_execution_result";
		static constexpr auto RegisterServiceName = "/gazebo/module_register";
		static constexpr auto UnregisterServiceName = "/gazebo/module_unregister";

		ModuleControl()  = default;
		~ModuleControl() = default;

		void Init();
		void Shutdown();

		/*!
		 * \brief Wait for all modules to load
		 * \param modules Names of modules to wait for
		 * \param waitTime Amoung of time to wait
		 * \return Returns vector of unregistered modules. Empty if all modules registered
		 */
		const std::vector<std::string> &WaitForModules(std::vector<std::string> &&modules,
		                                               ros::Publisher *clockPub,
		                                               const rosgraph_msgs::Clock &curTime = rosgraph_msgs::Clock(),
		                                               const std::chrono::duration<double> &waitTime = std::chrono::duration<double>::max());

		void UpdateModules(const ros::Time &curTime, ros::Publisher *clockPub);

	private:
		/*!
		 * \brief Prevent multiple threads from accessing this class' memory
		 */
		std::mutex _lock;

		/*!
		 * \brief Registered modules
		 */
		std::list<ModuleData> _modules;

		/*!
		 * \brief Modules to wait for
		 */
		std::vector<std::string> _waitModules;

		/*!
		 * \brief Module queue, sorted by time
		 */
		std::multimap<ros::Time, ModuleData *const> _queue;

		/*!
		 * \brief Contains all running modules
		 */
		std::map<ModuleData *const, ModuleExecutionData> _runningModules;

		/*!
		 * \brief ROS Node
		 */
		std::unique_ptr<ros::NodeHandle> _rNode;

		/*!
		 * \brief Callback Queue
		 */
		ros::CallbackQueue _rCbQueue;

		/*!
		 * \brief ROS Subscriber to receive module execution results
		 */
		ros::Subscriber _rExecResults;

		/*!
		 * \brief Register server
		 */
		ros::ServiceServer _rRegisterSrv;

		/*!
		 * \brief Unregister server
		 */
		ros::ServiceServer _rUnregisterSrv;

		/*!
		 * \brief Run a single Module and wait for completion response
		 * \param pModule Pointer to ModuleData
		 * \param Module start time
		 */
		void RunModule(const ModuleData *pModule, const ros::Time &startTime);

		/*!
		 * \brief Find Module by Name in _modules
		 * \return Returns pointer to module in _modules
		 */
		ModuleData *FindModule(const std::string &name);
		const ModuleData *FindModule(const std::string &name) const;

		/*!
		 * \brief Remove Module From _queue
		 * \param pData Pointer to module that should be removed
		 */
		void RemoveModuleFromQueue(const ModuleData *const pData);

		/*!
		 * \brief Remove Module From _runningModules
		 * \param pData Pointer to module that should be removed
		 */
		void RemoveModuleFromRunning(const ModuleData *const pData);

		/*!
		 * \brief Handle ROS Service RegisterModule request
		 * \param req ROS Request
		 * \param resp ROS Response
		 */
		bool HandleRegisterModule(world_step_control::RegisterModuleRequest &req, world_step_control::RegisterModuleResponse &resp);

		/*!
		 * \brief Handle ROS Service Unregister request
		 * \param req ROS Request
		 * \param resp ROS Response
		 */
		bool HandleUnregisterModule(world_step_control::UnregisterModuleRequest &req, world_step_control::UnregisterModuleResponse &resp);

		/*!
		 * \brief Handle Execution results of Modules
		 * \param msg Exec Result
		 */
		void HandleExecResult(const world_step_control::ModuleExecutionResult &msg);
};

#endif // MODULE_CONTROL_H
