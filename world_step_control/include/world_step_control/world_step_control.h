#ifndef WORLD_STEP_CONTROL_H
#define WORLD_STEP_CONTROL_H

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/world_control.pb.h>
#include <gazebo/transport/transport.hh>

#include "world_step_control/WorldSteps.h"
#include "world_step_control/module_control.h"

/*!
 * \brief Controls Gazebo world step execution
 */
class WorldStepControl
{
	public:
		static constexpr auto NodeName = "world_step_control";

		WorldStepControl() = default;
		~WorldStepControl();

		void Init();
		void Shutdown();

		/*!
		 * \brief Step Simulation
		 * \param steps Number of steps to execute
		 */
		void StepSimulation(uint32_t steps);

		/*!
		 * \brief Set simulation paused state
		 * \param set_paused Should sim be paused or unpaused?
		 */
		void PauseSim(bool set_paused = true);

	private:

		/*!
		 * \brief ROS Node. Manages ROS communication
		 */
		std::unique_ptr<ros::NodeHandle> _rosNode;

		/*!
		 * \brief Callback queue for this simulation
		 */
		ros::CallbackQueue _rQueue;

		/*!
		 * \brief ROS Service to step simulation
		 */
		ros::ServiceServer _rosStepSrv;

		/*!
		 * \brief Gazebo Node. Manages Gazebo communication
		 */
		gazebo::transport::NodePtr _gzNode;

		/*!
		 * \brief Gazebo Publisher. Steps simulation when required
		 */
		gazebo::transport::PublisherPtr _gzWCtrlPub;

		/*!
		 * \brief Handle ROS WorldStep service requests
		 * \param req ROS Request
		 * \param resp ROS Response
		 */
		bool HandleStepService(world_step_control::WorldSteps::Request &req, world_step_control::WorldSteps::Response &resp);
};

#endif // WORLD_STEP_CONTROL_H
