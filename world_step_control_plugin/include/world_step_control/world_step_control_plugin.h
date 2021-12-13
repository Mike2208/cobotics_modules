#ifndef WORLD_STEP_CONTROL_PLUGIN_H
#define WORLD_STEP_CONTROL_PLUGIN_H

#include "world_step_control/world_step_control.h"
#include "world_step_control/module_control.h"
#include "world_step_control/constants.h"

class WorldStepControlPlugin
        : public gazebo::WorldPlugin,
          protected WorldStepControl,
          protected ModuleControl
{
	public:
		WorldStepControlPlugin() = default;
		virtual ~WorldStepControlPlugin() = default;

		//void Load(int _argc = 0, char **_argv = nullptr) override;
		void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override;

		void Init() override;
		void Reset() override;

	private:
		gazebo::event::ConnectionPtr _conWorldCreated;
		gazebo::event::ConnectionPtr _conWorldStepEnd;

		gazebo::physics::WorldPtr _world;

		ros::Publisher _rworldClkPub;

		/*!
		 * \brief List of modules. WorldStepControlPlugin will wait for them
		 * before allowing Gazebo to start
		 */
		std::vector<std::string> _modules;

		void OnWorldCreated(std::string worldName);
		void OnWorldStepEnd();

		rosgraph_msgs::Clock GetClkTime();
};

#endif // WORLD_STEP_CONTROL_PLUGIN_H
