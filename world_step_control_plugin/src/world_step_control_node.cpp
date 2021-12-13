#include "world_step_control/world_step_control.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, WorldStepControl::NodeName);

	WorldStepControl stepCtrl;

	stepCtrl.Init();

	stepCtrl.Shutdown();

	return 0;
}
