#include "muscle_model_plugin/Millard2012Equilibrium.h"
#include "muscle_model_plugin/muscle_model_plugin.h"
#include <iostream>

int main(int argc, char **argv)
{
	Thelen2003MuscleGazebo muscle("BRA", 987, 0.0858, 0.0535, 0.0, 0.0979, 0.0399);
	std::cout << muscle.compute_muscle_equilibrium(0.0979, 0.0399, 0.1405) << std::endl;

	Thelen2003MuscleGazebo::ReturnValue ret;
	ret = muscle.compute_muscle_dynamics(0.0399, 0.2, 0.0979, 0.1405);
	std::cout << ret.f << std::endl << ret.q_dot << std::endl << ret.lm_dot << std::endl;

	std::cout << comp_l_bar_BRA(0.1) << std::endl;
	while(1)
	{}
}
