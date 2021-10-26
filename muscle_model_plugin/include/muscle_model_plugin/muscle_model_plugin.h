#pragma once

#include <gazebo/gazebo.hh>
#include <boost/numeric/odeint.hpp>

#include "muscle_model_plugin/Millard2012Equilibrium.h"

class MuscleModelPlugin
        : public gazebo::ModelPlugin
        
{
	public:
		virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
		virtual void Init();
		//virtual void Reset() {}

	private:
		gazebo::event::ConnectionPtr _onWorldUpdate;

		gazebo::physics::ModelPtr _model;
		std::list<Thelen2003MuscleGazebo> _muscles;

		void OnWorldUpdateBegin();
		
		std::array<double, 6> _stimulus = {0.01,0.01,0.01,0.15,0.15,0.15};
		std::array<double, 12> _states = {0,0,0,0,0,0,0,0,0,0,0,0};

		using stepper_t = decltype(boost::numeric::odeint::make_controlled(1.E-12, 1.E-12, boost::numeric::odeint::runge_kutta_dopri5< std::array<double,12> >()));
		stepper_t _stepper = boost::numeric::odeint::make_controlled(1.E-12, 1.E-12, boost::numeric::odeint::runge_kutta_dopri5< std::array<double,12> >());

		gazebo::common::Time _curTime = 0;

		void IntegrationFcn( const std::array<double,12> &x , std::array<double,12> &dxdt , const double t );
};

double comp_l_bar_BRA(double theta);
