#include "muscle_model_plugin/muscle_model_plugin.h"

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Joint.hh>
#include <sdf/sdf.hh>
#include <math.h>

#include "muscle_model_plugin/Millard2012Equilibrium.h"

//volatile bool waitForDebug = true;

void MuscleModelPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
	//while(waitForDebug){};

	// Create stepper

	this->_model = model;
	this->_onWorldUpdate = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&MuscleModelPlugin::OnWorldUpdateBegin, this));

	this->_muscles = {
	    Thelen2003MuscleGazebo("TRIlong", 798.52, 0.134, 0.1430, 0.2094, 0.1362, 0.0399),
	    Thelen2003MuscleGazebo("TRIlat", 624.3, 0.1138, 0.0980, 0.1571, 0.0702, 0.0399),
	    Thelen2003MuscleGazebo("TRImed", 624.3, 0.1138, 0.0908, 0.1571, 0.0653, 0.0400),
	    Thelen2003MuscleGazebo("BIClong", 624.3, 0.1157, 0.2723, 0, 0.1501, 0.0409),
	    Thelen2003MuscleGazebo("BICshort", 435.56, 0.1321, 0.1923, 0, 0.1473, 0.0404),
	    Thelen2003MuscleGazebo("BRA", 987.26, 0.0858, 0.0535, 0, 0.0979, 0.0399)
	};

	int i = 0;
	for(const auto &muscle : this->_muscles)
	{
		this->_states[2*i + 0] = muscle.q;
		this->_states[2*i + 1] = muscle.l;

		++i;
	}

	this->_curTime = this->_model->GetWorld()->SimTime();

	// Integration function
//	boost::numeric::odeint::integrate_adaptive(this->_stepper,
//	                                           std::bind(&MuscleModelPlugin::IntegrationFcn, this,
//	                                                     std::placeholders::_1, std::placeholders::_2,
//	                                                     std::placeholders::_3),
//	                                           this->_states,
	//	                                           0.0, 0.001, 0.0001);
}

void MuscleModelPlugin::Init()
{
	this->_model->GetJoint("arm_r_elbow_joint")->SetPosition(0,0);
}

double comp_l_bar_TRIlong(double theta)
{
	double p11 = -0.0537 + 0.0; // adding ground offset
	double p12 = -0.0137 + 0.0;

	double p21 = -0.0271 -0.017545; //adding humerus offset
	double p22 = -0.1144 -0.007;

	double p31 = -0.0318 -0.017545;
	double p32 = -0.2264 -0.007;

	double p41 = -0.0174 -0.017545;
	double p42 = -0.2676 -0.007;

	//double p51 = -0.0219 -0.011445; // adding ulna_radius_hand offset
	//double p52 = 0.0105 -0.2974;

	double p51l = -0.0219;
	double p52l = 0.0105;
	// rotate point and add offset to get global point
	double p51g = std::cos(theta) * p51l - std::sin(theta) * p52l -0.011445;
	double p52g = std::sin(theta) * p51l +std::cos(theta) * p52l -0.2974;

	// compute distance between each pair of points that form a direct connection globally
	double d1 = std::sqrt( std::pow((p21 - p11), 2) + std::pow((p22 - p12), 2) );
	double d2 = std::sqrt( std::pow((p31 - p21), 2) + std::pow((p32 - p22), 2) );
	double d3 = std::sqrt( std::pow((p41 - p31), 2) + std::pow((p42 - p32), 2) );
	double d4 = std::sqrt( std::pow((p51g - p41), 2) + std::pow((p52g - p42), 2) );

	double total_d = d1 + d2 + d3 + d4;

	return total_d;
}

double comp_r_TRIlong(double theta)
{

	double p41 = -0.0174 -0.017545;
	double p42 = -0.2676 -0.007;

	double p51l = -0.0219;
	double p52l = 0.0105;

	// rotate point and add offset to get global point
	double p51g = std::cos(theta) * p51l - std::sin(theta) * p52l -0.011445;
	double p52g = std::sin(theta) * p51l +std::cos(theta) * p52l -0.2974;

	double b1 = std::cos(theta) * p51l - std::sin(theta) * p52l;
	double b2 = std::cos(theta) * p51l - std::sin(theta) * p52l;

	double m1 = p51g - p41;
	double m2 = p52g - p42;

	double nb = std::sqrt( std::pow((b1), 2) + std::pow((b2), 2) );
	double nm = std::sqrt( std::pow((m1), 2) + std::pow((m2), 2) );

	double dot_product = m1 * b1 + m2 * b2;
	double cos_phi = dot_product / (nb + nm);
	double sin_phi = std::sqrt(std::pow((1 - cos_phi), 2));

	double r = - nb * sin_phi;

	return r;
}

double comp_l_bar_TRIlat(double theta)
{
	double p11 = -0.0060 -0.017545;
	double p12 = -0.1265 -0.007;

	double p21 = -0.0234 -0.017545; //adding humerus offset
	double p22 = -0.1453 -0.007;

	double p31 = -0.0318 -0.017545;
	double p32 = -0.2264 -0.007;

	double p41 = -0.0174 -0.017545;
	double p42 = -0.2676 -0.007;

	//double p51 = -0.0219 -0.011445; // adding ulna_radius_hand offset
	//double p52 = 0.0105 -0.2974;

	double p51l = -0.0219;
	double p52l = 0.0105;
	// rotate point and add offset to get global point
	double p51g = std::cos(theta) * p51l - std::sin(theta) * p52l -0.011445;
	double p52g = std::sin(theta) * p51l + std::cos(theta) * p52l -0.2974;

	// compute distance between each pair of points that form a direct connection globally
	double d1 = std::sqrt( std::pow((p21 - p11), 2) + std::pow((p22 - p12), 2) );
	double d2 = std::sqrt( std::pow((p31 - p21), 2) + std::pow((p32 - p22), 2) );
	double d3 = std::sqrt( std::pow((p41 - p31), 2) + std::pow((p42 - p32), 2) );
	double d4 = std::sqrt( std::pow((p51g - p41), 2) + std::pow((p52g - p42), 2) );

	double total_d = d1 + d2 + d3 + d4;

	return total_d;
}

double comp_r_TRIlat(double theta)
{

	double p41 = -0.0174 -0.017545;
	double p42 = -0.2676 -0.007;

	double p51l = -0.0219;
	double p52l = 0.0105;

	// rotate point and add offset to get global point
	double p51g = std::cos(theta) * p51l - std::sin(theta) * p52l -0.011445;
	double p52g = std::sin(theta) * p51l +std::cos(theta) * p52l -0.2974;

	double b1 = std::cos(theta) * p51l - std::sin(theta) * p52l;
	double b2 = std::cos(theta) * p51l - std::sin(theta) * p52l;

	double m1 = p51g - p41;
	double m2 = p52g - p42;

	double nb = std::sqrt( std::pow((b1), 2) + std::pow((b2), 2) );
	double nm = std::sqrt( std::pow((m1), 2) + std::pow((m2), 2) );

	double dot_product = m1 * b1 + m2 * b2;
	double cos_phi = dot_product / (nb + nm);
	double sin_phi = std::sqrt(std::pow((1 - cos_phi), 2));

	double r = - nb * sin_phi;

	return r;
}

double comp_l_bar_TRImed(double theta)
{
	double p11 = -0.0084 -0.017545;
	double p12 = -0.1370 -0.007;

	double p21 = -0.0260 -0.017545; //adding humerus offset
	double p22 = -0.1514 -0.007;

	double p31 = -0.0318 -0.017545;
	double p32 = -0.2264 -0.007;

	double p41 = -0.0174 -0.017545;
	double p42 = -0.2676 -0.007;

	//double p51 = -0.0219 -0.011445; // adding ulna_radius_hand offset
	//double p52 = 0.0105 -0.2974;

	double p51l = -0.0219;
	double p52l = 0.0105;
	// rotate point and add offset to get global point
	double p51g = std::cos(theta) * p51l - std::sin(theta) * p52l -0.011445;
	double p52g = std::sin(theta) * p51l + std::cos(theta) * p52l -0.2974;

	// compute distance between each pair of points that form a direct connection globally
	double d1 = std::sqrt( std::pow((p21 - p11), 2) + std::pow((p22 - p12), 2) );
	double d2 = std::sqrt( std::pow((p31 - p21), 2) + std::pow((p32 - p22), 2) );
	double d3 = std::sqrt( std::pow((p41 - p31), 2) + std::pow((p42 - p32), 2) );
	double d4 = std::sqrt( std::pow((p51g - p41), 2) + std::pow((p52g - p42), 2) );

	double total_d = d1 + d2 + d3 + d4;

	return total_d;
}

double comp_r_TRImed(double theta)
{

	double p41 = -0.0174 -0.017545;
	double p42 = -0.2676 -0.007;

	double p51l = -0.0219;
	double p52l = 0.0105;

	// rotate point and add offset to get global point
	double p51g = std::cos(theta) * p51l - std::sin(theta) * p52l -0.011445;
	double p52g = std::sin(theta) * p51l +std::cos(theta) * p52l -0.2974;

	double b1 = std::cos(theta) * p51l - std::sin(theta) * p52l;
	double b2 = std::cos(theta) * p51l - std::sin(theta) * p52l;

	double m1 = p51g - p41;
	double m2 = p52g - p42;

	double nb = std::sqrt( std::pow((b1), 2) + std::pow((b2), 2) );
	double nm = std::sqrt( std::pow((m1), 2) + std::pow((m2), 2) );

	double dot_product = m1 * b1 + m2 * b2;
	double cos_phi = dot_product / (nb + nm);
	double sin_phi = std::sqrt(std::pow((1 - cos_phi), 2));

	double r = - nb * sin_phi;

	return r;
}

double comp_l_bar_BIClong(double theta)
{
	double p11 = 0.0047 + 0.0; // adding ground offset
	double p12 = -0.0123 + 0.0;

	double p21 = -0.0289 + 0.0; //adding humerus offset
	double p22 = -0.0139 + 0.0;

	double p31 = 0.0213 -0.017545;
	double p32 = -0.0179 -0.007;

	double p41 = -0.0238 -0.017545;
	double p42 = -0.0051 -0.007;

	double p51 = 0.0135 - 0.017545;
	double p52 = -0.0283 - 0.007;

	double p61 = 0.0107 - 0.017545;
	double p62 = -0.0744 - 0.007;

	double p71 = 0.0170 - 0.017545;
	double p72 = -0.1213 - 0.007;

	double p81 = 0.0228 - 0.017545;
	double p82 = -0.1754 - 0.007;

	double p91l = 0.0075;
	double p92l = -0.0484;

	// rotate point and add offset to get global point
	double p91g = std::cos(theta) * p91l - std::sin(theta) * p92l -0.011445;
	double p92g = std::sin(theta) * p91l + std::cos(theta) * p92l -0.2974;

	// compute distance between each pair of points that form a direct connection globally
	double d1 = std::sqrt( std::pow((p21 - p11), 2) + std::pow((p22 - p12), 2) );
	double d2 = std::sqrt( std::pow((p31 - p21), 2) + std::pow((p32 - p22), 2) );
	double d3 = std::sqrt( std::pow((p41 - p31), 2) + std::pow((p42 - p32), 2) );
	double d4 = std::sqrt( std::pow((p51 - p41), 2) + std::pow((p52 - p42), 2) );
	double d5 = std::sqrt( std::pow((p61 - p51), 2) + std::pow((p62 - p52), 2) );
	double d6 = std::sqrt( std::pow((p71 - p61), 2) + std::pow((p72 - p62), 2) );
	double d7 = std::sqrt( std::pow((p81 - p71), 2) + std::pow((p82 - p72), 2) );
	double d8 = std::sqrt( std::pow((p91g - p81), 2) + std::pow((p92g - p82), 2) );


	double total_d = d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8;

	return total_d;
}

double comp_r_BIClong(double theta)
{

	double p81 = 0.0228 - 0.017545;
	double p82 = -0.1754 - 0.007;

	double p91l = 0.0075;
	double p92l = -0.0484;

	// rotate point and add offset to get global point
	double p91g = std::cos(theta) * p91l - std::sin(theta) * p92l -0.011445;
	double p92g = std::sin(theta) * p91l +std::cos(theta) * p92l -0.2974;

	double b1 = std::cos(theta) * p91l - std::sin(theta) * p92l;
	double b2 = std::cos(theta) * p91l - std::sin(theta) * p92l;

	double m1 = p91g - p81;
	double m2 = p92g - p82;

	double nb = std::sqrt( std::pow((b1), 2) + std::pow((b2), 2) );
	double nm = std::sqrt( std::pow((m1), 2) + std::pow((m2), 2) );

	double dot_product = m1 * b1 + m2 * b2;
	double cos_phi = dot_product / (nb + nm);
	double sin_phi = std::sqrt(std::pow((1 - cos_phi), 2));

	double r = nb * sin_phi;

	return r;
}

double comp_l_bar_BICshort(double theta)
{
	double p11 = 0.0047 + 0.0; // adding ground offset
	double p12 = -0.0123 + 0.0;

	double p21 = -0.0071 + 0.0; //adding humerus offset
	double p22 = -0.0400 + 0.0;

	double p31 = 0.0112 -0.017545;
	double p32 = -0.0758 -0.007;

	double p41 = -0.0170 -0.017545;
	double p42 = -0.1213 -0.007;

	double p51 = 0.0228 - 0.017545;
	double p52 = -0.1754 - 0.007;

	double p61l = 0.0075;
	double p62l = -0.0484;

	// rotate point and add offset to get global point
	double p61g = std::cos(theta) * p61l - std::sin(theta) * p62l -0.011445;
	double p62g = std::sin(theta) * p61l + std::cos(theta) * p62l -0.2974;

	// compute distance between each pair of points that form a direct connection globally
	double d1 = std::sqrt( std::pow((p21 - p11), 2) + std::pow((p22 - p12), 2) );
	double d2 = std::sqrt( std::pow((p31 - p21), 2) + std::pow((p32 - p22), 2) );
	double d3 = std::sqrt( std::pow((p41 - p31), 2) + std::pow((p42 - p32), 2) );
	double d4 = std::sqrt( std::pow((p51 - p41), 2) + std::pow((p52 - p42), 2) );
	double d5 = std::sqrt( std::pow((p61g - p51), 2) + std::pow((p62g - p52), 2) );



	double total_d = d1 + d2 + d3 + d4 + d5;

	return total_d;
}

double comp_r_BICshort(double theta)
{

	double p51 = 0.0228 - 0.017545;
	double p52 = -0.1754 - 0.007;

	double p61l = 0.0075;
	double p62l = -0.0484;

	// rotate point and add offset to get global point
	double p61g = std::cos(theta) * p61l - std::sin(theta) * p62l -0.011445;
	double p62g = std::sin(theta) * p61l +std::cos(theta) * p62l -0.2974;

	double b1 = std::cos(theta) * p61l - std::sin(theta) * p62l;
	double b2 = std::cos(theta) * p61l - std::sin(theta) * p62l;

	double m1 = p61g - p51;
	double m2 = p62g - p52;

	double nb = std::sqrt( std::pow((b1), 2) + std::pow((b2), 2) );
	double nm = std::sqrt( std::pow((m1), 2) + std::pow((m2), 2) );

	double dot_product = m1 * b1 + m2 * b2;
	double cos_phi = dot_product / (nb + nm);
	double sin_phi = std::sqrt(std::pow((1 - cos_phi), 2));

	double r = nb * sin_phi;

	return r;
}

double comp_l_bar_BRA(double theta)
{
	double p11 = 0.0068 -0.017545; // adding ground offset
	double p12 = -0.1739 -0.007;

	double p21l = -0.0032;
	double p22l = -0.0239;

	// rotate point and add offset to get global point
	double p21g = std::cos(theta) * p21l - std::sin(theta) * p22l -0.011445;
	double p22g = std::sin(theta) * p21l + std::cos(theta) * p22l -0.2974;

	// compute distance between each pair of points that form a direct connection globally
	double d1 = std::sqrt( std::pow((p21g - p11), 2) + std::pow((p22g - p12), 2) );

	double total_d = d1;

	return total_d;
}

double comp_r_BRA(double theta)
{

	double p11 = 0.0068 -0.017545; // adding ground offset
	double p12 = -0.1739 -0.007;

	double p21l = -0.0032;
	double p22l = -0.0239;

	// rotate point and add offset to get global point
	double p21g = std::cos(theta) * p21l - std::sin(theta) * p22l -0.011445;
	double p22g = std::sin(theta) * p21l +std::cos(theta) * p22l -0.2974;

	double b1 = std::cos(theta) * p21l - std::sin(theta) * p22l;
	double b2 = std::cos(theta) * p21l - std::sin(theta) * p22l;

	double m1 = p21g - p11;
	double m2 = p22g - p12;

	double nb = std::sqrt( std::pow((b1), 2) + std::pow((b2), 2) );
	double nm = std::sqrt( std::pow((m1), 2) + std::pow((m2), 2) );

	double dot_product = m1 * b1 + m2 * b2;
	double cos_phi = dot_product / (nb + nm);
	double sin_phi = std::sqrt(std::pow((1 - cos_phi), 2));

	double r = nb * sin_phi;

	return r;
}

void MuscleModelPlugin::IntegrationFcn(const std::array<double, 12> &x, std::array<double, 12> &dxdt, const double t)
{
	using fcn_t = double(*)(double);
	fcn_t fcns[6] = {&comp_l_bar_TRIlong, &comp_l_bar_TRIlat, &comp_l_bar_TRImed, &comp_l_bar_BIClong, &comp_l_bar_BICshort, &comp_l_bar_BRA};

	auto joint = this->_model->GetJoint("arm_r_elbow_joint");
	auto theta = joint->Position(0);

	int i = 0;
	for(auto &muscle : this->_muscles)
	{
		double l_bar = fcns[i](theta);
		Thelen2003MuscleGazebo::ReturnValue ret = muscle.compute_muscle_dynamics(x[2*i + 0], this->_stimulus[i], x[2*i + 1], l_bar);
		dxdt[2*i + 0] = ret.q_dot;
		dxdt[2*i + 1] = ret.lm_dot;

		++i;
	}

}

void MuscleModelPlugin::OnWorldUpdateBegin()
{
	//ignition::math::Pose3d pose = this->_model->WorldPose();
	//this->_model->WorldLinearVel();
	//this->_model->WorldAngularVel();
//	auto joint = this->_model->GetJoint("arm_r_elbow_joint");
//	auto rad = joint->Position(0);

	const auto newTime = this->_model->GetWorld()->SimTime();

	// Integration function
	boost::numeric::odeint::integrate_adaptive(this->_stepper,
	                                           std::bind(&MuscleModelPlugin::IntegrationFcn, this,
	                                                     std::placeholders::_1, std::placeholders::_2,
	                                                     std::placeholders::_3),
	                                           this->_states,
	                                           this->_curTime.Double(), newTime.Double(), 0.0001);

	this->_curTime = newTime;

	using fcn_t = double(*)(double);
	fcn_t fcns[6] = {&comp_r_TRIlong, &comp_r_TRIlat, &comp_r_TRImed, &comp_r_BIClong, &comp_r_BICshort, &comp_r_BRA};


	using fcn_t = double(*)(double);
	fcn_t fcnl[6] = {&comp_l_bar_TRIlong, &comp_l_bar_TRIlat, &comp_l_bar_TRImed, &comp_l_bar_BIClong, &comp_l_bar_BICshort, &comp_l_bar_BRA};

	auto joint = this->_model->GetJoint("arm_r_elbow_joint");
	auto theta = joint->Position(0);

	if(theta < 1.57)
		this->_stimulus = {0.01, 0.01, 0.01, 0.15, 0.15, 0.15};
	else
		this->_stimulus = {0.15, 0.15, 0.15, 0.01, 0.01, 0.01};

	int i = 0;
	double force = 0;
	for(auto &muscle : this->_muscles)
	{
		double l_bar = fcnl[i](theta);
		Thelen2003MuscleGazebo::ReturnValue ret = muscle.compute_muscle_dynamics(this->_states[i * 2 + 0], this->_stimulus[i], this->_states[i*2 + 1], l_bar);
		double r = fcns[i](theta);

		if(ret.f >= -9999999999999.9)
			force = force + r * ret.f;

		++i;
	}


	joint->SetForce(0, force*0.001);
}

GZ_REGISTER_MODEL_PLUGIN(MuscleModelPlugin);
