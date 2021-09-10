#include "osim_ybot_plugin/osim_ybot_plugin.h"

#include <assert.h>
#include <gazebo/physics/Model.hh>
#include <ros/param.h>

static volatile bool waitForDebug = true;

void OSimYBotPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
	this->_model = model;
}

void OSimYBotPlugin::Init()
{
//	if(waitForDebug)
//		ROS_ERROR("WAITING FOR GDB DEBUG!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

//	while(waitForDebug){}

	this->_nh.reset(new ros::NodeHandle());

	// Iterate over parameter and extract topic mapping
	XmlRpc::XmlRpcValue osimJointMapping;
	this->_nh->param(OSimYBotPlugin::JointPosTopicParam, osimJointMapping, {});

	this->_osimJointSubs.clear();
	this->_osimGzJointMap.clear();

	for(const auto &osimJointMap : osimJointMapping)
	{
		const std::string &osimJointName = osimJointMap.first;
		if(!osimJointMap.second.hasMember("gz_name"))
		{
			const auto errMsg = "Osim joint parameter \"" + osimJointName + "\" is missing a \"gz_name\" element. Aborting...";

			gzerr << errMsg << "\n";
			throw std::runtime_error(errMsg);
		}

		const std::string &gzJointName = osimJointMap.second["gz_name"];
		auto gzJoint = this->_model->GetJoint(gzJointName);
		if(!gzJoint)
		{
			const auto errMsg = "Could not find joint with name \"" + gzJointName + "\". Aborting...";

			gzerr << errMsg << "\n";
			throw std::runtime_error(errMsg);
		}

		this->_osimGzJointMap.emplace(osimJointName, GzJointData({gzJoint, {}}));

		for(const auto &stateTopic : { std::pair("pos", &OSimYBotPlugin::ReceiveOSimJointPosDataCallback),
		                               std::pair("vel", &OSimYBotPlugin::ReceiveOSimJointVelDataCallback),
		                               std::pair("acc", &OSimYBotPlugin::ReceiveOSimJointAccDataCallback) })
		{
			if(!osimJointMap.second.hasMember(stateTopic.first))
				continue;

			const std::string &topic = osimJointMap.second[stateTopic.first];
			if(this->_osimJointSubs.find(topic) == this->_osimJointSubs.end())
				this->_osimJointSubs.emplace(topic, ros::Subscriber(this->_nh->subscribe(topic, 100, stateTopic.second, this)));
		}
	}

	this->_worldUpdateConn = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&OSimYBotPlugin::OnWorldUpdateBegin, this));
}

void OSimYBotPlugin::Reset()
{
	this->_osimJointSubs.clear();
	this->_osimGzJointMap.clear();

	this->Init();
}

void OSimYBotPlugin::OnWorldUpdateBegin()
{
	auto mapLock = std::lock_guard(this->_lockJointMaps);

	const auto &curTime = ros::Time::now();

	for(auto &jointMap : this->_osimGzJointMap)
	{
		if(jointMap.second.States.empty())
			continue;

		const auto &stateMap = jointMap.second.States.rbegin();

		// Adjust joint state if new data is available
		if(stateMap->first <= curTime)
		{
			// Add the infinity comparison to find NaN values as a workaround.
			if(!GzJointState::IsNaN(stateMap->second.Pos) &&
			        stateMap->second.Pos >= -std::numeric_limits<double>::infinity())
				jointMap.second.Joint->SetPosition(0, stateMap->second.Pos);

			if(!GzJointState::IsNaN(stateMap->second.Vel) &&
			        stateMap->second.Vel >= -std::numeric_limits<double>::infinity())
				jointMap.second.Joint->SetVelocity(0, stateMap->second.Vel);

			//if(stateMap->second.Acc != GzJointState::NaN)
			//	jointMap.second.Joint->Set(0, stateMap->second.Acc);

			jointMap.second.States.clear();
		}
	}
}

//auto OSimYBotPlugin::LoadMappingFromParam(const std::string &jointTopic, const XmlRpc::XmlRpcValue &paramMap) -> joint_osim_gz_map_t&
//{
//	auto mapLock = std::lock_guard(this->_lockJointMaps);

//	joint_osim_gz_map_t topicJoints;
//	for(const auto &jointOsimGzNames : paramMap)
//	{
//		auto gzJoint = this->_model->GetJoint(jointOsimGzNames.second);
//		if(!gzJoint)
//		{
//			const auto errMsg = "Could not find joint with name \"" + jointTopic + "\". Aborting...";

//			gzerr << errMsg << "\n";
//			throw std::runtime_error(errMsg);
//		}

//		topicJoints.emplace(jointOsimGzNames.first, GzJointData({gzJoint, {}}));
//	}

//	return this->_osimGzJointMaps.emplace_back(std::move(topicJoints));
//}

void OSimYBotPlugin::ReceiveOSimJointPosDataCallback(const edlut_ros_osim::AnalogCompactDelay &msg)
{
	auto mapLock = std::lock_guard(this->_lockJointMaps);

	if(msg.data.size() != msg.names.size())
	{
		const auto errMsg = "Data and name arrays in message \"" + msg.header.frame_id + "\" are of unequal size. Aborting...";

		gzerr << errMsg << "\n";
		throw std::runtime_error(errMsg);
	}

	for(uint i = 0; i < msg.data.size(); ++i)
	{
		const auto &gzJointIt = this->_osimGzJointMap.find(msg.names[i]);
		if(gzJointIt !=  this->_osimGzJointMap.end())
		{
			const auto stateIt = gzJointIt->second.States.find(msg.header.stamp);
			if(stateIt != gzJointIt->second.States.end())
				stateIt->second.Pos = msg.data[i];
			else
				gzJointIt->second.States.emplace(msg.header.stamp, GzJointState({msg.data[i], GzJointState::NaN, GzJointState::NaN}));
		}
	}
}

void OSimYBotPlugin::ReceiveOSimJointVelDataCallback(const edlut_ros_osim::AnalogCompactDelay &msg)
{
	auto mapLock = std::lock_guard(this->_lockJointMaps);

	if(msg.data.size() != msg.names.size())
	{
		const auto errMsg = "Data and name arrays in message \"" + msg.header.frame_id + "\" are of unequal size. Aborting...";

		gzerr << errMsg << "\n";
		throw std::runtime_error(errMsg);
	}

	for(uint i = 0; i < msg.data.size(); ++i)
	{
		const auto &gzJointIt = this->_osimGzJointMap.find(msg.names[i]);
		if(gzJointIt != this->_osimGzJointMap.end())
		{
			const auto stateIt = gzJointIt->second.States.find(msg.header.stamp);
			if(stateIt != gzJointIt->second.States.end())
				stateIt->second.Vel = msg.data[i];
			else
				gzJointIt->second.States.emplace(msg.header.stamp, GzJointState({GzJointState::NaN, msg.data[i], GzJointState::NaN}));
		}
	}
}

void OSimYBotPlugin::ReceiveOSimJointAccDataCallback(const edlut_ros_osim::AnalogCompactDelay &msg)
{
	auto mapLock = std::lock_guard(this->_lockJointMaps);

	if(msg.data.size() != msg.names.size())
	{
		const auto errMsg = "Data and name arrays in message \"" + msg.header.frame_id + "\" are of unequal size. Aborting...";

		gzerr << errMsg << "\n";
		throw std::runtime_error(errMsg);
	}

	for(uint i = 0; i < msg.data.size(); ++i)
	{
		const auto &gzJointIt = this->_osimGzJointMap.find(msg.names[i]);
		if(gzJointIt != this->_osimGzJointMap.end())
		{
			const auto stateIt = gzJointIt->second.States.find(msg.header.stamp);
			if(stateIt != gzJointIt->second.States.end())
				stateIt->second.Acc = msg.data[i];
			else
				gzJointIt->second.States.emplace(msg.header.stamp, GzJointState({GzJointState::NaN, GzJointState::NaN, msg.data[i]}));
		}
	}
}
