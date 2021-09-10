#ifndef OSIM_YBOT_PLUGIN_H
#define OSIM_YBOT_PLUGIN_H

#include <edlut_ros_osim/AnalogCompactDelay.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <ros/ros.h>

class OSimYBotPlugin
        : public gazebo::ModelPlugin
{
		struct GzJointState
		{
			static constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

			double Pos = GzJointState::NaN;
			double Vel = GzJointState::NaN;
			double Acc = GzJointState::NaN;

			static constexpr bool IsNaN(const double val)
			{	return val == std::numeric_limits<double>::quiet_NaN() || val == std::numeric_limits<double>::signaling_NaN();	}
		};

		struct GzJointData
		{
			gazebo::physics::JointPtr Joint;

			/*!
			 * \brief Joint State at different times.
			 * TODO: Only store last state and time, as all others will be ignored
			 */
			std::map<ros::Time, GzJointState> States;
		};

	public:		
		/*!
		 * \brief
		 * osim_joint_name:
		 *     - gz_name: gz_joint_name
		 *     - pos: pos_topic_name
		 *     - vel: vel_topic_name
		 *     - acc: acc_topic_name
		 */
		static constexpr auto JointPosTopicParam = "/gazebo/joint_topics";

		void Load(gazebo::physics::ModelPtr model,
		          sdf::ElementPtr sdf) override;

		void Init() override;
		void Reset() override;

	private:
		std::unique_ptr<ros::NodeHandle> _nh;

		gazebo::event::ConnectionPtr _worldUpdateConn;

		/*!
		 * \brief YBot model
		 */
		gazebo::physics::ModelPtr _model;

		/*!
		 * \brief { joint_osim_name: joint_gz_ptr }
		 */
		using joint_osim_gz_map_t  = std::map<std::string, GzJointData>;

		/*!
		 * \brief { subscriber: { joint_osim_name: joint_gz_ptr } }
		 */
		using joint_sub_data_map_t = std::map<std::unique_ptr<ros::Subscriber>, joint_osim_gz_map_t >;

		/*!
		 * \brief Maps ROS Subscribers to Joint OpenSim names and Gazebo ptrs
		 */
		std::map<std::string, ros::Subscriber> _osimJointSubs;

		/*!
		 * \brief Manages write access to _osimGzJointMaps
		 */
		std::mutex _lockJointMaps;

		/*!
		 * \brief Mapping from OSim Joint name to GZ data
		 */
		joint_osim_gz_map_t _osimGzJointMap;

		///*!
		// * \brief Extract OSim joint -> Gz joint mapping from parameter and store it in _osimGzJointMaps
		// * \return Returns reference to generated mapping
		// */
		//joint_osim_gz_map_t &LoadMappingFromParam(const std::string &jointTopic, const XmlRpc::XmlRpcValue &paramMap);

		/*!
		 * \brief Update OSim joints on world update
		 */
		void OnWorldUpdateBegin();

		/*!
		 * \brief Receive current Joint data from OSim
		 * \param msg
		 */
		void ReceiveOSimJointPosDataCallback(const edlut_ros_osim::AnalogCompactDelay &msg);

		/*!
		 * \brief Receive current Joint data from OSim
		 * \param msg
		 */
		void ReceiveOSimJointVelDataCallback(const edlut_ros_osim::AnalogCompactDelay &msg);

		/*!
		 * \brief Receive current Joint data from OSim
		 * \param msg
		 */
		void ReceiveOSimJointAccDataCallback(const edlut_ros_osim::AnalogCompactDelay &msg);
};

GZ_REGISTER_MODEL_PLUGIN(OSimYBotPlugin);

#endif // OSIM_YBOT_PLUGIN_H
