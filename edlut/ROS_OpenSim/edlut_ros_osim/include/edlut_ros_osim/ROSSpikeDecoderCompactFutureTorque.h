/***************************************************************************
 *                           ROSSpikeDecoderCompactFutureTorque.h          *
 *                           -------------------                           *
 * copyright            : (C) 2016 by Jesus Garrido                        *
 * email                : jesusgarrido@ugr.es                              *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef ROSSPIKEDECODERCOMPACTFUTURETORQUE_H_
#define ROSSPIKEDECODERCOMPACTFUTURETORQUE_H_

#include <queue>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <edlut_ros_osim/Spike.h>
#include <edlut_ros_osim/Spike_group.h>
#include <edlut_ros_osim/AnalogCompact.h>



class spike_comparison {
public:
	bool operator() (const edlut_ros_osim::Spike lsp, const edlut_ros_osim::Spike rsp) const{
		if (lsp.time>rsp.time) {
			return true;
		} else if (lsp.time<rsp.time) {
			return false;
		} else {
			return (lsp.neuron_index > rsp.neuron_index);
		}
	}
};

/*
 * This class defines a spike-to-analog decoder.
 */
class ROSSpikeDecoderCompactFutureTorque {
private:

	// ROS Node Handler
	ros::NodeHandle NodeHandler;

	// Input spike activity suscriber
	ros::Subscriber subscriber;

	// ROS Callback queue
	ros::CallbackQueue CallbackQueue;

	// Queue of spike activity not processed yet
	std::priority_queue<edlut_ros_osim::Spike, std::vector<edlut_ros_osim::Spike>, spike_comparison > activity_queue;

	// Indexes of the neurons
	std::vector<unsigned int> min_neuron_index_pos;

	std::vector<unsigned int> max_neuron_index_pos;

	std::vector<unsigned int> min_neuron_index_neg;

	std::vector<unsigned int> max_neuron_index_neg;

	// Tau time constant
	double tau_time_constant;

	// Increment with each spike
	std::vector<double> spike_increment_pos;

	std::vector<double> spike_increment_neg;

	// Value of the output variables
	std::vector<double> output_var;

	// Torque buffer. Future torque commands are computed.
	std::vector<edlut_ros_osim::AnalogCompact> torque_buffer;

	// Last time when the decoder has been updated
	double last_time;

	// Time step to compute torque
	double time_step;

	// Time of the last spike received
	double last_spike_time;

	// Joint names
	std::vector<std::string> joint_list;

	// Callback function for reading input activity
	void SpikeCallback(const edlut_ros_osim::Spike_group::ConstPtr& msg);

	// Callback function for reading input activity
	//void SpikeCallback(const edlut_ros_osim::Spike::ConstPtr& msg);

public:

	/*
	 * This function initializes a spike decoder taking the activity from the specified ros topic.
	 */
	ROSSpikeDecoderCompactFutureTorque(std::string input_topic_name,
			std::vector<int> min_neuron_index_pos,
			std::vector<int> max_neuron_index_pos,
			std::vector<int> min_neuron_index_neg,
			std::vector<int> max_neuron_index_neg,
			double tau_time_constant,
			std::vector<double> spike_increment_pos,
			std::vector<double> spike_increment_neg,
			double time_step,
			std::vector<std::string> joint_list);

	/*
	 * Update the output variables with the current time and the output activity and return the output variables
	 */
	std::vector<edlut_ros_osim::AnalogCompact> UpdateDecoder(double end_time);

	/*
	 * Destructor of the class
	 */
	virtual ~ROSSpikeDecoderCompactFutureTorque();

};

#endif /* ROSSPIKEDECODERCOMPACTFUTURETORQUE_H_ */
