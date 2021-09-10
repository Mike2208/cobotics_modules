/***************************************************************************
 *                           BiFixedStep.h                                 *
 *                           -------------------                           *
 * copyright            : (C) 2015 by Francisco Naveros                    *
 * email                : fnaveros@ugr.es                                  *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef BIFIXEDSTEP_H_
#define BIFIXEDSTEP_H_

/*!
 * \file BiFixedStep.h
 *
 * \author Francisco Naveros
 * \date June 2015
 *
 * This file declares a class which abstracts all bi-fixed step integration methods in CPU.
 */

#include "./IntegrationMethod.h"

class TimeDrivenNeuronModel;


/*!
 * \class BiFixedStep
 *
 * \brief Bi-Fixed step integration methods
 *
 * This class abstracts the behavior of all bi-fixed step integration methods in CPU 
 * for time-driven neural model.
 * It includes internal model functions which define the behavior of integration methods
 * (initialization, calculate next value, ...).
 * This is only a virtual function (an interface) which defines the functions of the
 * inherited classes.
 *
 * \author Francisco Naveros
 * \date June 2015
 */
class BiFixedStep : public IntegrationMethod {
	protected:

		/*!
		 * \brief Ratio between the large and the small integration steps (how many times the large step is larger than the small step).
		*/
		int ratioLargerSmallerSteps;

		/*!
		 * \brief Elapsed time in neuron model scale of the adaptative zone.
		*/
		float bifixedElapsedTimeInNeuronModelScale;

		/*!
		 * \brief Elapsed time in second of the adaptative zone.
		*/
		float bifixedElapsedTimeInSeconds;


		/*!
		 * \brief When the membrane potential reaches this value, the multi-step integration methods change the integration
		 *  step from elapsedTimeInNeuronModelScale to bifixedElapsedTimeInNeuronModelScale.
		*/
		float startVoltageThreshold;

		/*!
		 * \brief When the membrane potential reaches this value, the multi-step integration methods change the integration
		 *  step from bifixedElapsedTimeInNeuronModelScale to ElapsedTimeInNeuronModelScale after timeAfterEndVoltageThreshold in seconds.
		*/
		float endVoltageThreshold;

		/*!
		 * \brief Number of small integration steps executed after the membrane potential reaches the endVoltageThreashold.
		*/
		int N_postBiFixedSteps;

		/*!
		 * \brief Auxiliar values used to know which integration method must be used.
		*/
		int * integrationMethodCounter;

		/*!
		 * \brief Auxiliar values used to know which integration method must be used.
		*/
		int * integrationMethodState;

	public:

		/*!
		 * \brief Constructor with parameters.
		 *
		 * It generates a new FiexedStep object.
		 *
		 * \param integrationMethodType integration method type.
		 */
		BiFixedStep(string integrationMethodType);

		/*!
		 * \brief Class destructor.
		 *
		 * It destroys an object of this class.
		 */
		~BiFixedStep();

		/*!
		 * \brief It calculate the new neural state variables for a defined elapsed_time.
		 *
		 * It calculate the new neural state variables for a defined elapsed_time.
		 *
		 * \param index for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 * \param NeuronState neuron state variables of one neuron.
		 * \return Retrun if the neuron spike
		 */
		virtual void NextDifferentialEquationValues(int index, float * NeuronState) = 0;

		/*!
		* \brief It calculate the new neural state variables for a defined elapsed_time and all the neurons.
		*
		* It calculate the new neural state variables for a defined elapsed_time and all the neurons.
		*
		* \return Retrun if the neuron spike
		*/
		virtual void NextDifferentialEquationValues() = 0;

		/*!
		* \brief It calculate the new neural state variables for a defined elapsed_time and all the neurons that require integration.
		*
		* It calculate the new neural state variables for a defined elapsed_time and all the neurons that requre integration.
		*
		* \param integration_required array that sets if a neuron must be integrated (for lethargic neuron models)
		* \return Retrun if the neuron spike
		*/
		virtual void NextDifferentialEquationValues(bool * integration_required, double CurrentTime) = 0;

		/*!
		 * \brief It prints the integration method info.
		 *
		 * It prints the current integration method characteristics.
		 *
		 * \param out The stream where it prints the information.
		 *
		 * \return The stream after the printer.
		 */
		virtual ostream & PrintInfo(ostream & out) = 0;


		/*!
		 * \brief It initialize the state of the integration method for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 *
		 * It initialize the state of the integration method for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 *
		 * \param N_neuron number of neurons in the neuron model.
		 * \param inicialization vector with initial values.
		 */
		virtual void InitializeStates(int N_neurons, float * inicialization) = 0;


		/*!
		 * \brief It reset the state of the integration method for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 *
		 * It reset the state of the integration method for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 *
		 * \param index indicate which neuron must be reseted.
		 */
		virtual void resetState(int index) = 0;

		/*!
		 * \brief It loads the integration method parameters.
		 *
		 * It loads the integration method parameters from the file that define the parameter of the neuron model.
		 *
		 * \param model time driven neuron model associated to this integration method.
		 * \param Pointer to a neuron description file (*.cfg). At the end of this file must be included 
		 *  the integration method type and its parameters.
		 * \param Currentline line inside the neuron description file where start the description of the integration method parameter. 
		 * \param fileName file name.
		 *
		 * \throw EDLUTFileException If something wrong has happened in the file load.
		 */
		void loadParameter(TimeDrivenNeuronModel * model, FILE *fh, long * Currentline, string fileName);

		/*!
		 * \brief It sets the required parameter in the adaptative integration methods (Bifixed_Euler, Bifixed_RK2, Bifixed_RK4, Bifixed_BDF1 and Bifixed_BDF2).
		 *
		 * It sets the required parameter in the adaptative integration methods (Bifixed_Euler, Bifixed_RK2, Bifixed_RK4, Bifixed_BDF1 and Bifixed_BDF2).
		 *
		 * \param startVoltageThreshold, when the membrane potential reaches this value, the multi-step integration methods change the integration
		 *  step from elapsedTimeInNeuronModelScale to bifixedElapsedTimeInNeuronModelScale.
		 * \param endVoltageThreshold, when the membrane potential reaches this value, the multi-step integration methods change the integration
		 *  step from bifixedElapsedTimeInNeuronModelScale to ElapsedTimeInNeuronModelScale after timeAfterEndVoltageThreshold in seconds.
		 * \param timeAfterEndVoltageThreshold, time in neuron model scale ("seconds" or "miliseconds") that the multi-step integration methods maintain 
		 *  the bifixedElapsedTimeInNeuronModelScale
		 *  after the endVoltageThreshold
		 */
		void SetBiFixedStepParameters(float startVoltageThreshold, float endVoltageThreshold, float timeAfterEndVoltageThreshold);

		/*!
		 * \brief It calculates the conductance exponential values for time driven neuron models.
		 *
		 * It calculates the conductance exponential values for time driven neuron models.
		 */
		virtual void Calculate_conductance_exp_values()=0;
};

#endif /* BIFIXEDSTEP_H_ */
