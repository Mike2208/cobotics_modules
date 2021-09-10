/***************************************************************************
 *                           Bifixed_Euler.h                                     *
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

#ifndef Bifixed_EULER_H_
#define Bifixed_EULER_H_

/*!
 * \file Bifixed_Euler.h
 *
 * \author Francisco Naveros
 * \date June 2015
 *
 * This file declares a class which implements an adaptative Euler integration method. This class implement a multi step
 * integration method.
 */

#include "./BiFixedStep.h"



class TimeDrivenNeuronModel;

/*!
 * \class Bifixed_Euler
 *
 * \brief Bifixed_Euler integration methods in CPU
 * 
 * This class abstracts the behavior of a Euler integration method for neurons in a 
 * time-driven spiking neural network.
 * It includes internal model functions which define the behavior of integration methods
 * (initialization, calculate next value, ...).
 *
 * \author Francisco Naveros
 * \date May 2013
 */
template <class Neuron_Model>

class Bifixed_Euler : public BiFixedStep {
	public:

		/*
		* Time driven neuron model
		*/
		Neuron_Model * neuron_model;

		/*!
		 * \brief Constructor with parameters.
		 *
		 * It generates a new Euler object.
		 *
		 * \param NewModel time driven neuron model associated to this integration method.
		 */
		Bifixed_Euler(Neuron_Model * NewModel) :BiFixedStep("Bifixed_Euler"), neuron_model(NewModel){
		}

		/*!
		 * \brief Class destructor.
		 *
		 * It destroys an object of this class.
		 */
		~Bifixed_Euler();


		/*!
		 * \brief It calculate the new neural state variables for a defined elapsed_time.
		 *
		 * It calculate the new neural state variables for a defined elapsed_time.
		 *
		 * \param index for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 * \param NeuronState neuron state variables of one neuron.
		 * \return Retrun if the neuron spike
		 */
		void NextDifferentialEquationValues(int index, float * NeuronState){

			float AuxNeuronState[MAX_VARIABLES];
			float PreviousNeuronState[MAX_VARIABLES];

			if (integrationMethodState[index] == 0){

				memcpy(PreviousNeuronState, NeuronState, sizeof(float)*this->neuron_model->N_NeuronStateVariables);

				this->neuron_model->EvaluateDifferentialEquation(NeuronState, AuxNeuronState, index, elapsedTimeInNeuronModelScale);

				for (int j = 0; j<this->neuron_model->N_DifferentialNeuronState; j++){
					NeuronState[j] += elapsedTimeInNeuronModelScale*AuxNeuronState[j];
				}

				this->neuron_model->EvaluateTimeDependentEquation(NeuronState, index, 0);
				if (NeuronState[0]>startVoltageThreshold){
					integrationMethodState[index] = 1;
					//Restore the neuron model state to a previous state.
					memcpy(NeuronState, PreviousNeuronState, sizeof(float)*this->neuron_model->N_NeuronStateVariables);
				}
				else{
					//Update the last spike time.
					this->neuron_model->GetVectorNeuronState()->AddElapsedTime(index, elapsedTimeInSeconds);

					//Acumulate the membrane potential in a variable
					this->IncrementValidIntegrationVariable(NeuronState[0]);
				}
			}

			if (integrationMethodState[index]>0){
				for (int iteration = 0; iteration<ratioLargerSmallerSteps; iteration++){
					float previous_V = NeuronState[0];

					this->neuron_model->EvaluateDifferentialEquation(NeuronState, AuxNeuronState, index, bifixedElapsedTimeInNeuronModelScale);

					for (int j = 0; j<this->neuron_model->N_DifferentialNeuronState; j++){
						NeuronState[j] += bifixedElapsedTimeInNeuronModelScale*AuxNeuronState[j];
					}

					this->neuron_model->EvaluateTimeDependentEquation(NeuronState, index, 1);

					//Update the last spike time.
					this->neuron_model->GetVectorNeuronState()->AddElapsedTime(index, bifixedElapsedTimeInSeconds);

					this->neuron_model->EvaluateSpikeCondition(previous_V, NeuronState, index, bifixedElapsedTimeInNeuronModelScale);

					//Acumulate the membrane potential in a variable
					this->IncrementValidIntegrationVariable(NeuronState[0]);
					
					if (NeuronState[0]>startVoltageThreshold && integrationMethodState[index] == 1){
						integrationMethodState[index] = 2;
					}
					else if (NeuronState[0]<endVoltageThreshold && integrationMethodState[index] == 2){
						integrationMethodState[index] = 3;
						integrationMethodCounter[index] = N_postBiFixedSteps;
					}
					if (integrationMethodCounter[index]>0 && integrationMethodState[index] == 3){
						integrationMethodCounter[index]--;
						if (integrationMethodCounter[index] == 0){
							integrationMethodState[index] = 0;
						}
					}

				}
				if (integrationMethodState[index] == 1){
					integrationMethodState[index] = 0;
				}
			}
		}


		/*!
		* \brief It calculate the new neural state variables for a defined elapsed_time and all the neurons.
		*
		* It calculate the new neural state variables for a defined elapsed_time and all the neurons.
		*
		* \return Retrun if the neuron spike
		*/
		void NextDifferentialEquationValues(){

			float AuxNeuronState[MAX_VARIABLES];
			float PreviousNeuronState[MAX_VARIABLES];

			for (int index = 0; index < this->neuron_model->State->GetSizeState(); index++){
				float * NeuronState = this->neuron_model->State->GetStateVariableAt(index);

				if (integrationMethodState[index] == 0){

					memcpy(PreviousNeuronState, NeuronState, sizeof(float)*this->neuron_model->N_NeuronStateVariables);

					this->neuron_model->EvaluateDifferentialEquation(NeuronState, AuxNeuronState, index, elapsedTimeInNeuronModelScale);

					for (int j = 0; j<this->neuron_model->N_DifferentialNeuronState; j++){
						NeuronState[j] += elapsedTimeInNeuronModelScale*AuxNeuronState[j];
					}

					this->neuron_model->EvaluateTimeDependentEquation(NeuronState, index, 0);
					if (NeuronState[0]>startVoltageThreshold){
						integrationMethodState[index] = 1;
						//Restore the neuron model state to a previous state.
						memcpy(NeuronState, PreviousNeuronState, sizeof(float)*this->neuron_model->N_NeuronStateVariables);
					}
					else{
						//Update the last spike time.
						this->neuron_model->GetVectorNeuronState()->AddElapsedTime(index, elapsedTimeInSeconds);

						//Acumulate the membrane potential in a variable
						this->IncrementValidIntegrationVariable(NeuronState[0]);
					}
				}

				if (integrationMethodState[index] > 0){
					for (int iteration = 0; iteration < ratioLargerSmallerSteps; iteration++){
						float previous_V = NeuronState[0];

						this->neuron_model->EvaluateDifferentialEquation(NeuronState, AuxNeuronState, index, bifixedElapsedTimeInNeuronModelScale);

						for (int j = 0; j<this->neuron_model->N_DifferentialNeuronState; j++){
							NeuronState[j] += bifixedElapsedTimeInNeuronModelScale*AuxNeuronState[j];
						}

						this->neuron_model->EvaluateTimeDependentEquation(NeuronState, index, 1);

						//Update the last spike time.
						this->neuron_model->GetVectorNeuronState()->AddElapsedTime(index, bifixedElapsedTimeInSeconds);

						this->neuron_model->EvaluateSpikeCondition(previous_V, NeuronState, index, bifixedElapsedTimeInNeuronModelScale);

						//Acumulate the membrane potential in a variable
						this->IncrementValidIntegrationVariable(NeuronState[0]);
						
						if (NeuronState[0]>startVoltageThreshold && integrationMethodState[index] == 1){
							integrationMethodState[index] = 2;
						}
						else if (NeuronState[0] < endVoltageThreshold && integrationMethodState[index] == 2){
							integrationMethodState[index] = 3;
							integrationMethodCounter[index] = N_postBiFixedSteps;
						}
						if (integrationMethodCounter[index]>0 && integrationMethodState[index] == 3){
							integrationMethodCounter[index]--;
							if (integrationMethodCounter[index] == 0){
								integrationMethodState[index] = 0;
							}
						}

					}
					if (integrationMethodState[index] == 1){
						integrationMethodState[index] = 0;
					}
				}
			}
		}


		/*!
		* \brief It calculate the new neural state variables for a defined elapsed_time and all the neurons that require integration.
		*
		* It calculate the new neural state variables for a defined elapsed_time and all the neurons that requre integration.
		*
		* \param integration_required array that sets if a neuron must be integrated (for lethargic neuron models)
		* \return Retrun if the neuron spike
		*/
		void NextDifferentialEquationValues(bool * integration_required, double CurrentTime){

			float AuxNeuronState[MAX_VARIABLES];
			float PreviousNeuronState[MAX_VARIABLES];

			for (int index = 0; index < this->neuron_model->State->GetSizeState(); index++){
				if (integration_required[index] == true){
					float * NeuronState = this->neuron_model->State->GetStateVariableAt(index);

					if (integrationMethodState[index] == 0){

						memcpy(PreviousNeuronState, NeuronState, sizeof(float)*this->neuron_model->N_NeuronStateVariables);

						this->neuron_model->EvaluateDifferentialEquation(NeuronState, AuxNeuronState, index, elapsedTimeInNeuronModelScale);

						for (int j = 0; j<this->neuron_model->N_DifferentialNeuronState; j++){
							NeuronState[j] += elapsedTimeInNeuronModelScale*AuxNeuronState[j];
						}

						this->neuron_model->EvaluateTimeDependentEquation(NeuronState, index, 0);
						if (NeuronState[0]>startVoltageThreshold){
							integrationMethodState[index] = 1;
							//Restore the neuron model state to a previous state.
							memcpy(NeuronState, PreviousNeuronState, sizeof(float)*this->neuron_model->N_NeuronStateVariables);
						}
						else{
							//Update the last spike time.
							this->neuron_model->GetVectorNeuronState()->AddElapsedTime(index, elapsedTimeInSeconds);

							//Acumulate the membrane potential in a variable
							this->IncrementValidIntegrationVariable(NeuronState[0]);
						}
					}

					if (integrationMethodState[index] > 0){
						for (int iteration = 0; iteration < ratioLargerSmallerSteps; iteration++){
							float previous_V = NeuronState[0];

							this->neuron_model->EvaluateDifferentialEquation(NeuronState, AuxNeuronState, index, bifixedElapsedTimeInNeuronModelScale);

							for (int j = 0; j<this->neuron_model->N_DifferentialNeuronState; j++){
								NeuronState[j] += bifixedElapsedTimeInNeuronModelScale*AuxNeuronState[j];
							}

							this->neuron_model->EvaluateTimeDependentEquation(NeuronState, index, 1);

							//Update the last spike time.
							this->neuron_model->GetVectorNeuronState()->AddElapsedTime(index, bifixedElapsedTimeInSeconds);

							this->neuron_model->EvaluateSpikeCondition(previous_V, NeuronState, index, bifixedElapsedTimeInNeuronModelScale);

							//Acumulate the membrane potential in a variable
							this->IncrementValidIntegrationVariable(NeuronState[0]);
							
							if (NeuronState[0]>startVoltageThreshold && integrationMethodState[index] == 1){
								integrationMethodState[index] = 2;
							}
							else if (NeuronState[0] < endVoltageThreshold && integrationMethodState[index] == 2){
								integrationMethodState[index] = 3;
								integrationMethodCounter[index] = N_postBiFixedSteps;
							}
							if (integrationMethodCounter[index]>0 && integrationMethodState[index] == 3){
								integrationMethodCounter[index]--;
								if (integrationMethodCounter[index] == 0){
									integrationMethodState[index] = 0;
								}
							}

						}
						if (integrationMethodState[index] == 1){
							integrationMethodState[index] = 0;
						}
					}

					//Set last update time for the analytic resolution of the differential equations in lethargic models 
					this->neuron_model->State->SetLastUpdateTime(index, CurrentTime);
				}
			}
		}





		/*!
		 * \brief It prints the integration method info.
		 *
		 * It prints the current integration method characteristics.
		 *
		 * \param out The stream where it prints the information.
		 *
		 * \return The stream after the printer.
		 */
		virtual ostream & PrintInfo(ostream & out){
			out << "Integration Method Type: " << this->GetType() << endl;

			return out;
		}


		/*!
		 * \brief It initialize the state of the integration method for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 *
		 * It initialize the state of the integration method for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 *
		 * \param N_neuron number of neurons in the neuron model.
		 * \param inicialization vector with initial values.
		 */
		void InitializeStates(int N_neurons, float * initialization){
			integrationMethodCounter = new int[N_neurons]();
			integrationMethodState = new int[N_neurons]();
		}

		/*!
		 * \brief It reset the state of the integration method for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 *
		 * It reset the state of the integration method for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 *
		 * \param index indicate which neuron must be reseted.
		 */
		void resetState(int index){};

		/*!
		 * \brief It calculates the conductance exponential values for time driven neuron models.
		 *
		 * It calculates the conductance exponential values for time driven neuron models.
		 */
		void Calculate_conductance_exp_values(){
			this->neuron_model->Initialize_conductance_exp_values(this->neuron_model->N_TimeDependentNeuronState, 2);
			//index 0
			this->neuron_model->Calculate_conductance_exp_values(0, elapsedTimeInNeuronModelScale);
			//index 1
			this->neuron_model->Calculate_conductance_exp_values(1, bifixedElapsedTimeInNeuronModelScale);
		}
};

#endif /* Bifixed_EULER_H_ */
