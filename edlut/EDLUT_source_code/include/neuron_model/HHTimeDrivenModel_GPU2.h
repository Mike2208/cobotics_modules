/***************************************************************************
 *                          IzhikevichTimeDrivenModel_GPU2.h                 *
 *                           -------------------                           *
 * copyright            : (C) 2012 by Francisco Naveros                    *
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

#ifndef HHTIMEDRIVENMODEL_GPU2_H_
#define HHTIMEDRIVENMODEL_GPU2_H_

/*!
 * \file LIFTimeDrivenModel_GPU.h
 *
 * \author Francisco Naveros
 * \date November 2012
 *
 * This file declares a class which abstracts a Leaky Integrate-And-Fire neuron model with one 
 * differential equation and two time dependent equations (conductances). This model is
 * implemented in GPU.
 */

#include "./TimeDrivenNeuronModel_GPU2.h"
#include "../integration_method/IntegrationMethod_GPU2.h"


//Library for CUDA
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

/*!
 * \class LIFTimeDrivenModel_1_2_GPU
 *
 *
 * \brief Leaky Integrate-And-Fire Time-Driven neuron model with a membrane potential and
 * two conductances. This model is implemented in GPU.
 *
 * This class abstracts the behavior of a neuron in a time-driven spiking neural network.
 * It includes internal model functions which define the behavior of the model
 * (initialization, update of the state, synapses effect, next firing prediction...).
 * This is only a virtual function (an interface) which defines the functions of the
 * inherited classes.
 *
 * \author Francisco Naveros
 * \date May 2013
 */

class HHTimeDrivenModel_GPU2 : public TimeDrivenNeuronModel_GPU2 {
	public:
		/*!
		 * \brief Excitatory reversal potential in mV units
		 */
		const float eexc;

		/*!
		 * \brief Inhibitory reversal potential in mV units
		 */
		const float einh;

		/*!
		 * \brief Resting potential in mV units
		 */
		const float erest;

		/*!
		 * \brief Resting conductance in nS units
		 */
		const float grest;

		/*!
		 * \brief Membrane capacitance in pF units
		 */
		const float cm;
		const float inv_cm;

		/*!
		 * \brief Virtual firing threshold in mV units
		 */
		const float vthr;

		/*!
		 * \brief AMPA receptor time constant in ms units
		 */
		const float texc;
		const float inv_texc;

		/*!
		 * \brief GABA receptor time constant in ms units
		 */
		const float tinh;
		const float inv_tinh;

		/*!
		* \brief Maximum value of sodium conductance in nS units
		*/
		const float gNa;
		
		/*!
		* \brief Maximum value of potasium conductance in nS units
		*/
		const float gKd;
		
		/*!
		* \brief Sodium potential in mV units
		*/
		const float ENa;
		
		/*!
		* \brief Potasium potential in mV units
		*/
		const float EK;
		
		/*!
		* \brief Efective threshold potential in mV units
		*/
		const float VT;




		/*!
		 * \brief Number of state variables for each cell.
		*/
		static const int N_NeuronStateVariables=6;

		/*!
		 * \brief Number of state variables which are calculate with a differential equation for each cell.
		*/
		static const int N_DifferentialNeuronState=4;

		/*!
		 * \brief Number of state variables which are calculate with a time dependent equation for each cell.
		*/
		static const int N_TimeDependentNeuronState=2;

		
		/*!
		 * \brief constructor with parameters.
		 *
		 * It generates a new neuron model object.
		 *
		 * \param Eexc eexc.
		 * \param Einh einh.
		 * \param Erest erest.
		 * \param Vthr vthr.
		 * \param Cm cm.
		 * \param Texc texc.
		 * \param Tinh tinh.
		 * \param Tref tref.
		 * \param Grest grest.
		 * \param integrationName integration method type.
		 * \param N_neurons number of neurons.
		 * \param Total_N_thread total number of CUDA thread.
		 * \param Buffer_GPU Gpu auxiliar memory.
		 *
		 */
		__device__ HHTimeDrivenModel_GPU2(float new_eexc, float new_einh, float new_erest, float new_grest, float new_cm, float new_vthr, float new_texc, 
				float new_tinh, float new_gNa, float new_gKd, float new_ENa, float new_EK, float new_VT,
				char const* integrationName, int N_neurons, void ** Buffer_GPU):TimeDrivenNeuronModel_GPU2(MilisecondScale_GPU),
				eexc(new_eexc), einh(new_einh), erest(new_erest), grest(new_grest), cm(new_cm), inv_cm(1.0f/new_cm), vthr(new_vthr), texc(new_texc), 
				inv_texc(1.0f/new_texc), tinh(new_tinh), inv_tinh(1.0f/new_tinh), gNa(new_gNa),	gKd(new_gKd), ENa(new_ENa), EK(new_EK),
				VT(new_VT){
			loadIntegrationMethod_GPU2(integrationName, Buffer_GPU);

			integrationMethod_GPU2->Calculate_conductance_exp_values();	
		}

		/*!
		 * \brief Class destructor.
		 *
		 * It destroys an object of this class.
		 */
		__device__ virtual ~HHTimeDrivenModel_GPU2(){
			delete integrationMethod_GPU2;
		}


		/*!
		 * \brief Update the neuron state variables.
		 *
		 * It updates the neuron state variables.
		 *
		 * \param index The cell index inside the StateGPU. 
		 * \param AuxStateGPU Auxiliary incremental conductance vector.
		 * \param StateGPU Neural state variables.
		 * \param LastUpdateGPU Last update time
		 * \param LastSpikeTimeGPU Last spike time
		 * \param InternalSpikeGPU In this vector is stored if a neuron must generate an output spike.
		 * \param SizeStates Number of neurons
		 * \param CurrentTime Current time.
		 *
		 * \return True if an output spike have been fired. False in other case.
		 */
		//__device__ void UpdateState(double CurrentTime)
		//{
		//	int index = blockIdx.x * blockDim.x + threadIdx.x;
		//	while (index<vectorNeuronState_GPU2->SizeStates){
		//		vectorNeuronState_GPU2->VectorNeuronStates_GPU[(this->N_DifferentialNeuronState)*vectorNeuronState_GPU2->SizeStates + index]+=vectorNeuronState_GPU2->AuxStateGPU[index];
		//		vectorNeuronState_GPU2->VectorNeuronStates_GPU[(this->N_DifferentialNeuronState+1)*vectorNeuronState_GPU2->SizeStates + index]+=vectorNeuronState_GPU2->AuxStateGPU[vectorNeuronState_GPU2->SizeStates + index];

		//		this->integrationMethod_GPU2->NextDifferentialEquationValues(index, vectorNeuronState_GPU2->SizeStates, vectorNeuronState_GPU2->VectorNeuronStates_GPU);

		//		vectorNeuronState_GPU2->LastUpdateGPU[index]=CurrentTime;

		//		this->CheckValidIntegration(index);

		//		index+=blockDim.x*gridDim.x;
		//	}
		//} 

		__device__ void UpdateState(double CurrentTime)
		{
			int index = blockIdx.x * blockDim.x + threadIdx.x;
			while (index < vectorNeuronState_GPU2->SizeStates){
				vectorNeuronState_GPU2->VectorNeuronStates_GPU[(this->N_DifferentialNeuronState)*vectorNeuronState_GPU2->SizeStates + index] += vectorNeuronState_GPU2->AuxStateGPU[index];
				vectorNeuronState_GPU2->VectorNeuronStates_GPU[(this->N_DifferentialNeuronState + 1)*vectorNeuronState_GPU2->SizeStates + index] += vectorNeuronState_GPU2->AuxStateGPU[vectorNeuronState_GPU2->SizeStates + index];

				index += blockDim.x*gridDim.x;
			}
			
			this->integrationMethod_GPU2->NextDifferentialEquationValues(vectorNeuronState_GPU2->SizeStates, vectorNeuronState_GPU2->VectorNeuronStates_GPU);
		}


		/*!
		 * \brief It evaluates if a neuron must spike.
		 *
		 * It evaluates if a neuron must spike.
		 *
		 * \param previous_V previous membrane potential
		 * \param NeuronState neuron state variables.
		 * \param index Neuron index inside the neuron model.
 		 * \param elapsedTimeInNeuronModelScale integration method step.
		 * \return It returns if a neuron must spike.
		 */
		__device__ void EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elapsedTimeInNeuronModelScale){
			if (NeuronState[index] > this->vthr && previous_V < this->vthr){
				vectorNeuronState_GPU2->LastSpikeTimeGPU[index]=0.0;
				vectorNeuronState_GPU2->InternalSpikeGPU[index] = true;
			}
		}


		__device__ void EvaluateDifferentialEquation(int index, int SizeStates, float * NeuronState, float * AuxNeuronState, float elapsed_time){
			float V=NeuronState[index];
			float m=NeuronState[SizeStates + index];
			float h=NeuronState[2*SizeStates + index];
			float n=NeuronState[3*SizeStates + index];
			float gexc=NeuronState[4*SizeStates + index];
			float ginh=NeuronState[5*SizeStates + index];

			if(V<EK){
				V=EK;
			}
			if(V>ENa){
				V=ENa;
			}

			if(m<0){
				m=0.0f;
			}
			if(m>1){
				m=1.0f;
			}
			if(h<0){
				h=0.0f;
			}
			if(h>1){
				h=1.0f;
			}
			if(n<0){
				n=0.0f;
			}
			if(n>0.72){
				n=0.72f;
			}

			//v			
			AuxNeuronState[blockDim.x*blockIdx.x + threadIdx.x]=(gexc * (this->eexc - V) + ginh * (this->einh - V) + grest * (this->erest - V) + gNa*m*m*m*h*(ENa - V) + gKd*n*n*n*n*(EK - V))*this->inv_cm;
			
			//m
			float alpha_m, beta_m;
			if((13.0f-V+VT)!=0.0f){
				alpha_m=0.32f*(13.0f-V+VT)/(__expf((13.0f-V+VT)*0.25f)-1.0f);
			}else{
				alpha_m=1.279999894092334f;
			}

			if((V-VT-40.0f)!=0.0f){
				beta_m=0.28f*(V-VT-40.0f)/(__expf((V-VT-40.0f)*0.2f)-1.0f);
			}else{
				beta_m=1.400030971093304f;
			}
			AuxNeuronState[blockDim.x*gridDim.x + blockDim.x*blockIdx.x + threadIdx.x]=(alpha_m*(1.0f-m)-beta_m*m);
			
			//h
			float alpha_h=0.128f*__expf((17.0f-V+VT)*0.0555555555555555555f);
			float beta_h=4.0f/(1.0f+__expf((40.0f-V+VT)*0.2f));
			AuxNeuronState[2*blockDim.x*gridDim.x + blockDim.x*blockIdx.x + threadIdx.x]=(alpha_h*(1-h)-beta_h*h);
			
			//n
			float alpha_n;
			if((15.0f-V+VT)!=0.0f){
				alpha_n=0.032f*(15.0f-V+VT)/(__expf((15.0f-V+VT)*0.2f)-1.0f);
			}else{
				alpha_n=0.1599999867615418f;
			}
			float beta_n=0.5f*__expf((10.0f-V+VT)*0.025f);
			AuxNeuronState[3*blockDim.x*gridDim.x + blockDim.x*blockIdx.x + threadIdx.x]=(alpha_n*(1-n)-beta_n*n);
		}


		/*!
		 * \brief It evaluates the time depedendent Equation in NeuronState for elapsed_time and it stores the results in NeuronState.
		 *
		 * It evaluates the time depedendent Equation in NeuronState for elapsed_time and it stores the results in NeuronState.
		 *
		 * \param index index inside the NeuronState vector.
		 * \param SizeStates number of element in NeuronState vector.
		 * \param NeuronState value of the neuron state variables where time dependent equations are evaluated.
		 * \param elapsed_time integration time step.
		 * \param elapsed_time_index index inside the conductance_exp_values array.
		 */
		__device__ void EvaluateTimeDependentEquation(int index, int SizeStates, float * NeuronState, float elapsed_time, int elapsed_time_index){
			float limit=1e-9;

			float * Conductance_values=this->Get_conductance_exponential_values(elapsed_time_index);
			
			if(NeuronState[this->N_DifferentialNeuronState*SizeStates + index]<limit){
				NeuronState[this->N_DifferentialNeuronState*SizeStates + index]=0.0f;
			}else{
				NeuronState[this->N_DifferentialNeuronState*SizeStates + index]*=  Conductance_values[0];
			}
			if(NeuronState[(this->N_DifferentialNeuronState+1)*SizeStates + index]<limit){
				NeuronState[(this->N_DifferentialNeuronState+1)*SizeStates + index]=0.0f;
			}else{
				NeuronState[(this->N_DifferentialNeuronState+1)*SizeStates + index]*=  Conductance_values[1];
			}
		}

		/*!
		 * \brief It calculates the conductace exponential value for an elapsed time.
		 *
		 * It calculates the conductace exponential value for an elapsed time.
		 *
		 * \param index elapsed time index .
		 * \param elapses_time elapsed time.
		 */
		__device__ void Calculate_conductance_exp_values(int index, float elapsed_time){
			//excitatory synapse.
			Set_conductance_exp_values(index, 0, __expf(-elapsed_time*this->inv_texc));
			//inhibitory synapse.
			Set_conductance_exp_values(index, 1, __expf(-elapsed_time*this->inv_tinh));
		}



		/*!
		* \brief It loads the integration method from the neuron model configuration file
		*
		* It loads the integration method from the neuron model configuration file
		*
		* \param integrationName integration method name
		* \param Buffer_GPU integration method parameters
		*/
		__device__ void loadIntegrationMethod_GPU2(char const* integrationName, void ** Buffer_GPU){

			//DEFINE HERE NEW INTEGRATION METHOD
			if (cmp4(integrationName, "Euler", 5) == 0){
				this->integrationMethod_GPU2 = (Euler_GPU2<HHTimeDrivenModel_GPU2> *) new Euler_GPU2<HHTimeDrivenModel_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "RK2", 3) == 0){
				this->integrationMethod_GPU2 = (RK2_GPU2<HHTimeDrivenModel_GPU2> *) new RK2_GPU2<HHTimeDrivenModel_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "RK4", 3) == 0){
				this->integrationMethod_GPU2 = (RK4_GPU2<HHTimeDrivenModel_GPU2> *) new RK4_GPU2<HHTimeDrivenModel_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "BDF", 3) == 0 && atoiGPU(integrationName, 3)>0 && atoiGPU(integrationName, 3)<7){
				this->integrationMethod_GPU2 = (BDFn_GPU2<HHTimeDrivenModel_GPU2> *) new BDFn_GPU2<HHTimeDrivenModel_GPU2>(this, Buffer_GPU, atoiGPU(integrationName, 3));
			}
			else if (cmp4(integrationName, "Bifixed_Euler", 13) == 0){
				this->integrationMethod_GPU2 = (Bifixed_Euler_GPU2<HHTimeDrivenModel_GPU2> *) new Bifixed_Euler_GPU2<HHTimeDrivenModel_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "Bifixed_RK2", 11) == 0){
				this->integrationMethod_GPU2 = (Bifixed_RK2_GPU2<HHTimeDrivenModel_GPU2> *) new Bifixed_RK2_GPU2<HHTimeDrivenModel_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "Bifixed_RK4", 11) == 0){
				this->integrationMethod_GPU2 = (Bifixed_RK4_GPU2<HHTimeDrivenModel_GPU2> *) new Bifixed_RK4_GPU2<HHTimeDrivenModel_GPU2>(this, Buffer_GPU);
			}
			else{
				printf("There was an error loading the integration methods of the GPU.\n");

			}
		}

};


#endif /* LIFTIMEDRIVENMODEL_1_2_GPU2_H_ */
