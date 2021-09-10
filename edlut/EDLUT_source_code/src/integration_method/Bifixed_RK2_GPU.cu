/***************************************************************************
 *                           RK2_GPU.cu                                    *
 *                           -------------------                           *
 * copyright            : (C) 2013 by Francisco Naveros                    *
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

#include "../../include/integration_method/Bifixed_RK2_GPU.h"
#include "../../include/integration_method/Bifixed_RK2_GPU2.h"
#include "../../include/neuron_model/TimeDrivenNeuronModel_GPU2.h"

//Library for CUDA
#include "cuda_runtime.h"
#include "device_launch_parameters.h"




Bifixed_RK2_GPU::Bifixed_RK2_GPU(TimeDrivenNeuronModel_GPU * NewModel, int N_neuronStateVariables, int N_differentialNeuronState, int N_timeDependentNeuronState):BiFixedStep_GPU(NewModel, "Bifixed_RK2", N_neuronStateVariables, N_differentialNeuronState, N_timeDependentNeuronState){
}

Bifixed_RK2_GPU::~Bifixed_RK2_GPU(){
	cudaFree(AuxNeuronState1);
	cudaFree(AuxNeuronState2);
}

__global__ void Bifixed_RK2_GPU_position(void ** vector, float * integration_method_parameters_GPU, float * element1, float * element2){
	vector[0]=integration_method_parameters_GPU;
	vector[1]=element1;
	vector[2]=element2;
}
	
void Bifixed_RK2_GPU::InitializeMemoryGPU(int N_neurons, int Total_N_thread){
	int size=4*sizeof(float *);

	cudaMalloc((void **)&Buffer_GPU, size);

	float integration_method_parameters_CPU[3];
	integration_method_parameters_CPU[0]=this->elapsedTimeInSeconds;
	integration_method_parameters_CPU[1]=this->bifixedElapsedTimeInSeconds;
	integration_method_parameters_CPU[2]=((float)this->N_BiFixedSteps);
	float * integration_method_parameters_GPU;
	cudaMalloc((void**)&integration_method_parameters_GPU, 3*sizeof(float));
	cudaMemcpy(integration_method_parameters_GPU,integration_method_parameters_CPU,3*sizeof(float),cudaMemcpyHostToDevice);

	cudaMalloc((void**)&AuxNeuronState1, N_NeuronStateVariables*Total_N_thread*sizeof(float));
	cudaMalloc((void**)&AuxNeuronState2, N_NeuronStateVariables*Total_N_thread*sizeof(float));

	Bifixed_RK2_GPU_position<<<1,1>>>(Buffer_GPU, integration_method_parameters_GPU, AuxNeuronState1, AuxNeuronState2);
	cudaFree(integration_method_parameters_GPU);
}





