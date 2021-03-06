/***************************************************************************
 *                           CompressTableBasedModel.h                     *
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

#ifndef COMPRESSTABLEBASEDMODEL_H_
#define COMPRESSTABLEBASEDMODEL_H_

/*!
 * \file CompressTableBasedModel.h
 *
 * \author Francisco Naveros
 * \date July 2015
 *
 * This file declares a class which implements a neuron model based in
 * look-up tables. This one works as the TableBasedModel class. The main
 * difference it is that this class try to compress some tables in only 
 * one.
 */

#include "EventDrivenNeuronModel.h"

#include "../spike/EDLUTFileException.h"

class CompressNeuronModelTable;
class Interconnection;

/*!
 * \class CompressTableBasedModel
 *
 * \brief Spiking neuron model based in look-up tables
 *
 * This class implements the behavior of a neuron in a spiking neural network.
 * It includes internal model functions which define the behavior of the model
 * (initialization, update of the state, synapses effect, next firing prediction...).
 * This behavior is calculated based in precalculated look-up tables.
 *
 * \author Francisco Naveros
 * \date July 2015
 */
class CompressTableBasedModel: public EventDrivenNeuronModel {
	protected:
		
		/*!
		 * \brief Number of state variables (no include time).
		 */
		unsigned int NumStateVar;

		/*!
		 * \brief Number of time dependent state variables.
		 */
		unsigned int NumTimeDependentStateVar;

		/*!
		 * \brief Number of synaptic variables.
		 */
		unsigned int NumSynapticVar;

		/*!
		 * \brief Index of synaptic variables.
		 */
		unsigned int * SynapticVar;

		/*!
		 * \brief Original order of state variables.
		 */
		unsigned int * StateVarOrderOriginalIndex;

		/*!
		 * \brief New order of state variables in compressed tables (some state variables will be stored in only one table).
		 */
		unsigned int * StateVarOrderIndex;
		
		/*!
		 * \brief New sub index inside of the state variables in compressed tables.
		 */
		unsigned int * StateVarOrderSubIndex;



		unsigned int * TablesIndex;

		/*!
		 * \brief Table which calculates each state variable.
		 */
		CompressNeuronModelTable ** StateVarTable;

		/*!
		 * \brief Firing time table
		 */
		CompressNeuronModelTable * FiringTable;

		/*!
		 * \brief Index of the firing time table
		 */
		unsigned int FiringIndex;

		/*!
		 * \brief Sub index inside the compressed table
		 */
		unsigned int FiringSubIndex;

		/*!
		 * \brief End firing time table
		 */
		CompressNeuronModelTable * EndFiringTable;

		/*!
		 * \brief Index of the end firing time table
		 */
		unsigned int EndFiringIndex;
		
		/*!
		 * \brief Sub index inside the compressed table
		 */
		unsigned int EndFiringSubIndex;

		/*!
		 * \brief Number of original tables
		 */
		unsigned int NumTables;

		/*!
		 * \brief Number of final compressed tables
		 */
		unsigned int NumCompresedTables;

		/*!
		 * \brief Precalculated tables
		 */
		CompressNeuronModelTable * Tables;

		/*!
		 * \brief Vector where we temporary store initial values
		 */
		float * InitValues;

		/*!
		 * \brief Number of state variables that store each compressed table
		 */
		int * NumVariablesPerCompressedTable;

		/*!
		 * \brief It loads the neuron model description.
		 *
		 * It loads the neuron type description from the file .cfg.
		 *
		 * \param ConfigFile Name of the neuron description file (*.cfg).
		 *
		 * \throw EDLUTFileException If something wrong has happened in the file load.
		 */
		virtual void LoadNeuronModel(string ConfigFile);

		/*!
		 * \brief It loads the neuron model tables.
		 *
		 * It loads the neuron model tables from his .dat associated file.
		 *
		 * \pre The neuron model must be previously initialized or loaded
		 *
		 * \param TableFile Name of the table file (*.dat).
		 *
		 * \see LoadNeuronModel()
		 * \throw EDLUTException If something wrong has happened in the tables loads.
		 */
		virtual void LoadTables(string TableFile);

		/*!
		 * \brief It returns the end of the refractory period.
		 *
		 * It returns the end of the refractory period.
		 *
		 * \param index index inside the VectorNeuronState of the neuron model.
		 * \param VectorNeuronState of the neuron model.
		 *
		 * \return The end of the refractory period. -1 if no spike is predicted.
		 */
		virtual double EndRefractoryPeriod(int index, VectorNeuronState * State);

		/*!
		 * \brief It updates the neuron state after the evolution of the time.
		 *
		 * It updates the neuron state after the evolution of the time.
		 *
		 * \param index index inside the VectorNeuronState of the neuron model.
		 * \param VectorNeuronState of the neuron model.
		 * \param CurrentTime Current simulation time.
		 */
		virtual void UpdateState(int index, VectorNeuronState * State, double CurrentTime);


		/*!
		 * \brief It abstracts the effect of an input spike in the cell.
		 *
		 * It abstracts the effect of an input spike in the cell.
		 *
		 * \param index index inside the VectorNeuronState of the neuron model.
		 * \param InputConnection Input connection from which the input spike has got the cell.
		 */
		virtual void SynapsisEffect(int index, Interconnection * InputConnection);


		/*!
		 * \brief It returns the next spike time.
		 *
		 * It returns the next spike time.
		 *
		 * \param index index inside the VectorNeuronState of the neuron model.
		 * \param VectorNeuronState of the neuron model.
		 *
		 * \return The next firing spike time. -1 if no spike is predicted.
		 */
		virtual double NextFiringPrediction(int index, VectorNeuronState * State);

	public:
		/*!
		 * \brief Default constructor with parameters.
		 *
		 * It generates a new neuron model object loading the configuration of
		 * the model and the look-up tables.
		 *
		 * \param NeuronTypeID Neuron model type.
		 * \param NeuronModelID Neuron model description file.
		 */
		CompressTableBasedModel(string NeuronTypeID, string NeuronModelID);

		/*!
		 * \brief Class destructor.
		 *
		 * It destroys an object of this class.
		 */
		~CompressTableBasedModel();


		/*!
		 * \brief It loads the neuron model description and tables (if necessary).
		 *
		 * It loads the neuron model description and tables (if necessary).
		 *
		 * \throw EDLUTFileException If something wrong has happened in the file load.
		 */
		virtual void LoadNeuronModel();

		/*!
		 * \brief It tries to compact the neuron tables.
		 *
		 * It tries to compact the neuron tables.
		 */
		virtual void CompactTables();

		/*!
		 * \brief It compares if two tables have the same index and interpolation method and can be compressed in only one table.
		 *
		 * It compares if two tables have the same index and interpolation method and can be compressed in only one table.
		 *
		 * \param table1 first table.
		 * \param table2 second table.
		 *
		 * \return Boolean value that return if both tables are equal.
		 */
		virtual bool CompareNeuronModelTableIndex(CompressNeuronModelTable * table1, CompressNeuronModelTable * table2);

		/*!
		 * \brief It creates the neuron state and initializes to defined values.
		 *
		 * It creates the neuron state and initializes to defined values.
		 *
		 * \return A new object with the neuron state.
		 */
		virtual VectorNeuronState * InitializeState();

		/*!
		 * \brief It generates the first spike (if any) in a cell.
		 *
		 * It generates the first spike (if any) in a cell.
		 *
		 * \param Cell The cell to check if activity is generated.
		 *
		 * \return A new internal spike if someone is predicted. 0 if none is predicted.
		 */
		virtual InternalSpike * GenerateInitialActivity(Neuron *  Cell);


		/*!
		 * \brief It processes a propagated spike (input spike in the cell).
		 *
		 * It processes a propagated spike (input spike in the cell).
		 *
		 * \note This function doesn't generate the next propagated spike. It must be externally done.
		 *
		 * \param inter the interconection which propagate the spike
		 * \param time the time of the spike.
		 *
		 * \return A new internal spike if someone is predicted. 0 if none is predicted.
		 */
		virtual InternalSpike * ProcessInputSpike(Interconnection * inter, double time);


		/*!
		 * \brief It predicts if the neuron would generate a internalSpike aftern all the propagated spikes have arrived. 
		 *
		 * It predicts if the neuron would generate a internalSpike after all the propagated spikes have arrived. 
		 *
		 * \param target Neuron that must be updated.
		 * \param time time.
		 *
		 * \return A new internal spike if someone is predicted. 0 if none is predicted.
		 */
		virtual InternalSpike * ProcessActivityAndPredictSpike(Neuron * target, double time);


		/*!
		 * \brief It processes an internal spike and generates an end refractory period event.
		 *
		 * It processes an internal spike and generates an end refractory period event.
		 *
		 * \param OutputSpike The spike happened.
		 *
		 * \return A new end refractory period event.
		 */
		virtual EndRefractoryPeriodEvent * ProcessInternalSpike(InternalSpike * OutputSpike);

		/*!
		 * \brief It calculates if an internal spike must be generated at the end of the refractory period.
		 *
		 * It calculates if an internal spike must be generated at the end of the refractory period.
		 *
		 * \param time end of the refractory period.
		 * \param neuron source neuron.
		 *
		 * \return A new internal spike.
		 */
		virtual InternalSpike * GenerateNextSpike(double time, Neuron * neuron);


		/*!
		 * \brief Check if the spike must be discard.
		 *
		 * Check if the spike must be discard. A spike must be discard if there are discrepancies between
		 * the next predicted spike and the spike time.
		 *
		 * \param OutputSpike The spike happened.
		 *
		 * \return True if the spike must be discard. False in otherwise.
		 */
		virtual bool DiscardSpike(InternalSpike *  OutputSpike);


		/*!
		* \brief It gets the neuron output activity type (spikes or currents).
		*
		* It gets the neuron output activity type (spikes or currents).
		*
		* \return The neuron output activity type (spikes or currents).
		*/
		enum NeuronModelOutputActivityType GetModelOutputActivityType();

		/*!
		* \brief It gets the neuron input activity types (spikes and/or currents or none).
		*
		* It gets the neuron input activity types (spikes and/or currents or none).
		*
		* \return The neuron input activity types (spikes and/or currents or none).
		*/
		enum NeuronModelInputActivityType GetModelInputActivityType();


		/*!
		 * \brief It prints the table based model info.
		 *
		 * It prints the current table based model characteristics.
		 *
		 * \param out The stream where it prints the information.
		 *
		 * \return The stream after the printer.
		 */
		virtual ostream & PrintInfo(ostream & out);


		/*!
		 * \brief It initialice VectorNeuronState.
		 *
		 * It initialice VectorNeuronState.
		 *
		 * \param N_neurons cell number inside the VectorNeuronState.
		 * \param OpenMPQueueIndex openmp index
		 */
		virtual void InitializeStates(int N_neurons, int OpenMPQueueIndex);


		/*!
		 * \brief It Checks if the neuron model has this connection type.
		 *
		 * It Checks if the neuron model has this connection type.
		 *
		 * \param Type input connection type.
		 *
		 * \return If the neuron model supports this connection type
		 */
		virtual bool CheckSynapseType(Interconnection * connection);

};

#endif /* COMPRESSTABLEBASEDMODEL_H_ */
