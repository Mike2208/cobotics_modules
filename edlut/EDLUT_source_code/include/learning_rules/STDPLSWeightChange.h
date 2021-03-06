/***************************************************************************
 *                           STDPLSWeightChange.h                          *
 *                           -------------------                           *
 * copyright            : (C) 2013 by Jesus Garrido                        *
 * email                : jgarrido@atc.ugr.es                              *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#ifndef STDPLSWEIGHTCHANGE_H_
#define STDPLSWEIGHTCHANGE_H_

#include "./STDPWeightChange.h"

/*!
 * \file STDPLSWeightChange.h
 *
 * \author Jesus Garrido
 * \date March 2013
 *
 * This file declares a class which abstracts a STDP learning rule (accounting only the last spike).
 */

class Interconnection;

/*!
 * \class STDPLSWeightChange
 *
 * \brief Learning rule.
 *
 * This class abstract the behaviour of a STDP learning rule (accounting only the last spike).
 *
 * \author Jesus Garrido
 * \date March 2013
 */
class STDPLSWeightChange: public STDPWeightChange {
	public:

		/*!
		 * \brief Default constructor with parameters.
		 *
		 * It generates a new learning rule with its index.
		 *
		 * \param NewLearningRuleIndex learning rule index.
		 */ 
		STDPLSWeightChange(int NewLearningRuleIndex);

		/*!
		 * \brief Object destructor.
		 *
		 * It remove the object.
		 */
		virtual ~STDPLSWeightChange();


		/*!
		 * \brief It initialize the state associated to the learning rule for all the synapses.
		 *
		 * It initialize the state associated to the learning rule for all the synapses.
		 *
		 * \param NumberOfSynapses the number of synapses that implement this learning rule.
		 * \param NumberOfNeurons the total number of neurons in the network
		 */
		void InitializeConnectionState(unsigned int NumberOfSynapses, unsigned int NumberOfNeurons);


		/*!
		 * \brief It prints the learning rule info.
		 *
		 * It prints the current learning rule characteristics.
		 *
		 * \param out The stream where it prints the information.
		 *
		 * \return The stream after the printer.
		 */
		virtual ostream & PrintInfo(ostream & out);
};

#endif /* STDPLSWEIGHTCHANGE_H_ */
