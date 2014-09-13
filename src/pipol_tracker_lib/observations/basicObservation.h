#ifndef basicObservation_H
#define basicObservation_H

#include <iostream>
#include <vector>
#include <math.h>
#include "timeStamp.h"

const unsigned int NEW_DATA = 0x1;
const unsigned int CORRECT_DATA = 0x2;

/**
 *
 * \brief CbasicObservation implements common features of observations.
 * 
 * CbasicObservation implements common features of observations to be integrated by fusing/perception algorithms
 * Three data members are considered: 
 *	status: integer that specifies if observation data is correct and if it has been already used by a process.
 *	timeStamp: time when observation was taken. Ideally, it should be set from acquisition data.
 * Observation specific data should be implemented by classes that inherits from this basic class
 *
*/
class CbasicObservation
{
	protected:
		/** \brief Observation status
		*
		* Observation status as a bitwise OR of different status values.
		*
		*/		
		unsigned int status;
		
		/** \brief Observation id
		*
		* Observation id. In some applications it could be necessary to identify observations with an ID
		*
		*/		
		unsigned int id;	
            
            /** \brief wheter this detection has been associated or not
             * 
             * wheter this detection has been associated or not
             * 
             **/
            bool associated_;
		
	public: 		
		/** \brief Time stamp
		*
		* Time where observation is taken. Expressed in seconds from 1th jan 1970.
		*
		*/		
		CtimeStamp timeStamp;

	public:
		/** \brief Constructor
		*
		* Constructor
		*
		*/				
		CbasicObservation();

		/** \brief Destructor
		*
		* Destructor
		*
		*/						
		~CbasicObservation();

		/** \brief Mark as new
		*
		* Marks status value to indicate that the observation is new (not already used)
		*
		*/								
		void markAsNew();
				
		/**
		* \brief Mark as old
		*
		* Marks status value to indicate that the observation is old (already used)
		*
		*/						
		void markAsOld();
		
		/**
		* \brief Mark as correct
		*
		* Marks status value to indicate that the observation data is correct 
		*
		*/								
		void markAsCorrect();
		
		/**
		* \brief Mark as uncorrect
		*
		* Marks status value to indicate that the observation data is uncorrect 
		*
		*/										
		void markAsUncorrect();
		
		/**
		* \brief Mark status
		*
		* Marks status to a given value
		*
		*/										
		void markStatus(unsigned int value);
		
		/**
		* \brief Asks if is this new.
		*
		* Returns true in case that observation data is new. False otherwise.
		*
		*/														
		bool isNew();
		
		/**
		* \brief Asks if is this correct
		*
		* Returns true in case that observation data is correct. False otherwise.
		*
		*/																
		bool isCorrect();
		
		/**
		* \brief Get status
		*
		* Returns status value
		*
		*/																
		unsigned int getStatus();
		
		/**
		* \brief Get id
		*
		* Returns id
		*
		*/																
		unsigned int getId();
		
		/**
		* \brief Set id
		*
		* Sets id
		*
		*/																
		void setId(unsigned int newId);
            
            /** \brief Set associated flag
             * 
             * Set associated flag
             * 
             **/
            void setAssociated(bool _assoc_flag);
            
            /** \brief Check associated flag
             * 
             * Check associated flag
             * 
             **/
            bool isAssociated() const;
            
				
};	
#endif
