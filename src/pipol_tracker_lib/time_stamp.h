#ifndef timeStamp_H
#define timeStamp_H

#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <math.h>

const unsigned int TIME_STAMP_DIGITS = 10;

/**
 *
 * \brief TimeStamp implements basic funcitionalities for time stamps
 * 
 * TimeStamp implements basic funcitionalities for time stamps
 *
*/
class TimeStamp
{
	protected:
		/**
		* \brief Time stamp
		*
		* Time. Expressed in seconds from 1th jan 1970.
		*
		*/		
		double timeStamp;

	public:
		/**
		* \brief Constructor
		*
		* Constructor
		*
		*/				
		TimeStamp();

		/**
		* \brief Destructor
		*
		* Destructor
		*
		*/						
		~TimeStamp();
		
		/**
		* \brief Time stamp to now
		*
		* Sets time stamp to now
		*
		*/										
		void setToNow();
		
		/**
		* \brief Set time stamp
		*
		* Sets time stamp to a given value passed as a timeval struct
		*
		*/												
		void set(timeval ts);
		
		/**
		* \brief Set time stamp
		*
		* Sets time stamp to a given value passed as a two-integer (seconds and nanoseconds)
		*
		*/												
		void set(unsigned long int sec, unsigned long int nanosec);
		
		/**
		* \brief Set time stamp
		*
		* Sets time stamp to a given value passed as a double (seconds)
		*
		*/														
		void set(double ts);

		/**
		* \brief Get time stamp
		*
		* Returns time stamp
		*
		*/																
		double get() const;
		
		/**
		* \brief Get time stamp (only seconds)
		*
		* Returns seconds of time stamp
		*
		*/																
		unsigned long int getSeconds();
		
		/**
		* \brief Get time stamp (only nano seconds)
		*
		* Returns nanoseconds part of time stamp
		*
		*/																
		unsigned long int getNanoSeconds();
			
		/**
		 * \brief Prints time stamp to a given ostream 
		 * 
		 */
		void print(std::ostream *ost = &std::cout) const; 
};	
#endif
