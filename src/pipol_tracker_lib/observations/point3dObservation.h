#ifndef point3dObservation_H
#define point3dObservation_H

#include "basicObservation.h"
#include "geometry/point3dCov.h"

/**
 *
 * \brief Cpoint3dObservation implements 2D posCov observation
 * 
 * Cpoint3dObservation implements 2D posCov observation
 * Data members are:
 *	Cposition3d
 * 	Covariance diagonal parameters cxx, cyy, czz, chh, cpp, crr
 * 	xy cross covariance cxy
 * 
*/
class Cpoint3dObservation : public CbasicObservation
{
	public:
		/**
		* \brief Point data
		*
		* Point data
		*
		*/		
		Cpoint3dCov point;
		
	public:
		/**
		* \brief Constructor
		*
		* Constructor
		*
		*/				
		Cpoint3dObservation();

		/**
		* \brief Destructor
		*
		* Destructor
		*
		*/						
		~Cpoint3dObservation();
};	
#endif
