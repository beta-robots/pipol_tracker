#ifndef bodyObservation_H
#define bodyObservation_H

#include "basicObservation.h"
#include "geometry/point3d.h"

/**
 *
 * \brief CbodyObservation implements 2D posCov observation
 * 
 * CbodyObservation implements a metric/appearance model for human body detections
 * 
*/
class CbodyObservation : public CbasicObservation
{
	public:
		/**
		* \brief Normalized vector pointing the body detection
		*
		* Normalized vector pointing the body detection
		*
		*/		
		Cpoint3d direction;
		
		/**
		* \brief RGB eigen vector
		*
		* RGB eigen vector.
		*
		*/		
		Cpoint3d rgbEigen;
		
		/**
		* \brief RGB cluster centers
		*
		* RGB cluster centers.
		* Components in the [0..1] range.
		*
		*/		
		std::vector<Cpoint3d> rgbCenters;
		
		/**
		 * \brief Hog vector
		 * 
		 * Histogram of gradients of the person region
		 * 
		 */
		std::vector<double> hog;
		
		double bbX,bbY; //up-left limits of the bounding box
		double bbW, bbH; //width and height of the bounding box
		
	public:
		/**
		* \brief Constructor
		*
		* Constructor
		*
		*/				
		CbodyObservation();

		/**
		* \brief Destructor
		*
		* Destructor
		*
		*/						
		~CbodyObservation();
};	
#endif
