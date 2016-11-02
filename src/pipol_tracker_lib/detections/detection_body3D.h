#ifndef detection_body3D_H
#define detection_body3D_H

//pipol tracker
#include "detection_base.h"

struct ParamsDetectionBody3D
{
	double person_radius_; //considered radius of a person, for body3D detector [m]
	double person_radius_sq_; //squared of the above [m^2]
	double matching_alpha_;
	double matching_beta_;
	double K1_; //constant computed from the above values to speed up processing.
}

/**
 *
 * \brief DetectionBody3D implements likelihood and cost of leg detections
 * 
 * DetectionBody3D implements likelihood and cost of leg detections
 *
*/
class DetectionBody3D : public DetectionBase
{
    protected:
        /** \brief leg detection point
         * 
         * Point 3D at which body is detected, with respect to robot frame
         * 
         **/
        Eigen::Vector3d point_;
		
		/** \brief Pointer to tunning parameters
		 * 
		 * Pointer to tunning parameters for likelihood and cost functions
		 * 
		 **/
		ParamsDetectionBody3D *params_;  

    public:
        /** \brief Constructor
        *
        * Constructor
        *
        */              
        DetectionBody3D(const Eigen::Vector3d & point_, const ParamsDetectionBody3D *_params);

        /** \brief Destructor
        *
        * Destructor
        *
        */                      
        ~DetectionBody3D();
                
        /** \brief Body 3D Detection likelihood. 
         * 
         * Body 3D Detection likelihood. 
         * Requires: A reference to an Eigen Vector, which is the state point to be evaluated. 
         * Assures: Return value should be between 0 (unlikely) and 1 (very likely). 
         * Used by particle filetering approach.
         * 
         **/
        double likelihood(const Eigen::VectorXs & _state) const;
        
        /** \brief Body 3D Detection cost. 
         * 
         * Body 3D Detection cost. 
         * Requires: A reference to an Eigen Vector, which is the state point to be evaluated. 
         * Assures: Return value should be between 0 (very likely) and inf (unlikely). 
         * Used by Optimization approach.
         * 
         **/
        double cost(const Eigen::VectorXs & _state);
                                
};  
#endif
