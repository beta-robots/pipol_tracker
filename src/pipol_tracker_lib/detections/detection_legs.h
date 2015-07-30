#ifndef detection_legs_H
#define detection_legs_H

//pipol tracker
#include "detection_base.h"

struct ParamsDetectionLegs
{
	double person_radius_; //considered radius of a person, for leg detector [m]
	double person_radius_sq_; //squared of the above [m^2]
	double matching_alpha_; //For legs likelihood, Difference between lik(0) and lik(border of "pass band") , in [0,0.5]
	double matching_beta_; //For legs likelihood, Off-band expoenential decayment, set in [2,50]
	double K1_; //constant computed once from the above values to speed up processing.
}

/**
 *
 * \brief DetectionLegs implements likelihood and cost of leg detections
 * 
 * DetectionLegs implements likelihood and cost of leg detections
 *
*/
class DetectionLegs : public DetectionBase
{
    protected:
        /** \brief leg detection point
         * 
         * Point at which legs are detected, with respect to robot frame
         * 
         **/
        Eigen::Vector2d point_;
		
		/** \brief Pointer to tunning parameters
		 * 
		 * Pointer to tunning parameters for likelihood and cost functions
		 * 
		 **/
		ParamsDetectionLegs *params_;  

    public:
        /** \brief Constructor
        *
        * Constructor
        *
        */              
        DetectionLegs(const Eigen::Vector2d & point_, const ParamsDetectionLegs *_params);

        /** \brief Destructor
        *
        * Destructor
        *
        */                      
        ~DetectionLegs();
                
        /** \brief Leg Detection likelihood. 
         * 
         * Leg Detection likelihood. 
         * Requires: A reference to an Eigen Vector, which is the state point to be evaluated. 
         * Assures: Return value should be between 0 (unlikely) and 1 (very likely). 
         * Used by particle filetering approach.
         * 
         **/
        double likelihood(const Eigen::VectorXs & _state) const;
        
        /** \brief Leg Detection cost. 
         * 
         * Leg Detection cost. 
         * Requires: A reference to an Eigen Vector, which is the state point to be evaluated. 
         * Assures: Return value should be between 0 (very likely) and inf (unlikely). 
         * Used by Optimization approach.
         * 
         **/
        double cost(const Eigen::VectorXs & _state);
                                
};  
#endif
