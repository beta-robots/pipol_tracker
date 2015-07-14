#ifndef detection_legs_H
#define detection_legs_H

//pipol tracker
#include "detection_base.h"

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

    public:
        /** \brief Constructor
        *
        * Constructor
        *
        */              
        DetectionLegs();

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
        double likelihood(const Eigen::VectorXs & _state);
        
        /** \brief Leg Detection cost. 
         * 
         * Leg Detection cost. 
         * Requires: A reference to an Eigen Vector, which is the state point to be evaluated. 
         * Assures: Return value should be between 0 (very likely) and inf (unlikely). 
         * Used by Optimization approach.
         * 
         **/
        double likelihood(const Eigen::VectorXs & _state);
                                
};  
#endif
