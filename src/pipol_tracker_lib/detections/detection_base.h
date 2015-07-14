#ifndef detection_base_H
#define detection_base_H

//std
#include <iostream>

//eigen
#include <eigen3/Eigen/Geometry>

//pipol tracker
#include "../time_stamp.h"

/**
 *
 * \brief DetectionBase implements common features of detections
 * 
 * DetectionBase implements common features of detections
 * Two data members are considered: 
 *  - status: integer that specifies if observation data is correct and if it has been already used by a process.
 *  - timeStamp: time when observation was taken. Ideally, it should be set from acquisition data.
 * 
 * Detection specific data should be implemented by classes that inherits from this basic class
 *
*/
class DetectionBase
{
    protected:
        /** \brief new flag
        * 
        * indicates if this detection is new, never used yet
        * 
        **/        
        bool is_new_; 
        
        /** \brief association flag
        * 
        * indicates if this detection is correct, if some correctness check is passed
        * 
        **/                
        bool is_correct_; 
        
        /** \brief association flag
        * 
        * indicates if this detection has been associated or not
        * 
        **/        
        bool is_associated_; 
        
        /** \brief Observation id
        *
        * Detection id. In some applications it could be necessary to identify detections with an ID
        *
        */      
        unsigned int id_;    
        
    public:         
        /** \brief Time stamp
        *
        * Time where detection is taken. Expressed in seconds from 1th jan 1970.
        *
        */      
        TimeStamp time_stamp_;
        
        /** \brief sensor frame
         * 
         * Sensor frame from which this detection has been taken
         * 
         **/

    public:
        /** \brief Constructor
        *
        * Constructor
        *
        */              
        DetectionBase();

        /** \brief Destructor
        *
        * Destructor
        *
        */                      
        ~DetectionBase();

        /** \brief Mark as new
        *
        * Mark as new. Set is_new_ flag
        *
        */                              
        void setAsNew();
                
        /**
        * \brief Mark as old
        *
        * Mark as old. Reset is_new_ flag
        *
        */                      
        void setAsOld();
        
        /**
        * \brief Mark as correct
        *
        * Mark as correct. Set is_correct_ flag.
        *
        */                              
        void setAsCorrect();
        
        /**
        * \brief Mark as uncorrect
        *
        * Mark as uncorrect. Reset is_correct_ flag.
        *
        */                                      
        void setAsUncorrect();
        
        /** \brief Mark as associated
        * 
        * Mark this detection as associated. Set is_associated_ flag.
        * 
        **/
        void setAsAssociated();        
        
        /** \brief Mark as unassociated
        * 
        * Mark this detection as unassociated. Reset is_associated_ flag.
        * 
        **/
        void setAsUnAssociated();                
        
        /** \brief Set id
        *
        * Sets id
        *
        */                                                              
        void setId(unsigned int _new_id);        
                
        /** \brief Asks if is this new.
        *
        * Returns true in case that observation data is new. False otherwise.
        *
        */                                                      
        bool isNew() const;
        
        /** \brief Asks if is this correct
        *
        * Returns true in case that observation data is correct. False otherwise.
        *
        */                                                              
        bool isCorrect() const;
            
        /** \brief Asks if is this associated
        * 
        * Check associated flag
        * 
        **/
        bool isAssociated() const;
        
        /** \brief Get id
        *
        * Returns id
        *
        */                                                              
        unsigned int getId() const;
                
        /** \brief Detection likelihood. 
         * 
         * Detection likelihood. 
         * Requires: A reference to an Eigen Vector, which is the state point to be evaluated. 
         * Assures: Return value should be between 0 (unlikely) and 1 (very likely). 
         * Used by particle filetering approach.
         * 
         **/
        virtual double likelihood(const Eigen::VectorXs & _state) = 0;
        
        /** \brief Detection cost. 
         * 
         * Detection cost. 
         * Requires: A reference to an Eigen Vector, which is the state point to be evaluated. 
         * Assures: Return value should be between 0 (very likely) and inf (unlikely). 
         * Used by Optimization approach.
         * 
         **/
        virtual double likelihood(const Eigen::VectorXs & _state) = 0;
                                
};  
#endif
