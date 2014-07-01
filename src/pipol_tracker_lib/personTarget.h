#ifndef personTarget_H
#define personTarget_H

#include "personParticleFilter.h"
// #include "appearance/colorBoxes.h"
// #include "appearance/hsHistogram.h"

//general constants
const double ZERO_PROB = 1e-20;
const double ZERO_MATCH = 1e-10;

//track maximum size
const double TRACK_SIZE = 10;

//status values
//enum targetStatus {TO_BE_REMOVED=0, CANDIDATE, LEGGED_TARGET, VISUALLY_CONFIRMED, FRIEND_IN_SIGHT, FRIEND_OUT_OF_RANGE};

//status values (v2)
enum statusMask {     
      TO_BE_REMOVED = 0x0001,
      IN_OCCLUSION = 0x0002,
      CANDIDATE = 0x0004,
      LEGGED_TARGET = 0x0008,
      VISUALLY_CONFIRMED = 0x0010,
      FRIEND_IN_SIGHT = 0x0020,
      BACK_LEARNT = 0x0040,
      FACE_LEARNT = 0x0080
};


/**
 * \brief Implements a person as the target of the tracking
 *
 * Implements a person as the target of the tracking, while inherits a particle filter tracking a single target. 
 * The main members are:
 *      - id: id number of this target     
 *      - status: stores the status if the target, which can be:
 * 	- track: set of teh last PERSON_PATH_SIZE estimated positions for the person
 * 	- appearance : model for the person appearance
 *
*/
class CpersonTarget : public CpersonParticleFilter
{
      protected:
            unsigned int id;
            unsigned int status;
            CtimeStamp tsInit;
            list<filterEstimate> track;
            
      public:
            /** \brief Probability of occlusion
             * 
             * Probability that this target is occluded by other targets
             * 
             */
            double pOcclusion;
                        
            /** \brief appearance model based on HS histogram
            * 
            * appearance model based on HS histogram
            * 
            **/             
            //HsHistogram appearanceHistHS;
            
            /** \brief Stores matching scores between this target and each detection
            * 
            * This table stores matching scores between this target and each detection for each detector.
            * The ij-th element of this table indicates the matching score
            * of the j-th current detection of the i-th detector with this target.
            * For instance, "matchScores[LEGS].at(3) = 0.95" sets to 0.95 the matching score between 
            * this target and the 3rd detection of the LEGS detector.
            * A value lower than ZERO_MATCH indicates no matching.
            * This member is public to facilitate the Tracker to manipulate its values, since it is 
            * the Tracker who has the detection set that allow computation of matching values.
            * 
            */
            std::vector<double> matchScores[NUM_DETECTORS];
            
            /** \brief Defines an association probability table
            * 
            * Defines an association probability table for a given target.
            * The ij-th element of this table indicates the association probability 
            * of the j-th current detection of the i-th detector with this target.
            * For instance, "aProbs[LEGS].at(3) = 0.57" sets a probability of 0.57 to the association 
            * event of this target with the 3rd detection of the LEGS detector.
            * A value lower than ZERO_PROB indicates no association.
            * This member is public to facilitate the Tracker to manipulate its values, since it is 
            * the Tracker who has the global vision that allow computation of association probabilities
            * 
            */
            std::vector<double> aProbs[NUM_DETECTORS];
            
            /** \brief Stores association decision
            * 
            * Marks wheter a detection is associated or not to this target.
            * For instance, "aDecision[LEGS].at(3) = 1" indicates that 3rd detection
            * of the LEGS detector has been associated to this target, while, for instance, 
            * "aDecision[BODY].at(2) = 0" would indicate that 2nd detection of the 
            * BODY detector is not associated with this target.
            * 
            */
            std::vector<bool> aDecisions[NUM_DETECTORS];
            
	public:
		/**
		* \brief Constructor
		*
		* Constructor
		*/
		CpersonTarget(unsigned int tid);

		/**
		* \brief Destructor
		*
		* Destructor
		*/						
		virtual ~CpersonTarget();

		/**
		* \brief Sets the target ID
		*
		* Sets the target ID
		*/				
		void setId(unsigned int tid);

		/**
		* \brief Returns the target ID
		*
		* Returns the target ID
		*/						
		unsigned int getId();

            /**
            * \brief Returns initial time stamp
            *
            * Returns time stamp of the creation of this target
            */                                              
            double getTsInit();

            /** \brief Returns the full value of target status
            *
            * Returns the full value of target status
            * 
            */                                              
            unsigned int getStatus();

            /** \brief Returns the maximum statusMask value of target status
            *
            * Returns the maximum statusMask value of target status
            * 
            */                                                          
            unsigned int getMaxStatus();

            /** \brief Sets a given value of the status mask
            *
            * Queries for a given value of the status mask
            * 
            */                                              
            void setStatus(statusMask sm, bool value);
            
            /** \brief Queries for a given value of the status mask
            *
            * Queries for a given value of the status mask
            * 
            */                                              
            bool isStatus(statusMask sm);
            
            /**
            * \brief Updates the target status
            *
            * Updates the target status
            * Arguments are thresholds as follows:
            *       - th1: number of consecutive uncorrected iterations to consider target to be removed
            *       - th2: number of iterations to switch from CANDIDATE to LEGGED_TARGET
            *       - th3: number of visual detections associated to switch from LEGGED_TARGET to VISUALLY_CONFIRMED
            *       - th4: number of visual detections associated to switch from VISUALLY_CONFIRMED to FRIEND_IN_SIGHT
            */                              
            void updateStatus(unsigned int th1, unsigned int th2, unsigned int th3, unsigned int th4);
            
		/**
		* \brief Gets the last position point
		*
		* Gets the last position point.
		*/		
		void getPositionEstimate(Cpoint3dCov & est);
		
		/**
		* \brief Adds a new estimate to the track
		*
		* Adds a new estimate to the track
		* Pushes back point/timeStamp to respective vectors. If track.size()>PERSON_PATH_SIZE, removes the former element in vectors
		*/		
// 		void addToTrack(const filterEstimate &est);
		
		/**
		* \brief Single person motion prediction
		*
		* Predicts the next position, given this->track and the delta time of prediction. 
		* Sets pediction to predictedPoint.
		*/		
// 		void predict(double dT, Cpoint3d & predictedPoint);
		
		/**
		* \brief Single person motion prediction
		*
		* Predicts the next position, given this->track, an extra point and the delta time of prediction.
		* The extra point replaces the last point in track. It will be typically a particle.
		* Returns the association probability and sets pediction to predictedPoint.
		*/		
// 		void predict(double dT, Cpoint3d & extraPoint, Cpoint3d & predictedPoint);
            
            /** \brief Resets matchScores table
            * 
            * Resets matchScores table
            * 
            */                                                                                                                                                                
            void resetMatchScores();

            /** \brief Resets association probability table
            * 
            * Resets association probability table
            * 
            */                                                                                                                                                                
            void resetAssociationProbs();

            /** \brief Resets association decision table
            * 
            * Resets association decision table
            * 
            */                                                                                                                                                                
            void resetAssociationDecisions();

            /** \brief Resizes association decision table
            * 
            * Resizes association decision table
            * 
            */                                                                                                                                                                
            void resizeAssociationDecisions(const unsigned int nLegsDet, const unsigned int nBodyDet, const unsigned int nFaceDet, const unsigned int nBody3dDet);                

		/**
		 * \brief Returns the association probability 
		 * 
		 * Returns the association probability. 
		 * It is computed as the mahalanobis distance between the last track estimate and the provided xy detection
		 */		
		double associationProb(Cpoint3d & pDet);
            
            /** \brief adds this->estimate to target track
            * 
            * adds this->estimate to target track
            * 
            **/
            void addEstimateToTrack();                
      
		/**
		 * \brief Prints target data to stdout
		 * 
		 * Prints target data to stdout
		 */
		virtual void print();
            
            /**
             * \brief Prints aProb and aDecision tables
             * 
             * Prints aProb and aDecision tables
             * 
             */            
            void printTables();
};
#endif
