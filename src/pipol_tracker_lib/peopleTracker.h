#ifndef peopleTracker_H
#define peopleTracker_H

//#include "personPfilter.h"
#include "personTarget.h"
#include "association_tree.h"
#include "association_nnls.h"
#include "geometry/point.h"
#include "geometry/line.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

//default parameter values
const double MINIMUN_DISTANCE_BETWEEN_PEOPLE = 0.3; //[m]
const double MINIMUM_ASSOCIATION_PROB = 1e-2;
const double MAX_DETECTION_DISTANCE_ACCEPTED = 5.0;  //[m]
const double MIN_DETECTION_DISTANCE_ACCEPTED = 0.1;  //[m]
const double MAX_DETECTION_AZIMUT_ACCEPTED = 60.*M_PI/180.; //[rad]
const unsigned int MAX_CONSECUTIVE_UNCORRECTED_ITERATIONS = 5;
const unsigned int MINIMUM_ITERATIONS_TO_BE_TARGET = 7;
const unsigned int MINIMUM_APPEARANCE_REGION_SIZE = 100;
const unsigned int MINIMUM_ITERATIONS_TO_BE_VISUALLY_CONFIRMED = 5;
const unsigned int MINIMUM_ITERATIONS_TO_BE_FRIEND = 20;
enum dataAssociationTypes {MHTREE = 0, NNLS};

/** \brief Tracker parameters
 * 
 * 
 * Struct grouping tracker parameters
 * 
 */
struct trackerParameters
{
    double minDistanceBetweenPeople;
    double minAssociationProb;
    double maxDetectionDistance;
    double minDetectionDistance;
    double maxDetectionAzimut;
    unsigned int maxConsecutiveUncorrected;
    unsigned int minIterationsToBeTarget;	
    unsigned int minAppearanceRegionSize;
    unsigned int iterationsToBeVisuallyConfirmed;
    unsigned int iterationsToBeFriend;        
};

/** \brief Struct grouping data to take associative decisions
 * 
 * Struct grouping data to take associative decisions between detections and targets
 * 
 **/
// struct decisionElement
// {
//     double aProb;
//     unsigned int targetIdx;
//     unsigned int detectionIdx;
//     bool assigned;
//     bool operator<(decisionElement de)
//     {
//         if (this->aProb < de.aProb) return 0;
//         else return 1;
//     }
// };

class CpeopleTracker
{
        protected:
            
        /** \brief Next target id 
        * 
        * Next target id to be assigned
        * 
        */
        unsigned int nextTargetId; 

        /** \brief Next detection id 
        * 
        * Next detection id to be assigned
        * 
        */
        unsigned int nextDetectionId[NUM_DETECTORS]; 

        /** \brief Target id of the follow me person
        * 
        * Target id of the follow me person, after an enrollment step
        * if 0, there is no follow me target present at the scene
        * 
        */
        int followMeTargetId; 

        /** \brief Initial TLD box
        * 
        * Bounidng box to initialize the TLD tracker
        * 
        **/
        cv::Rect_<int> tldBox;

        /** \brief List of leg detections
        * 
        * List of current leg detections received from the leg detector
        * 
        */		
        std::list<Cpoint3dObservation> laserDetSet;

        /** \brief List of body detections
        * 
        * List of current body detections received from the body detector
        * 
        */				
        std::list<CbodyObservation> bodyDetSet;

        /** \brief List of face detections
        * 
        * List of current face detections received from the face detector
        * 
        */                     
        std::list<CfaceObservation> faceDetSet;
        
        /** \brief List of body 3d detections
        * 
        * List of current body 3d detections received from the body 3d detector
        * 
        */         
        std::list<Cpoint3dObservation> body3dDetSet;            
        
        /** \brief Current TLD detection
        * 
        * Current TLD detection. TLD only outputs one detection, so it is not a list
        * 
        */                                 
        CbodyObservation tldDetection;
    
		/** \brief List of particle filters
		 * 
		 * For each target, there is a particle filter that estimates its state (position and velocity)
		 * 
		 */		
		std::list<CpersonTarget> targetList;
		
		/** \brief Tunning parameters used by this tracker
		 * 
		 * Tunning parameters used by this tracker
		 * 
		 */
		trackerParameters params;
		
        /** \brief Filter parameters
        * 
        * Filter parameters to be passed to new created filters
        * 
        */
        pFilterParameters filterParams;

        /** \brief Current image required for target appearance computation
        * 
        * Current image required for target appearance computation
        * 
        **/
        cv::Mat img;

        /** \brief Association tree
        * 
        * Asociation Mulit_Hypothesis tree. 
        * Used to stablish the most likely association event between detections and targets. 
        * 
        **/
        AssociationTree tree_;
        
        /** \brief Association Nearest Neighbor
        * 
        * Asociation Nearest Neighbor. 
        * Used to stablish the most likely association event between detections and targets. 
        * 
        **/
        AssociationNNLS nnls_;
                    
	public:
        CpeopleTracker();
        virtual ~CpeopleTracker();
        void setDefaultParameters();
        void setParameters(const trackerParameters & tp);
        void setFilterParameters(const pFilterParameters & pfp);
            
        /** \brief Sets followMeTargetId
            * 
            * sets followMeTargetId
            * 
            */         
        void setFollowMeTargetId(int fmtid);

        /** \brief Gets followMeTargetId
            * 
            * returns followMeTargetId
            * 
            */         
        int getFollowMeTargetId();

        /** \brief Checks if TLD can be initialized
            * 
            * Checks if TLD can be initialized. Heuristic condition
            * 
            */         
        bool checkTLDinit();

        /** \brief init TLD bounding box
            * 
            * Computes the initial TLD bounding box
            * 
            */                     
        void initTLD();

        /** \brief Gets TLD bounding box
            * 
            * Gets TLD bounding box
            * 
            **/
        void getTLDbb(unsigned int & bbx, unsigned int & bby, unsigned int & bbw, unsigned int & bbh);

        /** \brief Adds a leg detection
        * 
        * Adds a point-like detection to the leg detection set
        * 
        */		
        void addDetection(Cpoint3dObservation & newDet);
		
        /** \brief Adds a body detection
        * 
        * Adds a body detection to the body detection set
        * 
        */				
        void addDetection(CbodyObservation & newDet);

        /** \brief Adds a face detection
            * 
            * Adds a face detection to the face detection set
            * 
            */                     
        void addDetection(CfaceObservation & newDet);

        /** \brief Adds a body 3d detection
            * 
            * Adds a body 3d detection to the body3d detection set
            * 
            */                     
        void addDetectionBody3d(Cpoint3dObservation & newDet);            

        /** \brief Sets the TLD detection
            * 
            * Sets the TLD detection
            * 
            */                                 
        void setTLDdetection(CbodyObservation & newDet);

        /** \brief Gets TLD detection angle
            * 
            * Gets TLD detection angle
            * 
            */                                 
        void getTLDdetection(CbodyObservation & det);

        /** \brief Resets the detection set indicated by detId
        * 
        * Resets the detection set indicated by detId : {LEGS=0, BODY, FACE}
        * 
        */						
        void resetDetectionSets(int detId);

        /** \brief Computes Occlusion probabilities
            * 
            * Computes Occlusion probabilities between targets
            * 
            */
        void computeOcclusions();

        /** \brief Updates matching scores, association probabilities and association decisions
        * 
        * Updates matching scores, association probabilities and association decisions of all filters
        * with respect all current detections
        * 
        */
        void dataAssociation(unsigned int _type = MHTREE);        
        //void updateAssociationTablesOld();

        /** \brief Sets association of target tIdx with detection dIdx
        * 
        * Sets association of target tIdx with detection dIdx in the corresponding aDecisions array
        * detId = LEGS,BODY,FACE
        * 
        */
        void setAssociationDecision(unsigned int _detector_id, unsigned int _tj, unsigned int _di);
            
        /** \brief Updates target status
        * 
        * Updates target status by calling personTarget::updateStatus()
        * 
        **/
        void updateTargetStatus();

        /** \brief Creates new filters with unassociated detections
        * 
        * Creates new targets with unassociated detections
        * 
        **/            
        void createFilters();

        /** \brief Deletes filters according its status
        * 
        * Deletes targets according its status
        * 
        **/                        
        void deleteFilters();

        /** \brief Updates estimates of each filter
        * 
        * Updates estimates (mean and covariance) of each filter by computing weighted sample mean and covariance
        * 
        */
        void updateFilterEstimates();

        /** \brief Adds estimates to target tracks
        * 
        * Adds current estimate of each filter to the corresponding target track hsitory
        * 
        */
        void addEstimatesToTracks();		

        /** \brief Propagates particles according odometry reported motion
        * 
        * Propagates particles according odometry reported motion
        * 
        */             
        void propagateFilters(CodometryObservation & odoIncrement);

        /** \brief Propagates particles according their linear velocities
        * 
        * Propagates particles according their linear velocities
        * 
        */             
        void propagateFilters();
                            
        /** \brief Corrects particle weights
        * 
        * Corrects particle weights according detection sets and association tables of each target
        * 
        */             
        void correctFilters();
            
        /** \brief Resamples particle sets
        * 
        * Resamples particle sets
        * 
        */             
        void resampleFilters();

        /** \brief Returns a reference to the filter list
        * 
        * Returns a reference to the filter list. 
        * Useful for debugging and visualization
        * 
        */		
        std::list<CpersonTarget> & getTargetList();

        /** \brief Returns a reference to the laser detection set
        * 
        * Returns a reference to the laser detection set.
        * Useful for debugging and visualization
        * 
        */		
        std::list<Cpoint3dObservation> & getLaserDetSet();

        /** \brief Returns a reference to the body detection set
        * 
        * Returns a reference to the body detection set.
        * Useful for debugging and visualization
        * 
        */		
        std::list<CbodyObservation> & getBodyDetSet();

        /** \brief Returns a reference to the body3d detection set
        * 
        * Returns a reference to the body3d detection set.
        * Useful for debugging and visualization
        * 
        */         
        std::list<Cpoint3dObservation> & getBody3dDetSet();            

        /** \brief Sets current image
        * 
        * Sets current image
        * 
        **/
        void setCurrentImage(cv::Mat & inImg);

        /** \brief Gets current image
        * 
        * Gets current image. Display utility if marks has been drawn over the image.
        * 
        **/		
        void getCurrentImage(cv::Mat & outImg);

        /** \brief Marks body bounding boxes on image
            * 
            * Marks body bounding boxes on image
            * 
            */
        void markBodies();

        /** \brief Marks face bounding boxes on image
            * 
            * Marks face bounding boxes on image
            * 
            */
        void markFaces();

        /** \brief Prints detection sets
        * 
        * Prints detection sets
        * 
        **/            
        void printDetectionSets();
            
        /** \brief Prints people set
        * 
        * Prints people set
        * 
        **/                            
        void printPeopleSet();
};
#endif
