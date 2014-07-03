#ifndef personParticleFilter_H
#define personParticleFilter_H

#include <list>
#include "geometry/point3d.h"
#include "geometry/point3dCov.h"
#include "observations/odometryObservation.h"
#include "observations/point3dObservation.h"
#include "observations/bodyObservation.h"
#include "observations/faceObservation.h"
#include "random/simpleRnd.h"
#include "personParticle.h"

//numerical constants
const double SQRT_2 = 1.4142;
const double ZERO_LIKELIHOOD = 1e-30;

//detector index
const unsigned int NUM_DETECTORS = 4; //TLD does not count as the other detectors 
enum detectorIds {LEGS=0, BODY, FACE, BODY3D, TLD};

//motion model modes
enum motionModes {MODE_STOP=0, MODE_GO};

//default parameter values
const unsigned int DEFAULT_NP = 300; //number of particles [#]
const double INIT_DELTA_XY = 0.2; //init sample range for x,y components [m]
const double INIT_DELTA_VXY = 1.5; //init sample range for vx,vy components [m/s]
const double SIGMA_FIXED_RESAMPLING_XY = 0.05; //resampling constants noise (sigma) for x,y components [m]
const double SIGMA_RATIO_RESAMPLING_VXY = 0.2; ////resampling constants noise (sigma) for vx,vy components [m/s]
const double SIGMA_MIN_RESAMPLING_VXY = 0.05; ////resampling constants noise (sigma) for vx,vy components [m/s]
const double MAX_SPEED_STOPPED = 0.1;
const double PROB_STOP2STOP = 0.9; //prob for a given STOPPED target to remain STOPPED in the next iteration, [0,1]
const double PROB_GO2GO = 0.8; //prob for a given GO target to remain GO in the next iteration, [0,1]
const double PERSON_RADIUS = 0.2; //Considered person radius [m]
const double MATCHING_LEGS_ALPHA = 0.1;//For legs likelihood, Difference between lik(0) and lik(border of "pass band") , in [0,0.5]
const double MATCHING_LEGS_BETA = 10;//For legs likelihood, Off-band expoenential decayment, set in [2,50]
const double MATCHING_BODY_ALPHA = 0.1;//For bearing likelihood, Difference between lik(0) and lik(border of "pass band") , in [0,0.5]
const double MATCHING_BODY_BETA = 10;//For bearing likelihood, Off-band expoenential decayment, set in [2,50]
const unsigned int MAX_REMOVING_ITERATIONS = 7;

/**
 * \brief Filter parameters
 * 
 * Struct grouping filter parameters
 * 
 */
struct pFilterParameters
{
        unsigned int numParticles;//particle set size, [#]
        double initDeltaXY; //initial resampling box size for x,y components, [m]
        double initDeltaVxy; //initial resampling box size for vx,vy components, [m/s]
        double sigmaResamplingXY; //resampling std dev for x,y components, [m]
        double sigmaRatioResamplingVxy; //resampling std dev for vx,vy components, [m/s]
        double sigmaMinResamplingVxy; //resampling std dev for vx,vy components, [m/s]
        double personRadius; //considered radius of the tracked person, [m]
        double matchingLegsAlpha; //For legs likelihood, Difference between lik(0) and lik(border of "pass band") , in [0,0.5]
        double matchingLegsBeta; //For legs likelihood, Off-band expoenential decayment, set in [2,50]
        double matchingBearingAlpha; //For bearing likelihood, Difference between lik(0) and lik(border of "pass band") , in [0,0.5]
        double matchingBearingBeta; //For bearing likelihood, Off-band expoenential decayment, set in [2,50]    
};

/**
 * \brief Filter constants derived from parameters
 * 
 * Struct grouping filter constants derived from parameters
 * 
 */
struct derivedConstants
{
        double legsK1;
};

/**
 *\brief Struct encapsulating the state of a target as
 * 
 * Struct encapsulating the filter estimate for a target with a time stamp
 * Particle set estimates 2D-position and 2D-velocity, so state space is 4D
 * z-components of position and velocity are not used.
 */
struct filterEstimate
{
        CtimeStamp ts;
        Cpoint3dCov position;
        Cpoint3dCov velocity;
};

class CpersonParticleFilter
{
      protected:
            /** \brief Particle set
            * 
            * Particle set. Sample set on 2D-position + 2D-velocity space.
            * 
            */
            std::list<CpersonParticle> pSet;
            
            /** \brief current estimate
            * 
            * Current estimate computed by updateEstimate() as the sample mean and covariance. 
            * By default, it is not pushed back to target.path. To do that, call to addEstimateToTargetPath()
            * 
            */
            filterEstimate estimate;
            
            /** \brief Time stamp of the last prior
            * 
            * Time stamp of the last prior
            * 
            */
            CtimeStamp tsLastPrior;                
            
            /** \brief Indicates motion mode
            * 
            * Indicates whether the target is in STOP or GO mode
            * 
            **/
            unsigned int motionMode;            
            
            /** \brief Iteration counter
            * 
            * Iteration counter
            * 
            */
            unsigned int countIterations;
            
            /** \brief Counter of consecutive uncorrected iterations
            * 
            * Counter of consecutive uncorrected iterations.
            * Used to manage target removal.
            * 
            */
            unsigned int countConsecutiveUncorrected;

            /** \brief Counter of visually corrected iterations
            * 
            * Counter of visually corrected iterations. Visual detector is teh BODY one.
            * Used to manage target removal.
            * 
            */
            unsigned int countVisuallyCorrected;
            
            /** \brief Counts target reomving iterations
            * 
            * Counts target reomving iterations
            * 
            **/
            unsigned int countToBeRemoved;
                        
            /** \brief Tunning parameters used by this filter
            * 
            * Tunning parameters used by this filter
            * 
            */
            pFilterParameters params;

            /** \brief Derived parameters from tunning parameters
            * 
            * Derived parameters from tunning parameters.
            * 
            */
            derivedConstants dConstants;
                
      public:
            /** \brief Default constructor
            * 
            * Default constructor
            * 
            */                
            CpersonParticleFilter();

            /** \brief Default destructor
            * 
            * Default destructor
            * 
            */                                
            virtual ~CpersonParticleFilter();
            
            /** \brief Sets filter parameters to default values
            * 
            * Sets filter parameters to default values
            * 
            */                                
            void setDefaultParameters();
            
            /** \brief Sets filter parameters
            * 
            * Sets filter parameters
            * 
            */                                                
            void setParameters(const pFilterParameters & pfp);
            
            /** \brief Gets number of particles
            * 
            * Gets number of particles used by this filter 
            * 
            */                                                                
            unsigned int getNP();
            
            /** \brief Gets personRadius
            * 
            * Gets person radius used by this filter 
            * 
            */                                                                
            double getPersonRadius();                

            /** \brief Gets iteration counter
            * 
            * Gets iteration counter
            * 
            */                                                                
            unsigned int getIterations();
            
            /** \brief Gets consecutive uncorrected iteration counter
            * 
            * Gets consecutive uncorrected iteration counter
            * 
            */                                                                
            unsigned int getConsecutiveUncorrected();
            
            /** \brief Gets the last position estimate
            * 
            * Gets the last position estimate
            * 
            */                                                                                
            void getPositionEstimate(Cpoint3dCov & est);
            
            /** \brief Gets the current filter estimate
            * 
            * Gets the current filter estimate
            * 
            */                                                                                                
            void getEstimate(filterEstimate & est);
            
            /** \brief Returns a reference to the particle set
            * 
            * Returns a reference to the particle set
            * 
            */                                                                                                                                                
            std::list<CpersonParticle> & getParticleSet();
            
            /** \brief Updates iteration counters
            * 
            * Increments iteration counter and updates consecutiveUncorrected ans visuallyCorrected
            * Inputs are 
            * 
            */                                                                                                                                                
            void updateCounters(bool corrected, bool visual, bool occluded);
            
            void incrementToBeRemovedCounter();

            void resetToBeRemovedCounter();
            
            /** \brief Initializes particle set
            * 
            * Initializes particle set
            * 
            */                                                                                                                                                                                
            void init(Cpoint3dObservation & pDet);
            
            /** \brief Predicts particle positions from odometry motion
            * 
            * Predicts particle positions from odometry motion
            * 
            **/
            void predictPset(CodometryObservation & odo);
            
            /** \brief Predicts particle positions from their linear velocities
            * 
            * Predicts particle positions from their linear velocities
            * 
            **/                
            void predictPset();
            
            void computeWeights(Cpoint3dObservation & pDet, vector<double> & ww);
            void computeWeights(CbodyObservation & pDet, vector<double> & ww);
            void computeWeights(CfaceObservation & pDet, vector<double> & ww);
            void computeWeightsBody3d(Cpoint3dObservation & pDet, vector<double> & ww);
            void setWeights(const vector<double> & ww);
            void normalizePset();
            void resamplePset();
            void updateEstimate();
            void setMotionMode();
            unsigned int getMotionMode();
            double legMatchingFunction(Cpoint3d & p1);
            double legMatchingFunction(Cpoint3d & p1, Cpoint3d & p2);
            double bodyMatchingFunction(Cpoint3d & pD);
            double bodyMatchingFunction(Cpoint3d & pD, Cpoint3d & pT);
            double faceMatchingFunction(Cpoint3d & pD);
            double faceMatchingFunction(Cpoint3d & pD, Cpoint3d & pT);
            double body3dMatchingFunction(Cpoint3d & pD);
            double body3dMatchingFunction(Cpoint3d & pD, Cpoint3d & pT);            
            virtual void print(unsigned int tId=0);
            void printParticleSet();
};
#endif
