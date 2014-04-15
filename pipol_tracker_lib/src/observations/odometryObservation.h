#ifndef odometryObservation_H
#define odometryObservation_H

#include "basicObservation.h"

/**
 *
 * \brief CodometryObservation implements 2D odometry observation
 * 
 * CodometryObservation implements 2D odometry observation
 * Data members are :
 *	deltaTrans: translational displacement [meters]
 *	deltaH: heading rotational displacement [rad]
 *	deltaP: pitch rotational displacement [rad]  
 *	deltaR: roll rotational displacement [rad]  
 * Robot axis are considered as X(front), Y(left), Z(up). H,P,R angles follow the euler-ZYX convention. 
*/
class CodometryObservation : public CbasicObservation
{
	protected:
		/**
		* \brief Translational displacement
		*
		* Translational displacement accumulated during the current elapsed time interval
		*
		*/		
		double deltaTrans;
		
		/**
		* \brief Heading rotational displacement
		*
		* Heading rotational displacement accumulated during the current elapsed time interval
		*
		*/		
		double deltaH;

		/**
		* \brief Pitch rotational displacement
		*
		* Pitch rotational displacement accumulated during the current elapsed time interval
		*
		*/		
		double deltaP;

		/**
		* \brief Roll rotational displacement
		*
		* Roll rotational displacement accumulated during the current elapsed time interval
		*
		*/		
		double deltaR;
		
	public:
		/**
		* \brief Translational percentual error
		*
		* Percentual error for translational increment
		*
		*/		
		double error_factor_XY;

		/**
		* \brief Heading percentual error
		*
		* Percentual error for heading increment
		*
		*/		
		double error_factor_H;

		/**
		* \brief Pitch percentual error
		*
		* Percentual error for pitch increment
		*
		*/		
		double error_factor_P;

		/**
		* \brief Roll percentual error
		*
		* Percentual error for roll increment
		*
		*/		
		double error_factor_R;
			
	public:
		/**
		* \brief Constructor
		*
		* Constructor
		*
		*/				
		CodometryObservation();

		/**
		* \brief Destructor
		*
		* Destructor
		*
		*/						
		~CodometryObservation();
		
		/**
		* \brief set deltaTrans
		*
		* Sets deltaTrans
		*
		*/			
		void setDeltaTrans(double dT);
		
		/**
		* \brief set deltaH
		*
		* Sets deltaH
		*
		*/			
		void setDeltaH(double dH);

		/**
		* \brief set deltaP
		*
		* Sets deltaP
		*
		*/			
		void setDeltaP(double dP);

		/**
		* \brief set deltaR
		*
		* Sets deltaR
		*
		*/			
		void setDeltaR(double dR);		
		
		/**
		* \brief Accumulate to deltaTrans
		*
		* Accumulates dT to deltaTrans
		*
		*/			
		void accumDeltaTrans(double dT);
		
		/**
		* \brief Accumulate to deltaH
		*
		* Accumulates dH to deltaH
		*
		*/			
		void accumDeltaH(double dH);

		/**
		* \brief Accumulate to deltaP
		*
		* Accumulates dP to deltaP
		*
		*/			
		void accumDeltaP(double dP);
		
		/**
		* \brief Accumulate to deltaR
		*
		* Accumulates dR to deltaR
		*
		*/			
		void accumDeltaR(double dR);
		
		/**
		* \brief Reset deltas
		*
		* Resets deltas
		*
		*/			
		void resetDeltas();

		/**
		* \brief Get deltaTrans
		*
		* Gets deltaTrans
		*
		*/			
		double getDeltaTrans();
		
		/**
		* \brief Get deltaH
		*
		* Gets deltaH
		*
		*/			
		double getDeltaH();

		/**
		* \brief Get deltaP
		*
		* Gets deltaP
		*
		*/			
		double getDeltaP();

		/**
		* \brief Get deltaR
		*
		* Gets deltaR
		*
		*/			
		double getDeltaR();
		
		/**
		* \brief Prints Observation
		*
		* Prints Observation
		*
		*/			
		void printObservation();
		
};	
#endif
