
#ifndef personParticle_H
#define personParticle_H

#include <iostream>
#include "geometry/point3d.h"

/**
 *
 * \brief Weighted samples on the space of 2D points
 * 
 * Particles are weighted samples on the space 2D position/velocity (z components are set to 0)
 * 
*/
class CpersonParticle
{
	protected:
		double ww;
		
	public:
		Cpoint3d position;
		Cpoint3d velocity;
		
	public:
		CpersonParticle();		
		CpersonParticle(double px, double py, double vx, double vy, double wt);
		virtual ~CpersonParticle();

		void setW(double wt);		
		double getW();
		void predict(double deltaT);
		void printParticle(bool endLine = true);
		bool operator<(CpersonParticle px); 
};
#endif		
