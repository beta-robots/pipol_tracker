#include "personParticle.h"

CpersonParticle::CpersonParticle()
{
	ww = 0;
}

CpersonParticle::CpersonParticle(double px, double py, double vx, double vy, double wt)
{
	position.setXYZ(px,py,0);
	velocity.setXYZ(vx,vy,0);
	ww = wt;
}

CpersonParticle::~CpersonParticle()
{
	//
}

void CpersonParticle::setW(double wt){ ww = wt; }

double CpersonParticle::getW(){ return ww; }

void CpersonParticle::predict(double deltaT)
{
	position.setX( position.getX() + velocity.getX()*deltaT );
	position.setY( position.getY() + velocity.getY()*deltaT );
}

void CpersonParticle::printParticle(bool endLine)
{
	std::cout << "(" << position.getX() << "," << position.getY() << "," << velocity.getX() << "," << velocity.getY() << ")," << ww << "; ";
	if(endLine) {std::cout << std::endl;}
}

bool CpersonParticle::operator<(CpersonParticle px)
{
	if (this->ww < px.getW()) {return 0;}
	else {return 1;}
}
