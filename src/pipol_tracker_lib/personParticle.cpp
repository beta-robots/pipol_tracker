#include "personParticle.h"

CpersonParticle::CpersonParticle()
{
	ww_ = 0;
}

CpersonParticle::CpersonParticle(double px, double py, double vx, double vy, double wt)
{
	position.setXYZ(px,py,0);
	velocity.setXYZ(vx,vy,0);
	ww_ = wt;
}

CpersonParticle::~CpersonParticle()
{
	//
}

void CpersonParticle::setW(double wt)
{ 
    ww_ = wt; 
}

double CpersonParticle::getW()
{ 
    return ww_; 
}

void CpersonParticle::predictStopped(double deltaT)
{
    //to do    
    //At the moment do nothing
}

void CpersonParticle::predictVlinear(double deltaT)
{
	position.setX( position.getX() + velocity.getX()*deltaT );
	position.setY( position.getY() + velocity.getY()*deltaT );
}

void CpersonParticle::printParticle(bool endLine)
{
	std::cout << "(" << position.getX() << "," << position.getY() << "," << velocity.getX() << "," << velocity.getY() << ")," << ww_ << "; ";
	if(endLine) {std::cout << std::endl;}
}

bool CpersonParticle::operator<(CpersonParticle px)
{
	if (this->ww_ < px.getW()) {return 0;}
	else {return 1;}
}
