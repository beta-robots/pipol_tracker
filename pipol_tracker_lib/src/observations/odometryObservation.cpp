#include "odometryObservation.h"

CodometryObservation::CodometryObservation()
{
	deltaTrans = 0;
	deltaH = 0;
	deltaP = 0;
	deltaR = 0;
}

CodometryObservation::~CodometryObservation()
{
	//nothing to do
}

void CodometryObservation::setDeltaTrans(double dT)
{
	deltaTrans = dT;
}

void CodometryObservation::setDeltaH(double dH)
{
	deltaH = dH;
}

void CodometryObservation::setDeltaP(double dP)
{
	deltaP = dP;
}

void CodometryObservation::setDeltaR(double dR)
{
	deltaR = dR;
}

void CodometryObservation::accumDeltaTrans(double dT)
{
	deltaTrans += dT;
}

void CodometryObservation::accumDeltaH(double dH)
{
	deltaH += dH;
}

void CodometryObservation::accumDeltaP(double dP)
{
	deltaP += dP;
}

void CodometryObservation::accumDeltaR(double dR)
{
	deltaR += dR;
}

void CodometryObservation::resetDeltas()
{
	deltaTrans = 0;
	deltaH = 0;
	deltaP = 0;
	deltaR = 0;	
}

double CodometryObservation::getDeltaTrans()
{
	return deltaTrans;
}

double CodometryObservation::getDeltaH()
{
	return deltaH;
}

double CodometryObservation::getDeltaP()
{
	return deltaP;
}

double CodometryObservation::getDeltaR()
{
	return deltaR;
}

void CodometryObservation::printObservation()
{
	std::cout << "dT = " << deltaTrans << "; dH = " << deltaH << "; dP = " << deltaP << "; dR = " << deltaR << std::endl;
}
