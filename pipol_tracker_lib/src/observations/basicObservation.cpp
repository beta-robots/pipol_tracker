#include "basicObservation.h"

CbasicObservation::CbasicObservation()
{
	status = 0; //indicates old data + uncorrect data 
}

CbasicObservation::~CbasicObservation()
{
	//nothing to do
}

void CbasicObservation::markAsNew()
{
	status |= NEW_DATA; 
}

void CbasicObservation::markAsOld()
{
	status &= ~NEW_DATA; 
}

void CbasicObservation::markAsCorrect()
{
	status |= CORRECT_DATA; 
}

void CbasicObservation::markAsUncorrect()
{
	status &= ~CORRECT_DATA; 
}

void CbasicObservation::markStatus(unsigned int bitWiseValue)
{
	status = bitWiseValue;
}

bool CbasicObservation::isNew()
{
	if (status && NEW_DATA) return true;
	else return false;
}

bool CbasicObservation::isCorrect()
{
	if (status && CORRECT_DATA) return true;
	else return false;
}

unsigned int CbasicObservation::getStatus()
{
	return status;
}

unsigned int CbasicObservation::getId()
{
	return id;
}

void CbasicObservation::setId(unsigned int newId)
{
	id = newId;
}
