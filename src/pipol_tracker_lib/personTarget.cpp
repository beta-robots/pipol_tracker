#include "personTarget.h"

CpersonTarget::CpersonTarget(unsigned int tid):CpersonParticleFilter()
{
        id = tid;
        pOcclusion = 0;
        status = ( 0 | CANDIDATE );
        tsInit.setToNow();
}
 
CpersonTarget::~CpersonTarget()
{
	track.clear();
}

void CpersonTarget::setId(unsigned int tid)
{
	id = tid;
}

unsigned int CpersonTarget::getId()
{
	return id;
}

double CpersonTarget::getTsInit()
{
      return tsInit.get();
}

unsigned int CpersonTarget::getStatus()
{
        return status;
}

unsigned int CpersonTarget::getMaxStatus()
{
      unsigned int ii, msk;
      
      msk = FACE_LEARNT;
      for (ii=0; ii<sizeof(status); ii++)
      {
            if (status & msk) break;
            else msk = msk>>1;
      }
      return msk;
}

void CpersonTarget::setStatus(statusMask sm, bool value)
{
      if ( value ) 
            this->status |= sm;
      else 
            this->status &= ~sm;
}

bool CpersonTarget::isStatus(statusMask sm)
{
      return bool(this->status & sm);
}

void CpersonTarget::updateStatus(unsigned int th1, unsigned int th2, unsigned int th3, unsigned int th4)
{
      
//       if ( ( countToBeRemoved > MAX_REMOVING_ITERATIONS ) || ( countConsecutiveUncorrected > th1*this->status ) )
      if ( ( countToBeRemoved > MAX_REMOVING_ITERATIONS ) || ( countConsecutiveUncorrected > th1 ) ) 
      {
            //this->status = TO_BE_REMOVED;
            setStatus(TO_BE_REMOVED,true);
      }
      
      else //no removing case
      {
//             switch(this->status)
//             {
//                   case CANDIDATE: 
//                         if ( countIterations > th2 ) this->status = LEGGED_TARGET;
//                         if ( this->estimate.position.getCovTrace() > 1 ) this->status = TO_BE_REMOVED;
//                         break;
//                         
//                   case LEGGED_TARGET:
//                         if ( countVisuallyCorrected > th3 ) this->status = VISUALLY_CONFIRMED;//this->print();
//                         if ( this->estimate.position.getCovTrace() > 1 ) this->status = TO_BE_REMOVED;
//                         break;
//                         
//                   case VISUALLY_CONFIRMED:
//                         if ( countVisuallyCorrected > th4 ) this->status = FRIEND_IN_SIGHT;
//                         break;
//                         
//                   case FRIEND_IN_SIGHT:
//                         // to do : some rule to transition to FRIEND_OUT_OF_RANGE
//                         // to do : friends are never removed !
//                         break;
//                         
//                   case FRIEND_OUT_OF_RANGE:
//                         // to do: some rule to come back to FRIEND_IN_SIGHT
//                         // to do: friends are never removed !
//                         break;
//                         
//                   default:
//                         break;
//             }
            if ( isStatus(VISUALLY_CONFIRMED) ) {
                  if ( countVisuallyCorrected > th4 ) setStatus(FRIEND_IN_SIGHT, true);                  
                  return;}
            if ( isStatus(LEGGED_TARGET) ) {
                  if ( countVisuallyCorrected > th3 ) setStatus(VISUALLY_CONFIRMED, true);
                  if ( this->estimate.position.getCovTrace() > 1 ) setStatus(TO_BE_REMOVED,true);
                  return;}
            if ( isStatus(CANDIDATE) ) {
                  if ( countIterations > th2 ) 
                  {
                        setStatus(LEGGED_TARGET,true);
                        setStatus(CANDIDATE,false);
                  }
                  if ( this->estimate.position.getCovTrace() > 1 ) setStatus(TO_BE_REMOVED,true);
                  return;}
      }
}

void CpersonTarget::getPositionEstimate(Cpoint3dCov & est)
{
	est = track.back().position;
}

// void CpersonTarget::addToTrack(const filterEstimate &est)
// {
// 	track.push_back(est);
// 	if ( track.size() > TRACK_SIZE )
// 	{
// 		track.pop_front();
// 	}
// 	
// }

// void CpersonTarget::predict(double dT, Cpoint3d & predictedPoint)
// {
// 	// to do 
// }	
// 
// void CpersonTarget::predict(double dT, Cpoint3d & extraPoint, Cpoint3d & predictedPoint)
// {
//         // to do
// }

void CpersonTarget::resetMatchScores()
{
        for (unsigned int ii=0; ii<NUM_DETECTORS; ii++) matchScores[ii].clear();        
}

void CpersonTarget::resetAssociationProbs()
{
        for (unsigned int ii=0; ii<NUM_DETECTORS; ii++) aProbs[ii].clear();
}

void CpersonTarget::resetAssociationDecisions()
{
        for (unsigned int ii=0; ii<NUM_DETECTORS; ii++) aDecisions[ii].clear();
}

void CpersonTarget::resizeAssociationDecisions(const unsigned int nLegsDet, const unsigned int nBodyDet, const unsigned int nFaceDet, const unsigned int nBody3dDet)
{
        this->resetAssociationDecisions();
        aDecisions[LEGS].resize(nLegsDet);
        aDecisions[BODY].resize(nBodyDet);
        aDecisions[FACE].resize(nFaceDet);
        aDecisions[BODY3D].resize(nBody3dDet);
}

double CpersonTarget::associationProb(Cpoint3d & pDet)
{
	double dM; //mahalanobis distance
	double prob; //probability result
	
	filterEstimate & lastEstimate = track.back();
	dM = lastEstimate.position.mahalanobisDistance2D(pDet);
	prob = erfc(dM/SQRT_2);
	
	return prob;
}

void CpersonTarget::addEstimateToTrack()
{
        track.push_back(this->estimate);
        if ( track.size() > TRACK_SIZE )
        {
                track.pop_front();
        }
}

void CpersonTarget::print()
{
	std::list<filterEstimate>::iterator iiE;
      
      CpersonParticleFilter::print();
	
// 	std::cout << "Target ID: " << id << std::endl;
//       std::cout << "countVisuallyCorrected: " << countVisuallyCorrected << std::endl;
// 	for (iiE=track.begin();iiE!=track.end();iiE++)
// 	{
// 		std::cout << "   "; iiE->ts.print(); std::cout << std::endl;
// 		std::cout << "   "; iiE->position.printPoint();
// 		std::cout << "   "; iiE->velocity.printPoint();
// 		std::cout << std::endl;
// 	}
      std::cout << std::endl;
}

void CpersonTarget::printTables()
{
      std::cout << "TARGET " << this->getId() << std::endl;
      for (unsigned int ii=0; ii<NUM_DETECTORS; ii++) 
      {
            std::cout << "   DETECTOR " << ii << std::endl;
            std::cout << "     aP: ";
            for (unsigned int jj=0; jj<aProbs[ii].size(); jj++) 
            {
                  std::cout << aProbs[ii].at(jj) << " ";
            }
            std::cout << std::endl;
            
            std::cout << "     aD: ";
            for (unsigned int jj=0; jj<aProbs[ii].size(); jj++) 
            {
                  std::cout << aDecisions[ii].at(jj) << " ";
            }
            std::cout << std::endl;
      }
}
