#include "peopleTracker.h"

CpeopleTracker::CpeopleTracker()
{
	nextTargetId = 1;
      followMeTargetId = -1;//initially there is no folloMe target Id 
	for (unsigned int ii=0; ii<NUM_DETECTORS; ii++) nextDetectionId[ii] = 1;
	setDefaultParameters();
}

CpeopleTracker::~CpeopleTracker()
{
	laserDetSet.clear();
	bodyDetSet.clear();
	targetList.clear();
}		

void CpeopleTracker::setDefaultParameters()
{
      //tracker params
	params.minDistanceBetweenPeople = MINIMUN_DISTANCE_BETWEEN_PEOPLE;
	params.minAssociationProb = MINIMUM_ASSOCIATION_PROB;
	params.maxDetectionDistance = MAX_DETECTION_DISTANCE_ACCEPTED;
	params.minDetectionDistance = MIN_DETECTION_DISTANCE_ACCEPTED;
      params.maxDetectionAzimut = MAX_DETECTION_AZIMUT_ACCEPTED; 
	params.maxConsecutiveUncorrected = MAX_CONSECUTIVE_UNCORRECTED_ITERATIONS;
	params.minIterationsToBeTarget = MINIMUM_ITERATIONS_TO_BE_TARGET;	
      params.minAppearanceRegionSize = MINIMUM_APPEARANCE_REGION_SIZE;
      params.iterationsToBeVisuallyConfirmed = MINIMUM_ITERATIONS_TO_BE_VISUALLY_CONFIRMED;
      params.iterationsToBeFriend = MINIMUM_ITERATIONS_TO_BE_FRIEND;
        
      //particle filter params  (for each created filter)
	filterParams.numParticles = DEFAULT_NP;
	filterParams.initDeltaXY = INIT_DELTA_XY;
	filterParams.initDeltaVxy = INIT_DELTA_VXY;
	filterParams.sigmaResamplingXY = SIGMA_FIXED_RESAMPLING_XY;
	filterParams.sigmaResamplingVxy = SIGMA_FIXED_RESAMPLING_VXY;
	filterParams.personRadius = PERSON_RADIUS;
	filterParams.matchingLegsAlpha = MATCHING_LEGS_ALPHA;
	filterParams.matchingLegsBeta = MATCHING_LEGS_BETA;
	filterParams.matchingBearingAlpha = MATCHING_BODY_ALPHA;
	filterParams.matchingBearingBeta = MATCHING_BODY_BETA;
}

void CpeopleTracker::setParameters(const trackerParameters & tp)
{
	params.minDistanceBetweenPeople = tp.minDistanceBetweenPeople;
	params.minAssociationProb = tp.minAssociationProb;
	params.maxDetectionDistance = tp.maxDetectionDistance;
	params.minDetectionDistance = tp.minDetectionDistance;
      params.maxDetectionAzimut = tp.maxDetectionAzimut;
	params.maxConsecutiveUncorrected = tp.maxConsecutiveUncorrected;
	params.minIterationsToBeTarget = tp.minIterationsToBeTarget;
      params.minAppearanceRegionSize = tp.minAppearanceRegionSize;
      params.iterationsToBeVisuallyConfirmed = tp.iterationsToBeVisuallyConfirmed;
      params.iterationsToBeFriend = tp.iterationsToBeFriend;
      
      std::cout << std::endl << 
            "TRACKER PARAMETERS ***********************************" << std::endl << 
            "params.minDistanceBetweenPeople: " << params.minDistanceBetweenPeople << std::endl <<
            "params.minAssociationProb: " << params.minAssociationProb << std::endl <<
            "params.maxDetectionDistance: " << params.maxDetectionDistance << std::endl <<
            "params.minDetectionDistance: " << params.minDetectionDistance << std::endl <<
            "params.maxDetectionAzimut: " << params.maxDetectionAzimut << std::endl <<
            "params.maxConsecutiveUncorrected: " << params.maxConsecutiveUncorrected << std::endl <<
            "params.minIterationsToBeTarget: " << params.minIterationsToBeTarget << std::endl <<
            "params.minAppearanceRegionSize: " << params.minAppearanceRegionSize << std::endl <<
            "params.iterationsToBeVisuallyConfirmed: " << params.iterationsToBeVisuallyConfirmed << std::endl <<
            "params.iterationsToBeFriend: " << params.iterationsToBeFriend << std::endl << 
            "******************************************************" << std::endl << std::endl; 
}

void CpeopleTracker::setFilterParameters(const pFilterParameters & pfp)
{
	filterParams.numParticles = pfp.numParticles;
	filterParams.initDeltaXY = pfp.initDeltaXY;
	filterParams.initDeltaVxy = pfp.initDeltaVxy;
	filterParams.sigmaResamplingXY = pfp.sigmaResamplingXY;
	filterParams.sigmaResamplingVxy = pfp.sigmaResamplingVxy;
	filterParams.personRadius = pfp.personRadius;
	filterParams.matchingLegsAlpha = pfp.matchingLegsAlpha; 
	filterParams.matchingLegsBeta = pfp.matchingLegsBeta; 
	filterParams.matchingBearingAlpha = pfp.matchingBearingAlpha;
	filterParams.matchingBearingBeta = pfp.matchingBearingBeta;
}

// void CpeopleTracker::setOnBoardCamPose(Cposition3d & camP)
// {
//       this->camINbase = camP;
//       std::cout << "CpeopleTracker: Camera pose in base link is: "; 
//       this->camINbase.printPosition();
// }

void CpeopleTracker::setOnBoardCamCalMatrix()
{
      std::cout << "CpeopleTracker: Camera calibration matrix is: " << std::endl; 
}

void CpeopleTracker::setFollowMeTargetId(int fmtid)
{
      this->followMeTargetId = fmtid;
      std::cout << "FOLLOW ME TARGET ID SET TO: " << this->followMeTargetId << std::endl;
}

int CpeopleTracker::getFollowMeTargetId()
{
      return this->followMeTargetId;
}


bool CpeopleTracker::checkTLDinit()
{
      std::list<CpersonTarget>::iterator iiT;
      CtimeStamp tsNow;
      bool tldInitConditionMeet = false;
      
      //check init condition over the set of targets that are VISUALLY_CONFIRMED
      tsNow.setToNow();
      for (iiT=targetList.begin();iiT!=targetList.end();iiT++)
      {
            if ( ( iiT->isStatus(VISUALLY_CONFIRMED) ) && ( (tsNow.get()-iiT->getTsInit()) > 9 ) )
            {
                 tldInitConditionMeet = true;
                 //followMeTargetId = iiT->getId();
                 break;
            }
      }
      
      return tldInitConditionMeet;
}     
      
void CpeopleTracker::initTLD()
{
      std::list<CpersonTarget>::iterator iiT;
      filterEstimate tgEst;
      Cpoint3d tgPoint;
      
      //camera intrinsic params
      cv::Matx33d camK(525, 0, 319.5,
                       0, 525, 239.5,
                       0, 0, 1);
      
      //select followMeTargetId
      for (iiT=targetList.begin();iiT!=targetList.end();iiT++)
      {
            if ( iiT->getId() == this->followMeTargetId ) break;
      }
      
      //gets target point of iiT
      iiT->getEstimate(tgEst);
      tgPoint = tgEst.position;
      tgPoint.setZ(1.2);//120cm will be the central point of the bounding box
           
      // Computes the estimated bbox to initialize TLD. Now it is hardcoded ... to do
      tldBox.x = 140;
      tldBox.y = 140;
      tldBox.width = 140;
      tldBox.height = 200;
}

void CpeopleTracker::getTLDbb(unsigned int & bbx, unsigned int & bby, unsigned int & bbw, unsigned int & bbh)
{
      bbx = tldBox.x;
      bby = tldBox.y;
      bbw = tldBox.width;
      bbh = tldBox.height;
}

void CpeopleTracker::addDetection(Cpoint3dObservation & newDet)
{
	double dist;
      double azimut;
	
	dist = newDet.point.norm();
      azimut = fabs(atan2(newDet.point.getY(),newDet.point.getX()));
	if ( (dist >= params.minDetectionDistance) && (dist <= params.maxDetectionDistance) && (azimut < params.maxDetectionAzimut) )
      //if ( (dist >= params.minDetectionDistance) && (dist <= params.maxDetectionDistance) )
	{
		newDet.setId(nextDetectionId[LEGS]);
		laserDetSet.push_back(newDet);
		nextDetectionId[LEGS]++;
	}
}

void CpeopleTracker::addDetection(CbodyObservation & newDet)
{
	newDet.setId(nextDetectionId[BODY]);
	bodyDetSet.push_back(newDet);
	nextDetectionId[BODY]++;
}

void CpeopleTracker::addDetection(CfaceObservation & newDet)
{
      newDet.setId(nextDetectionId[FACE]);
      faceDetSet.push_back(newDet);
      nextDetectionId[FACE]++;      
}

void CpeopleTracker::setTLDdetection(CbodyObservation & newDet)
{
      tldDetection.timeStamp.set(newDet.timeStamp.get());
      tldDetection.direction = newDet.direction;
      tldDetection.rgbEigen = newDet.rgbEigen;
      tldDetection.bbX = newDet.bbX;
      tldDetection.bbY = newDet.bbY;
      tldDetection.bbW = newDet.bbW;
      tldDetection.bbH = newDet.bbH;
}

void CpeopleTracker::getTLDdetection(CbodyObservation & det)
{
      det.timeStamp.set(tldDetection.timeStamp.get());
      det.direction = tldDetection.direction;
      det.bbX = tldDetection.bbX;
      det.bbY = tldDetection.bbY;
      det.bbW = tldDetection.bbW;
      det.bbH = tldDetection.bbH;
}

// double CpeopleTracker::getTldAngle()
// {
//       //std::cout << "tldX: " << tldDetection.direction.getX() << "; tldY: " << tldDetection.direction.getY() << std::endl;
//       return atan2(tldDetection.direction.getY(),tldDetection.direction.getX());
// }
      
void CpeopleTracker::resetDetectionSets(int detId)
{
      switch (detId)
      {
            case LEGS: 
                  laserDetSet.clear(); 
                  nextDetectionId[LEGS] = 1;
                  break;
            case BODY: 
                  bodyDetSet.clear(); 
                  nextDetectionId[BODY] = 1;
                  break;
            case FACE: 
                  faceDetSet.clear(); 
                  nextDetectionId[FACE] = 1;
                  break;
            case TLD:
                  tldDetection.bbW = 0;
                  break;
            default:
                  break;
      }
}

void CpeopleTracker::computeOcclusions()
{
      std::list<CpersonTarget>::iterator iiT, jjT;
      Cline iiLine;
      Cpoint jjTpoint;
      filterEstimate fEst;
      bool occlusionFound;
      double ddij;
           
      //for each target iiT, compute potential occlusions caused by other targets jjT
      for (iiT=targetList.begin();iiT!=targetList.end();iiT++)
      {
            //builds iiLine, from the robot to target ii
            iiT->getEstimate(fEst);
            iiLine.set_point_coord(0,0,fEst.position.getX(),fEst.position.getY());
            
            //resets flag
            occlusionFound = false;
            
            //jjT targets will cause potential occlusions
            for (jjT=targetList.begin();jjT!=targetList.end();jjT++)
            {
                  if (iiT!=jjT)
                  {
                        jjT->getEstimate(fEst);
                        jjTpoint.set_point(fEst.position.getX(),fEst.position.getY());
                        ddij = iiLine.d2point(&jjTpoint);
                        if ( ddij < jjT->getPersonRadius()*1.2 )
                        {
                              occlusionFound = true;
                              break; //iterate to the next target iiT if an occlusion is found
                        }
                  }
            }
                  
            //set status and probability of occlusion for target iiT (ToDo: improve step function with a smoother one)
            if ( occlusionFound ) 
            {
                  iiT->pOcclusion = 1; //0.9
                  iiT->setStatus(IN_OCCLUSION,true);
            }
            else 
            {
                  iiT->pOcclusion = 0;
                  iiT->setStatus(IN_OCCLUSION,false);
            }
      }
}

void CpeopleTracker::updateAssociationTables()
{
	std::list<Cpoint3dObservation>::iterator jjL, kkL, llL;
	std::list<CbodyObservation>::iterator jjB, kkB;
      std::list<CfaceObservation>::iterator jjF, kkF;
	std::list<CpersonTarget>::iterator iiT, llT;
	double matchingValue, assocProb;
    unsigned int ii = 0;
    unsigned int jj = 0;
    unsigned int ll = 0;
	decisionElement de;
	std::list<decisionElement> deList;
	std::list<decisionElement>::iterator k1E, k2E;

	//resets matching and association tables
	for (iiT=targetList.begin();iiT!=targetList.end();iiT++)
	{
		iiT->resetMatchScores();
		iiT->resetAssociationProbs();
		iiT->resetAssociationDecisions();
	}
	
	//Resizes aDecisions vectors
	for (iiT=targetList.begin();iiT!=targetList.end();iiT++)
		iiT->resizeAssociationDecisions(laserDetSet.size(), bodyDetSet.size(), faceDetSet.size());	
	
	
	//1A. LEG DETECTOR. Matching: for each LEG detection, compute matching score to each target
	for (jjL=laserDetSet.begin();jjL!=laserDetSet.end();jjL++)
	{
		for (iiT=targetList.begin();iiT!=targetList.end();iiT++)
		{
			matchingValue = iiT->legMatchingFunction(jjL->point);
			iiT->matchScores[LEGS].push_back(matchingValue);
			//std::cout << "LD" << iiD->getId() << ", F" << iiT->getTargetId() << ": match: " << matchingValue << std::endl;
		}
	}
	
      //1B. LEG DETECTOR. Association Probability: for each leg detection, computes association probability to each target
//       for (jjL=laserDetSet.begin();jjL!=laserDetSet.end();jjL++)
//       {
//             for (iiT=targetList.begin();iiT!=targetList.end();iiT++)
//             {
//                   assocProb = 1;
//                   for (kkL=laserDetSet.begin(),ii=0;kkL!=laserDetSet.end();kkL++,ii++)
//                   {
//                         if(kkL!=jjL) //product of all negative matching events
//                         {
//                               assocProb *= ( 1 - iiT->matchScores[LEGS].at(ii));
//                         }
//                         else //contribution of the positive event
//                         {
//                               assocProb *= iiT->matchScores[LEGS].at(ii);
//                         }
//                   }
//                   iiT->aProbs[LEGS].push_back(assocProb);
//                   //iiT->aProbs[LEGS].push_back((1-iiT->pOcclusion)*assocProb);
// 		}
// 	}
      for (iiT=targetList.begin();iiT!=targetList.end();iiT++)
      {
            for (jjL=laserDetSet.begin(),jj=0;jjL!=laserDetSet.end();jjL++,jj++)
            {
                  assocProb = 1;
                  for (llL=laserDetSet.begin(),ll=0;llL!=laserDetSet.end();llL++,ll++)
                  {
                        if( llL==jjL ) //contribution of the positive event
                        {
                              assocProb *= iiT->matchScores[LEGS].at(ll);
                        }
                        else //product of all negative matching events
                        {
                              assocProb *= ( 1 - iiT->matchScores[LEGS].at(ll));
                        }
                  }
                  for (llT=targetList.begin();llT!=targetList.end();llT++)
                  {
                        if( llT!=iiT ) //product of all negative matching events
                        {
                              assocProb *= ( 1 - llT->matchScores[LEGS].at(jj));
                        }
                  }
                  iiT->aProbs[LEGS].push_back(assocProb);
            }
      }
      
      //1C. LEG DETECTOR. Association decision: for each leg detection decides to which target is associated
      //1C-a. First, build a list
      deList.clear();
      for (iiT=targetList.begin(),ii=0;iiT!=targetList.end();iiT++, ii++)
      {
            for (jjL=laserDetSet.begin(),jj=0;jjL!=laserDetSet.end();jjL++, jj++)
            {
                  if (iiT->aProbs[LEGS].at(jj) > params.minAssociationProb)
                  {
                        de.aProb = iiT->aProbs[LEGS].at(jj);
                        de.targetIdx = ii;
                        de.detectionIdx = jj;
                        de.assigned = false;
                        deList.push_back(de);
                  }
            }
      }

      //1C-b. Sorts the list following aProbs field
      deList.sort();
		
	//1C-c. Sets Association decisions starting from the highest probabilistic event 
	for (k1E = deList.begin(); k1E != deList.end(); k1E++ )
	{
		if ( !k1E->assigned )
		{
			ii = k1E->targetIdx;
			jj = k1E->detectionIdx;
			setAssociationDecision(LEGS,ii,jj); //sets association decision
		}
		for (k2E = k1E; k2E != deList.end(); k2E++ )
		{
			if ( (k2E->targetIdx == ii) || (k2E->detectionIdx == jj) )
			{
				k2E->assigned = true; //Mark target ii and detection jj as assigned
			}
		}
	}
	
	//2A. BODY DETECTOR. Matching: for each BODY detection, compute matching score to each filter
	for (jjB=bodyDetSet.begin();jjB!=bodyDetSet.end();jjB++)
	{
		for (iiT=targetList.begin();iiT!=targetList.end();iiT++)
		{
			matchingValue = iiT->bodyMatchingFunction(jjB->direction);
			iiT->matchScores[BODY].push_back(matchingValue);
		}
	}
	
	//2B. BODY DETECTOR. Association Probability: for each body detection, computes association probability to each filter
	for (jjB=bodyDetSet.begin();jjB!=bodyDetSet.end();jjB++)
	{
		for (iiT=targetList.begin();iiT!=targetList.end();iiT++)
		{
			assocProb = 1;
			for (kkB=bodyDetSet.begin(),ii=0;kkB!=bodyDetSet.end();kkB++,ii++)
			{
				if(kkB!=jjB) //product of all negative matching events
				{
					assocProb *= ( 1 - iiT->matchScores[BODY].at(ii));
				}
				else //contribution of the positive event
				{
					assocProb *= iiT->matchScores[BODY].at(ii);
				}
			}
			iiT->aProbs[BODY].push_back((1-iiT->pOcclusion)*assocProb);
		}
	}

	//2C. BODY DETECTOR. Association decision: for each body detection decides to which target is associated
	//2C-a. First, build a list
	deList.clear();
	for (iiT=targetList.begin(),ii=0;iiT!=targetList.end();iiT++, ii++)
	{
		for (jjB=bodyDetSet.begin(),jj=0;jjB!=bodyDetSet.end();jjB++, jj++)
		{
			if (iiT->aProbs[BODY].at(jj) > params.minAssociationProb)
			{
				de.aProb = iiT->aProbs[BODY].at(jj);
				de.targetIdx = ii;
				de.detectionIdx = jj;
				de.assigned = false;
				deList.push_back(de);
                        
                        //debug 
                        //std::cout << "LINE: "<< __LINE__  << ", aProb: " << de.aProb << std::endl;
                        //std::cout << "params.minAssociationProb: " << params.minAssociationProb << std::endl;
			}
		}
	}
	
	//2C-b. Sorts the list following aProbs field
	deList.sort();
	
	//2C-c. Sets Association decisions starting from the highest probabilistic event 
	for (k1E = deList.begin(); k1E != deList.end(); k1E++ )
	{
		if ( !k1E->assigned )
		{
			ii = k1E->targetIdx;
			jj = k1E->detectionIdx;
			setAssociationDecision(BODY,ii,jj); //sets association decision
                  //std::cout << "BODY associated: t" << ii << ", d" << jj << std::endl;
		}
		for (k2E = k1E; k2E != deList.end(); k2E++ )
		{
			if ( (k2E->targetIdx == ii) || (k2E->detectionIdx == jj) )
			{
				k2E->assigned = true; //Mark target ii and detection jj as assigned
			}
		}
	}

      //3A. FACE DETECTOR. Matching: for each FACE detection, compute matching score to each target
      for (jjF=faceDetSet.begin();jjF!=faceDetSet.end();jjF++)
      {
            for (iiT=targetList.begin();iiT!=targetList.end();iiT++)
            {
                  matchingValue = iiT->faceMatchingFunction(jjF->faceLoc);
                  iiT->matchScores[FACE].push_back(matchingValue);
            }
      }

      //3B. FACE DETECTOR. Association Probability: for each face detection, computes association probability to each target
      for (jjF=faceDetSet.begin();jjF!=faceDetSet.end();jjF++)
      {
            for (iiT=targetList.begin();iiT!=targetList.end();iiT++)
            {
                  assocProb = 1;
                  for (kkF=faceDetSet.begin(),ii=0;kkF!=faceDetSet.end();kkF++,ii++)
                  {
                        if(kkF!=jjF) //product of all negative matching events
                        {
                              assocProb *= ( 1 - iiT->matchScores[FACE].at(ii));
                        }
                        else //contribution of the positive event
                        {
                              assocProb *= iiT->matchScores[FACE].at(ii);
                        }
                  }
                  iiT->aProbs[FACE].push_back(assocProb);
            }
      }

      //3C. FACE DETECTOR. Association decision: for each face detection decides to which target is associated
      //3C-a. First, build a list
      deList.clear();
      for (iiT=targetList.begin(),ii=0;iiT!=targetList.end();iiT++, ii++)
      {
            for (jjF=faceDetSet.begin(),jj=0;jjF!=faceDetSet.end();jjF++, jj++)
            {
                  if (iiT->aProbs[FACE].at(jj) > params.minAssociationProb)
                  {
                        de.aProb = iiT->aProbs[FACE].at(jj);
                        de.targetIdx = ii;
                        de.detectionIdx = jj;
                        de.assigned = false;
                        deList.push_back(de);
                  }
            }
      }

      //3C-b. Sorts the list following aProbs field
      deList.sort();

      //3C-c. Sets Association decisions starting from the highest probabilistic event 
      for (k1E = deList.begin(); k1E != deList.end(); k1E++ )
      {
            if ( !k1E->assigned )
            {
                  ii = k1E->targetIdx;
                  jj = k1E->detectionIdx;
                  setAssociationDecision(FACE,ii,jj); //sets association decision
                  //std::cout << "FACE associated: t" << ii << ", d" << jj << std::endl; 
            }
            for (k2E = k1E; k2E != deList.end(); k2E++ )
            {
                  if ( (k2E->targetIdx == ii) || (k2E->detectionIdx == jj) )
                  {
                        k2E->assigned = true; //Mark target ii and detection jj as assigned
                  }
            }
      }

	//frees memory allocated
	deList.clear();
      
      //debug 
      //for (iiT=targetList.begin();iiT!=targetList.end();iiT++) iiT->printTables();
}

void CpeopleTracker::setAssociationDecision(unsigned int detId, unsigned int tIdx, unsigned int dIdx)
{
	std::list<CpersonTarget>::iterator iiT;
	unsigned int ii;
	
	for (iiT=targetList.begin(),ii=0; iiT!=targetList.end(); iiT++,ii++)
	{
		if (ii == tIdx) iiT->aDecisions[detId].at(dIdx) = true;
            
            //debugging
//             if (detId == BODY) 
//             {
//                   std::cout << "BODY associated: t" << iiT->getId() << ", d" << dIdx << std::endl;
//                   std::cout << "   Prob: " << iiT->aProbs[detId].at(dIdx) << std::endl;
//             }
//             if (detId == FACE) std::cout << "FACE associated: t" << iiT->getId() << ", d" << dIdx << std::endl;
	}
}

void CpeopleTracker::updateTargetStatus()
{
        std::list<CpersonTarget>::iterator iiT;
        for (iiT=targetList.begin(); iiT!=targetList.end(); iiT++)
                iiT->updateStatus( params.maxConsecutiveUncorrected, params.minIterationsToBeTarget, 
                                   params.iterationsToBeVisuallyConfirmed, params.iterationsToBeFriend );
}
	
void CpeopleTracker::createFilters()
{
      CpersonTarget *newTarget;
      std::list<Cpoint3dObservation>::iterator iiD;
      std::list<CpersonTarget>::iterator jjT;
      std::list<CpersonTarget> newFilters;
      double assocProb;
      bool associated;
      unsigned int ii;
      Cline iiLine;
      Cpoint jjTpoint;
      filterEstimate fEst;            
      double ddij;
      bool detectionInOcclusion;

      //check for unassociated detections of LEG detector & create new filters for unassociated LEG detections
      //createNewFilters();
      for (iiD=laserDetSet.begin(),ii=0;iiD!=laserDetSet.end();iiD++,ii++)
      {
            associated = false;
            for (jjT=targetList.begin();jjT!=targetList.end();jjT++)
            {
                  assocProb = jjT->aProbs[LEGS].at(ii);
                  //std::cout << "Det: " << ii << "; Filter: " << jjT->getTargetId() << std::endl;
                  //std::cout << "    assocP = " << assocProb << std::endl;
                  if ( assocProb > params.minAssociationProb ) //iiD detection has been associated, at least, to filter jjT
                  {
                        associated = true;
                        break;
                  }
            }
            if (!associated) //iiD has not been associated to any target/filter, so we launch a new filter
            {
                  //first check if some target is causing occlusion of iiD detection
                  detectionInOcclusion = false;
                  iiLine.set_point_coord(0,0,iiD->point.getX(),iiD->point.getY());
                  for (jjT=targetList.begin();jjT!=targetList.end();jjT++)
                  {
                        jjT->getEstimate(fEst);
                        jjTpoint.set_point(fEst.position.getX(),fEst.position.getY());
                        ddij = iiLine.d2point(&jjTpoint);
                        if ( ddij < jjT->getPersonRadius()*1.2 )
                        {
                              detectionInOcclusion = true;
                              break; //iterate to the next target iiT if an occlusion is found
                        }
                  }
                  
                  //if no occlusion, then initialize a new target by creating a new filter                  
                  if (!detectionInOcclusion)
                  {
                        newTarget = new CpersonTarget(nextTargetId);
                        newTarget->setParameters(filterParams);
                        nextTargetId++;
                        newTarget->init(*iiD);
                        newTarget->updateEstimate();
                        newFilters.push_back(*newTarget);
                        delete newTarget;
                        //std::cout << "createFilters(): new target with id: "<< newTarget->getTargetId() << std::endl;
                  }
            }
      }	
      targetList.splice(targetList.end(),newFilters);//adds new filters to the targetList maintained by this Tracker
      newFilters.clear();
}

void CpeopleTracker::deleteFilters()
{
	std::list<CpersonTarget>::iterator iiF, jjF;//, auxF;	
	Cpoint3dCov iiTrckPos, jjTrckPos;
	double dist;
      bool incrementToBeRemoved_ii;
	//bool iiFilterDeleted, jjFilterDeleted;      
	
	//Delete filters that have status=TO_BE_REMOVED
	for (iiF=targetList.begin();iiF!=targetList.end();iiF++)
	{
		if ( iiF->isStatus(TO_BE_REMOVED) )
		{
			targetList.erase(iiF);
			break;
		}
	}
	
      //update toBeRemovedCounters of each filter
      for (iiF=targetList.begin();iiF!=targetList.end();iiF++)
      {
            iiF->getPositionEstimate(iiTrckPos);
            incrementToBeRemoved_ii = false;
            for (jjF=targetList.begin();jjF!=targetList.end();jjF++)
            {
                  jjF->getPositionEstimate(jjTrckPos);
                  dist = iiTrckPos.d2point(jjTrckPos);
                  if ( dist < params.minDistanceBetweenPeople )
                  {
                        if ( iiF->getMaxStatus() < jjF->getMaxStatus() ) //ii has lower status than jj
                        {
                              incrementToBeRemoved_ii = true;
                              break;
                        }
                        
                        if ( (iiF->getMaxStatus() == jjF->getMaxStatus()) && (iiF->getId() > jjF->getId()) ) //same status & ii is newer-> increment counter of filter ii
                        {
                              incrementToBeRemoved_ii = true;
                              break;
                        }
                  }
            }
            if (incrementToBeRemoved_ii) iiF->incrementToBeRemovedCounter();
            else iiF->resetToBeRemovedCounter();
      }
	
      
      /*
	//removes filters that are closer between them. Keeps id of the oldest (lower id)
	iiF=targetList.begin();
	while(iiF!=targetList.end())
	{
		iiFilterDeleted = false;
		iiF->getPositionEstimate(iiTrckPos);
		jjF = iiF;
		jjF++;
		while(jjF!=targetList.end())
		{
			jjFilterDeleted = false;
			jjF->getPositionEstimate(jjTrckPos);
			dist = iiTrckPos.d2point(jjTrckPos);
			if ( dist < params.minDistanceBetweenPeople )
			{
				if( jjF->getId() > iiF->getId() ) //jj is newer-> remove filter jj
				{
					auxF = jjF;
					jjF++;
					targetList.erase(auxF);
					jjFilterDeleted = true;
				}
				else //ii is newer -> remove filter ii
				{
					auxF = iiF;
					iiF++;
					targetList.erase(auxF);	
					iiFilterDeleted = true;
					break;
				}
			}
			if (!jjFilterDeleted) jjF++;
		}
		if (!iiFilterDeleted) iiF++;
	}
	*/
}

void CpeopleTracker::updateFilterEstimates()
{
	std::list<CpersonTarget>::iterator iiF;
	
	for (iiF=targetList.begin();iiF!=targetList.end();iiF++)
	{
		iiF->updateEstimate();
	}		
}

void CpeopleTracker::addEstimatesToTracks()
{
	std::list<CpersonTarget>::iterator iiF;
	for (iiF=targetList.begin();iiF!=targetList.end();iiF++)
	{
		iiF->addEstimateToTrack();
	}		
}

void CpeopleTracker::propagateFilters(CodometryObservation & odoIncrement)
{
	std::list<CpersonTarget>::iterator iiF;
	
	for (iiF=targetList.begin();iiF!=targetList.end();iiF++)
	{
		iiF->predictPset(odoIncrement);
	}
}

void CpeopleTracker::propagateFilters()
{
	std::list<CpersonTarget>::iterator iiF;
	
	for (iiF=targetList.begin();iiF!=targetList.end();iiF++)
	{
		iiF->predictPset();
	}
}

void CpeopleTracker::correctFilters()
{
	std::list<CpersonTarget>::iterator iiT;
	std::list<Cpoint3dObservation>::iterator jjL;
	std::list<CbodyObservation>::iterator jjB;
      std::list<CfaceObservation>::iterator jjF;
	unsigned int numP,ii,jj;
	bool associated_legs=false, associated_body=false, associated_face =false;
	vector<double> ww_fusion, ww_legs, ww_body, ww_face;	
//       bool targetOccluded;
		
	for (iiT=targetList.begin(),ii=0;iiT!=targetList.end();iiT++,ii++)
	{
		numP = iiT->getNP();
		
		//LEGS
		ww_legs.reserve(numP);//init vector sizes
		for (jj=0; jj<numP; jj++) ww_legs.push_back(0);//reset values //ALERT: do a resize before and here reset to 1's (without push_back) to avoid case !associated_legs
		associated_legs = false;
		for (jjL=laserDetSet.begin(),jj=0;jjL!=laserDetSet.end();jjL++,jj++)
		{
			if ( iiT->aDecisions[LEGS].at(jj) ) //leg detection jj is associated with filter ii
			{
				associated_legs = true;
				iiT->computeWeights(*jjL, ww_legs);
			}
		}
		if (!associated_legs) 
		{
			for (ii=0; ii<numP; ii++) ww_legs.at(ii) = 1;//leg detection does not "shape" particle set
		}
		
		//BODY
		ww_body.reserve(numP);//init vector sizes
		for (jj=0; jj<numP; jj++) ww_body.push_back(0);//reset values //ALERT: do a resize before, and here reset to 1's (without push_back) to avoid case !associated_body
		associated_body = false;
		for (jjB=bodyDetSet.begin(),jj=0;jjB!=bodyDetSet.end();jjB++,jj++)
		{
			if ( iiT->aDecisions[BODY].at(jj) ) //body detection jj is associated with filter ii
			{
				associated_body = true;
				iiT->computeWeights(*jjB, ww_body);
			}
		}
		if (!associated_body) 
		{
			for (ii=0; ii<numP; ii++) ww_body.at(ii) = 1;//jj-th body detection does not correct ii-th particle set
		}

            //FACE
            ww_face.reserve(numP);//init vector sizes
            for (jj=0; jj<numP; jj++) ww_face.push_back(0);//reset values //ALERT: do a resize before, and here reset to 1's (without push_back) to avoid case !associated_body
            associated_face = false;
            for (jjF=faceDetSet.begin(),jj=0;jjF!=faceDetSet.end();jjF++,jj++)
            {
                  if ( iiT->aDecisions[FACE].at(jj) ) //face detection jj is associated with target ii
                  {
                        associated_face = true;
                        iiT->computeWeights(*jjF, ww_face);
                  }
            }
            if (!associated_face) 
            {
                  for (ii=0; ii<numP; ii++) ww_face.at(ii) = 1;//face detector does not correct ii-th target
            }

		//Check whether the iiT target has been associated to some detection or not
            if ( (associated_legs) || (associated_body) || (associated_face) )
            {
                  ww_fusion.clear();
                  ww_fusion.reserve(numP);
                  for (ii=0; ii<numP; ii++)
                  {
                        ww_fusion.push_back( ww_legs.at(ii)*ww_body.at(ii)*ww_face.at(ii) );
                  }
                  iiT->setWeights(ww_fusion);
            }
            
            //check if it is occluded
//             if ( iiT->pOcclusion > 0.5 ) targetOccluded = 1;
//             else targetOccluded = 0;
            
            //update target counters
//             iiT->updateCounters( (associated_legs || associated_body || associated_face) , (associated_body || associated_face) , targetOccluded );
            iiT->updateCounters( 
                  (associated_legs || associated_body || associated_face),
                  (associated_body || associated_face),
                  iiT->isStatus(IN_OCCLUSION) );
      }
}

void CpeopleTracker::resampleFilters()
{
	std::list<CpersonTarget>::iterator iiF;
	
	for (iiF=targetList.begin();iiF!=targetList.end();iiF++)
	{
		iiF->resamplePset();
	}
}

std::list<CpersonTarget> & CpeopleTracker::getTargetList()
{
	return targetList;
}

std::list<Cpoint3dObservation> & CpeopleTracker::getLaserDetSet()
{
	return laserDetSet;
}

std::list<CbodyObservation> & CpeopleTracker::getBodyDetSet()
{
	return bodyDetSet;
}

void CpeopleTracker::setCurrentImage(cv::Mat & inImg)
{
	this->img = inImg.clone();
}

void CpeopleTracker::getCurrentImage(cv::Mat & outImg)
{
	outImg = this->img.clone();
}

/*
void CpeopleTracker::computeTargetAppearance()
{
	cv::Rect_<int> bb;
	std::list<CbodyObservation>::iterator jjB;
      std::list<CpersonTarget>::iterator iiF;        
      unsigned int ii,jj;
      std::vector<HsHistogram> detApps;
      double mValue; //matching value
      std::ostringstream label;
      cv::Scalar labelColor;

	//for each body detection
      detApps.resize(bodyDetSet.size());
	for (jjB=bodyDetSet.begin(),jj=0;jjB!=bodyDetSet.end();jjB++,jj++)
	{
            //sets the bounding box of the detection jj
		bb.x = jjB->bbX;
		bb.y = jjB->bbY;
		bb.width = jjB->bbW;
		bb.height = jjB->bbH;
                
            //draws a red bounding box on the image according to detection jj
		//cv::rectangle(img, bb, cv::Scalar(0,0,255), 3);
                
            //computes an appearance of the bounding box
            detApps[jj].addAppearance(this->img,bb);
            
            //For each target "FRIEND_IN_SIGHT"
            for (iiF=targetList.begin(),ii=0;iiF!=targetList.end();iiF++,ii++)
            {
                  if ( iiF->isStatus(FRIEND_IN_SIGHT) )
                  {
                        //computes matching value
                        mValue = iiF->appearanceHistHS.match(&detApps[jj]);

                        //if target iiF associated to jj body detection
                        if ( iiF->aDecisions[BODY].at(jj) == true ) 
                        {
                              //draw a blue bounding box
                              cv::rectangle(img, bb, cv::Scalar(255,0,0), 3);
                              
                              //Add appearance to the target model
                              iiF->appearanceHistHS.addAppearance(img,bb);                                        
      
                              //set label color to blue
                              labelColor = cv::Scalar(255,0,0);
                        }
                        else
                        {
                              //set label color to red
                              labelColor = cv::Scalar(0,0,255);
                        }

                        //Display matching values
                        label.str("");
                        label << mValue; 
                        cv::putText(img,label.str(),cv::Point(bb.x+bb.width,bb.y+ii*20),cv::FONT_HERSHEY_SIMPLEX,0.4,labelColor,1);
                        //std::cout << "Matching appearance of target/detection " << iiF->getId() << "/" << jj << ": " << mValue << std::endl;
                  }
            }
      }	
}
*/

void CpeopleTracker::markBodies()
{
      cv::Rect_<int> bb;
      std::list<CbodyObservation>::iterator jjB;

      for (jjB=bodyDetSet.begin();jjB!=bodyDetSet.end();jjB++)
      {
            //sets the bounding box of the detection jj
            bb.x = jjB->bbX;
            bb.y = jjB->bbY;
            bb.width = jjB->bbW;
            bb.height = jjB->bbH;
                
            //draws a green bounding box on the image according to detection jj
            cv::rectangle(img, bb, cv::Scalar(0,255,255), 3);
      }
}

void CpeopleTracker::markFaces()
{
      cv::Rect_<int> bb;
      std::list<CfaceObservation>::iterator iiF;

      //for each face detection
      for (iiF=faceDetSet.begin();iiF!=faceDetSet.end();iiF++)
      {
            //sets the bounding box of the detection iiF
            bb.x = iiF->bbX;
            bb.y = iiF->bbY;
            bb.width = iiF->bbW;
            bb.height = iiF->bbH;
                
            //draws an orange bounding box on the image according to detection iiF
            cv::rectangle(img, bb, cv::Scalar(255,255,0), 3);
      }
}

void CpeopleTracker::markTld()
{
      cv::Rect_<int> bb;
      std::ostringstream label;
      
      label.precision(2);
      
      //draws a yellow bounding box on image according to current tld detection
      if ( tldDetection.bbW > 0 )
      {
            bb.x = tldDetection.bbX;
            bb.y = tldDetection.bbY;
            bb.width = tldDetection.bbW;
            bb.height = tldDetection.bbH;            
            cv::rectangle(img, bb, cv::Scalar(20,180,240), 3);
            label.str("");
            label << tldDetection.rgbEigen.getX();//confidence stored in the rgbEigen.X field
            cv::putText(img,label.str(),cv::Point(bb.x+bb.width,bb.y),cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(20,180,240),1);
            //std::cout << "markTld(): bb: " << bb.x << "," << bb.y << "," << bb.width << "," << bb.height << std::endl;
      }
}

void CpeopleTracker::printDetectionSets()
{
	std::list<Cpoint3dObservation>::iterator iiLd;
	std::list<CbodyObservation>::iterator iiBd;
      std::list<CfaceObservation>::iterator iiF;     
	
	std::cout << "Laser Leg Detections: "<< std::endl;
	for (iiLd=laserDetSet.begin();iiLd!=laserDetSet.end();iiLd++)
	{
		std::cout << iiLd->getId() << " ";
		iiLd->timeStamp.print();
		iiLd->point.printPoint();
	}

	std::cout << "Camera Body Detections: "<< std::endl;
	for (iiBd=bodyDetSet.begin();iiBd!=bodyDetSet.end();iiBd++)
	{
		std::cout << iiBd->getId() << " ";
		iiBd->timeStamp.print();
		iiBd->direction.printPoint();
		iiBd->rgbEigen.printPoint();
	}
	
      std::cout << "Camera Face Detections: "<< std::endl;
      for (iiF=faceDetSet.begin();iiF!=faceDetSet.end();iiF++)
      {
            std::cout << iiF->getId() << " ";
            iiF->timeStamp.print();
            iiF->faceLoc.printPoint();
      }
	
}

void CpeopleTracker::printPeopleSet()
{
	std::list<CpersonTarget>::iterator iiF;
	
	std::cout << std::endl;
	for (iiF=targetList.begin();iiF!=targetList.end();iiF++)
	{
		iiF->print();
		//iiF->printParticleSet();
	}
}


// void CpeopleTracker::initSingleTarget()
// {
//      std::list<Cpoint3dObservation>::iterator jjL;
//      double dd, angle;
//      CpersonTarget *newTarget;
// 
//      if (targetList.size() == 0)
//      {
//              for (jjL=laserDetSet.begin();jjL!=laserDetSet.end();jjL++)
//              {
//                      dd = jjL->point.norm();
//                      angle = fabs( atan2(jjL->point.getY(),jjL->point.getX()) ); 
//                      if ( (dd<1) && (angle<20*M_PI/180.0) )
//                      {
//                              newTarget = new CpersonTarget(nextTargetId);
//                              newTarget->setParameters(filterParams);
//                              nextTargetId++;
//                              newTarget->init(*jjL);
//                              newTarget->updateEstimate();
//                              targetList.push_back(*newTarget);
//                              break;
//                      }
//              }
//      }
// }


// void CpeopleTracker::correctFiltersSingleTarget()
// {
//      std::list<CpersonTarget>::iterator iiF;
//      std::list<Cpoint3dObservation>::iterator jjL, kkL;
//      std::list<CbodyObservation>::iterator jjB, kkB;
//      double dd, dMin, delta, deltaMin, estimatedAngle, detectedAngle;
//      vector<double> ww_fusion, ww_legs, ww_body;
//      unsigned int ii, numP;
//      bool associated_legs=false, associated_body=false;
//      
//      //legs
//      for (iiF=targetList.begin();iiF!=targetList.end();iiF++)
//      {
//              dMin = 1.5*params.minDistanceBetweenPeople;
//              associated_legs = false;
//              numP = iiF->getNP();
//              ww_legs.reserve(numP);//init vector sizes
//              for (ii=0; ii<numP; ii++) ww_legs.push_back(0);//reset values
//              targetState & filterEstimate = iiF->getEstimate();
//              for (jjL=laserDetSet.begin();jjL!=laserDetSet.end();jjL++)
//              {
//                      dd = filterEstimate.position.d2point(jjL->point);
//                      if( dd < dMin)
//                      {
//                              dMin = dd;
//                              kkL = jjL;
//                              associated_legs = true;
//                      }
//              }
//              if (associated_legs)
//                      iiF->computeWeights(*kkL, ww_legs);
//              else 
//                      for (ii=0; ii<numP; ii++) ww_legs.at(ii) = 1;
//      }
//      
//      //body
//      for (iiF=targetList.begin();iiF!=targetList.end();iiF++)
//      {
//              deltaMin = 10*M_PI/180.0;
//              associated_body = false;
//              numP = iiF->getNP();
//              ww_body.reserve(numP);//init vector sizes
//              for (ii=0; ii<numP; ii++) ww_body.push_back(0);//reset values
//              targetState & filterEstimate = iiF->getEstimate();
//              estimatedAngle = atan2(filterEstimate.position.getY(),filterEstimate.position.getX());
//              for (jjB=bodyDetSet.begin();jjB!=bodyDetSet.end();jjB++)
//              {
//                      detectedAngle = atan2(jjB->direction.getY(),jjB->direction.getX());
//                      delta = fabs(detectedAngle-estimatedAngle);
//                      if( delta < deltaMin)
//                      {
//                              deltaMin = delta;
//                              kkB = jjB;
//                              associated_body = true;
//                      }
//              }
//              if (associated_body)
//                      iiF->computeWeights(*kkB, ww_body);
//              else 
//                      for (ii=0; ii<numP; ii++) ww_body.at(ii) = 1;
//      }
// 
//      //fusion
//      if ( (associated_legs) || (associated_body) ) // || (associated_face) )
//      {
//              ww_fusion.clear();
//              ww_fusion.reserve(numP);
//              for (ii=0; ii<numP; ii++)
//              {
//                      ww_fusion.push_back( ww_legs.at(ii)*ww_body.at(ii) );
//              }
//              iiF = targetList.begin();
//              iiF->setWeights(ww_fusion);
//      }
// }

/*
void CpeopleTracker::removeCrossAssociatedParticles()
{
        std::list<CpersonTarget>::iterator iiF, jjF;
        std::list<CpersonParticle> * pList;
        std::list<CpersonParticle>::iterator kkP;
        Cpoint3dCov iiEstimate, jjEstimate;
        double dist, dM1, dM2;
        
        for (iiF=targetList.begin();iiF!=targetList.end();iiF++)
        {       
                pList = iiF->getParticleList();
                iiF->getPositionEstimate(iiEstimate);
                for (jjF=targetList.begin();jjF!=targetList.end();jjF++)
                {
                        if (iiF->getTargetId() != jjF->getTargetId()) //evaluate when different id
                        {
                                jjF->getPositionEstimate(jjEstimate);
                                dist = iiEstimate.d2point(jjEstimate);
                                if ( dist < params.minDistanceBetweenPeople ) //evaluate only if filters ii and jj are closer enough
                                {
                                        for (kkP = pList->begin(); kkP!=pList->end(); kkP++) //particle kk belongs to ii filter
                                        {
                                                //dM1 = iiEstimate.mahalanobisDistance2D(kkP->position);//mahalanobis between filter estimate ii and particle kk                                                
                                                //dM2 = jjEstimate.mahalanobisDistance2D(kkP->position);//mahalanobis between filter estimate jj and particle kk
                                                dM1 = iiEstimate.d2point(kkP->position);
                                                dM2 = jjEstimate.d2point(kkP->position);
                                                //std::cout << "removeCrossAssociatedParticles(): ";
                                                if(dM2 < dM1) //particle kk is closer to another filter (jj) than to that it belongs to (ii)
                                                {
                                                        //std::cout << "(" << dM1 << "," << dM2 << "); " << std::endl;
                                                        kkP->setW(0.0);
                                                }
                                                //std::cout << std::endl;
                                        }
                                }
                        }
                }
        }
}
*/
