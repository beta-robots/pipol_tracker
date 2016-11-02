#include "personParticleFilter.h"

CpersonParticleFilter::CpersonParticleFilter()
{
        //init iteration counters
        iteration_counter_ = 0;
        countConsecutiveUncorrected = 0;
        countVisuallyCorrected = 0;
        countToBeRemoved = 0;
        
        //initializes random generator;
        srand ( time(NULL) ); 
        
        //initializes parameters
        setDefaultParameters();
        
        //forcing GO mode
        motionMode = MODE_GO;
}

CpersonParticleFilter::~CpersonParticleFilter()
{
	p_set_.clear();//clears particle set
	detection_list_
}

void CpersonParticleFilter::setDefaultParameters()
{
        params.numParticles = DEFAULT_NP;
        params.initDeltaXY = INIT_DELTA_XY;
        params.initDeltaVxy = INIT_DELTA_VXY;
        params.sigmaResamplingXY = SIGMA_FIXED_RESAMPLING_XY;
        params.sigmaRatioResamplingVxy = SIGMA_RATIO_RESAMPLING_VXY;
        params.sigmaMinResamplingVxy = SIGMA_MIN_RESAMPLING_VXY;
//         params.personRadiusLegs = PERSON_RADIUS_LEGS;
//         params.personRadiusBody = PERSON_RADIUS_BODY;
//         params.matchingLegsAlpha = MATCHING_LEGS_ALPHA;
//         params.matchingLegsBeta = MATCHING_LEGS_BETA;
//         params.matchingBearingAlpha = MATCHING_BODY_ALPHA;
//         params.matchingBearingBeta = MATCHING_BODY_BETA;
//         params.matchingBody3dAlpha = MATCHING_BODY3D_ALPHA;
//         params.matchingBody3dBeta = MATCHING_BODY3D_BETA;        
//         dConstants.legsK1 = -params.matchingLegsAlpha/(params.personRadiusLegs*params.personRadiusLegs);
//         dConstants.body3dK1 = -params.matchingBody3dAlpha/(params.personRadiusBody*params.personRadiusBody);
}

void CpersonParticleFilter::setParameters(const pFilterParameters & pfp)
{
        params.numParticles = pfp.numParticles;
        params.initDeltaXY = pfp.initDeltaXY;
        params.initDeltaVxy = pfp.initDeltaVxy;
        params.sigmaResamplingXY = pfp.sigmaResamplingXY;
        params.sigmaRatioResamplingVxy = pfp.sigmaRatioResamplingVxy;
        params.sigmaMinResamplingVxy = pfp.sigmaMinResamplingVxy;
//         params.personRadiusLegs = pfp.personRadiusLegs;
//         params.personRadiusBody = pfp.personRadiusBody;
//         params.matchingLegsAlpha = pfp.matchingLegsAlpha; 
//         params.matchingLegsBeta = pfp.matchingLegsBeta; 
//         params.matchingBearingAlpha = pfp.matchingBearingAlpha;
//         params.matchingBearingBeta = pfp.matchingBearingBeta;
//         params.matchingBody3dAlpha = pfp.matchingBody3dAlpha;
//         params.matchingBody3dBeta = pfp.matchingBody3dBeta;                
//         dConstants.legsK1 = -params.matchingLegsAlpha/(params.personRadiusLegs*params.personRadiusLegs);
//         dConstants.body3dK1 = -params.matchingBody3dAlpha/(params.personRadiusBody*params.personRadiusBody);
}

unsigned int CpersonParticleFilter::getNP()
{
        return params.numParticles;
}

// double CpersonParticleFilter::getPersonRadius()
// {
//         return params.personRadiusLegs;
// }

unsigned int CpersonParticleFilter::getIterations()
{
        return iteration_counter_;
}

unsigned int CpersonParticleFilter::getConsecutiveUncorrected()
{
        return countConsecutiveUncorrected;
}

void CpersonParticleFilter::getPositionEstimate(Cpoint3dCov & est)
{
        //target.getPositionEstimate(est);
        est = estimate.position;
}

void CpersonParticleFilter::getEstimate(filterEstimate & est)
{
        est.ts.set(estimate.ts.get());
        est.position = estimate.position;
        est.velocity = estimate.velocity;
}

std::list<CpersonParticle> & CpersonParticleFilter::getParticleSet()
{
        return p_set_;
}

void CpersonParticleFilter::updateCounters(bool corrected, bool visual, bool occluded)
{
        iteration_counter_ ++;
        if(!occluded) //if occluded do not modify counters
        {
            if ( corrected ) countConsecutiveUncorrected = 0;
            else countConsecutiveUncorrected ++;
            if ( visual ) countVisuallyCorrected ++;
        }
}

void CpersonParticleFilter::incrementToBeRemovedCounter()
{
      countToBeRemoved ++;
}

void CpersonParticleFilter::resetToBeRemovedCounter()
{
      countToBeRemoved = 0;
}


// void CpersonParticleFilter::countAsCorrected()
// {
//         countConsecutiveUncorrected = 0;
//         iteration_counter_ ++;
// }
// 
// void CpersonParticleFilter::countAsUncorrected()
// {
//         countConsecutiveUncorrected ++;
//         iteration_counter_ ++;
// }

void CpersonParticleFilter::init(Cpoint3dObservation & pDet)
{
	unsigned int ii;
	double px,py,pvx,pvy;

	p_set_.clear();
	for(ii=0; ii<params.numParticles; ii++)
	{
		px = pDet.point.getX() + params.initDeltaXY*(double)rand()/((double)RAND_MAX)-params.initDeltaXY/2.0;
		py = pDet.point.getY() + params.initDeltaXY*(double)rand()/((double)RAND_MAX)-params.initDeltaXY/2.0;
		pvx = 0 + params.initDeltaVxy*(double)rand()/((double)RAND_MAX)-params.initDeltaVxy/2.0;
		pvy = 0 + params.initDeltaVxy*(double)rand()/((double)RAND_MAX)-params.initDeltaVxy/2.0;
		p_set_.push_back(CpersonParticle(px,py,pvx,pvy,1.0/(double)params.numParticles));
		//newPart->printParticle();
	}
	iteration_counter_ ++;//init() counts as a first iteration
//         tsInit.setToNow();
	tsLastPrior.setToNow();
}

void CpersonParticleFilter::predictPset(CodometryObservation & odo)
{
        std::list<CpersonParticle>::iterator iiP;
        Vector3f vp1, vp2;
        Matrix3f homT1; //current local frame wrt to past local frame
        Matrix3f homT2; //past local frame wrt to current local frame
        double dTheta, dTrans; 
        
        //updates particle points
        for (iiP=p_set_.begin();iiP!=p_set_.end();iiP++)
        {
                //add noise per each particle 
                dTheta = odo.getDeltaH() + random_normal(0,0.05); // 1 deg aprox
                dTrans = odo.getDeltaTrans() + random_normal(0,0.001); //1mm
                homT1 << cos(dTheta), -sin(dTheta), dTrans*cos(dTheta/2.0),
                        sin(dTheta),  cos(dTheta), dTrans*sin(dTheta/2.0),
                        0, 0, 1;
                homT2 = homT1.inverse();
                
                //particle position
                vp1 << iiP->position.getX(), iiP->position.getY(), 1;
                vp2 = homT2*vp1;
                iiP->position.setX(vp2(0)/vp2(2));
                iiP->position.setY(vp2(1)/vp2(2));
                
                //particle velocities
                vp1 << iiP->velocity.getX(), iiP->velocity.getY(), 0; //Last component 0 because velocity have to be rotated only
                //homT1(0,2) = 0.; //20Hz->50ms
                //homT1(1,2) = 0.; //20Hz->50ms
                //homT2 = homT1.inverse();
                vp2 = homT2*vp1;
                iiP->velocity.setX(vp2(0));
                iiP->velocity.setY(vp2(1));
        }
}

void CpersonParticleFilter::predictPset()
{
        double dT, rnd;
        CtimeStamp now;
        std::list<CpersonParticle>::iterator iiP;
        
        //compute time elapsed since the last call 
        now.setToNow();
        dT = now.get() - tsLastPrior.get();
        tsLastPrior.set(now.get());
        
        //call prediction model according current motion mode and transition probabilities between STOP & GO
        switch(motionMode)
        {
            case MODE_STOP:
                for (iiP=p_set_.begin();iiP!=p_set_.end();iiP++)
                {
                    rnd = ((double)rand()) / ((double)RAND_MAX);
                    if ( rnd < PROB_STOP2STOP ) iiP->predictStopped(dT);
                    else iiP->predictVlinear(dT);
                }
                break;
                
            case MODE_GO:
                for (iiP=p_set_.begin();iiP!=p_set_.end();iiP++)
                {
                    rnd = ((double)rand()) / ((double)RAND_MAX);
                    if ( rnd < PROB_GO2GO ) iiP->predictVlinear(dT);
                    else iiP->predictStopped(dT);
                }
                break;
    
            default: 
                break;
        }
}

void CpersonParticleFilter::correctPset()
{
	std::list<DetectionBase>::iterator iiD; 
	std::list<CpersonParticle>::iterator iiP;
	double weight; 
	Eigen::VectorXs state(4); //a state vector of 4 components

	for (iiP=p_set_.begin(); iiP!=p_set_.end(); iiP++)
	{
		weight = 1;
		state << iiP->position.getX(), iiP->position.getY(), iiP->velocity.getX(), iiP->velocity.getY();
		for (iiD=detection_list_.begin(); iiD!=detection_list_.end(); iiD++)
		{
			weight = weight*(*iiD)->likelihood(Eigen::Vector2s()); 
		}
		iiP->setW(weight);
	}

}

// void CpersonParticleFilter::computeWeights(Cpoint3dObservation & pDet, vector<double> & ww)
// {
//         std::list<CpersonParticle>::iterator iiP;
//         unsigned int ii;
//         
//         for (iiP=p_set_.begin(), ii=0;iiP!=p_set_.end();iiP++, ii++)
//         {
//                 //dd = pDet.point.mahalanobisDistance2D(iiP->position);
//                 //ww.at(ii) += assocP*erfc(dM/SQRT_2);
//                 ww.at(ii) = legMatchingFunction(iiP->position, pDet.point);
//         }
// }
// 
// void CpersonParticleFilter::computeWeights(CbodyObservation & pDet, vector<double> & ww)
// {
//         std::list<CpersonParticle>::iterator iiP;
//         unsigned int ii;
//         
//         for (iiP=p_set_.begin(), ii=0;iiP!=p_set_.end();iiP++, ii++)
//         {
//                 ww.at(ii) = bodyMatchingFunction(pDet.direction, iiP->position);
//         }       
// }
// 
// void CpersonParticleFilter::computeWeights(CfaceObservation & pDet, vector<double> & ww)
// {
//         std::list<CpersonParticle>::iterator iiP;
//         unsigned int ii;
//         
//         for (iiP=p_set_.begin(), ii=0;iiP!=p_set_.end();iiP++, ii++)
//         {
//                 ww.at(ii) = faceMatchingFunction(pDet.faceLoc, iiP->position);
//         }       
// }
// 
// void CpersonParticleFilter::computeWeightsBody3d(Cpoint3dObservation & pDet, vector<double> & ww)
// {
//         std::list<CpersonParticle>::iterator iiP;
//         unsigned int ii;
//         
//         for (iiP=p_set_.begin(), ii=0;iiP!=p_set_.end();iiP++, ii++)
//         {
//                 ww.at(ii) = body3dMatchingFunction(iiP->position, pDet.point);
//         }       
// }
// 
// void CpersonParticleFilter::setWeights(const vector<double> & ww)
// {
//         std::list<CpersonParticle>::iterator iiP;
//         unsigned int ii=0;
//         
//         for (iiP=p_set_.begin();iiP!=p_set_.end();iiP++)
//         {
//                 iiP->setW(ww.at(ii));
//                 ii++;
//         }
// }

void CpersonParticleFilter::normalizePset()
{
	double sumW=0;
	std::list<CpersonParticle>::iterator iiP;
        
	for (iiP=p_set_.begin();iiP!=p_set_.end();iiP++)
	{
		sumW+=iiP->getW();
	}

	//check case 
	if (sumW < ZERO_LIKELIHOOD) //case with extreme low likelihood, set all particles to 1/NP again
	{
		for (iiP=p_set_.begin();iiP!=p_set_.end();iiP++){iiP->setW(1.0/(double)params.numParticles);}
	}
	else//usual case
	{
		for (iiP=p_set_.begin();iiP!=p_set_.end();iiP++){iiP->setW((iiP->getW())/sumW);}
	}
}

void CpersonParticleFilter::resamplePset()
{
    double rnd, cw;
    unsigned int ii=0;
    std::list<CpersonParticle>::iterator iiP, jjP;
    double pX, pY, pVx, pVy; //the sample values to generate a new particle
    CpersonParticle *newP; //to generate new particles
    double v_mod;
    
    //normalize p_set_ (It could be not normalized due to particle deletion)
    this->normalizePset(); 

    //draws a random unmber in [0,1/NP]
    rnd=((double)rand()) / (((double)RAND_MAX)*(double)params.numParticles);
    
    //sets p_set_ pointers
    jjP = p_set_.end();
    jjP--;//points to the last particle of the current set. Used at end of the function to delete the old set
    iiP = p_set_.begin();
    cw = iiP->getW();//initialized to first weight
    
    //main resampling loop. Chooses old particles to be reampled according rnd and weights
    while(ii<params.numParticles)
    {
        while (rnd > cw) 
        {
            iiP++;
            cw += iiP->getW();
        }
        pX = iiP->position.getX() + random_normal(0,params.sigmaResamplingXY); //centered at particle ii
        pY = iiP->position.getY() + random_normal(0,params.sigmaResamplingXY); //centered at particle ii

//         pVx = iiP->velocity.getX() + random_normal(0,params.sigmaRatioResamplingVxy*iiP->velocity.getX()+params.sigmaMinResamplingVxy);
//         pVy = iiP->velocity.getY() + random_normal(0,params.sigmaRatioResamplingVxy*iiP->velocity.norm()+params.sigmaMinResamplingVxy);                
        
        pVx = iiP->velocity.getX() + random_normal(0,params.sigmaRatioResamplingVxy*iiP->velocity.norm()+params.sigmaMinResamplingVxy);
        pVy = iiP->velocity.getY() + random_normal(0,params.sigmaRatioResamplingVxy*iiP->velocity.norm()+params.sigmaMinResamplingVxy);                

        newP = new CpersonParticle(pX,pY,pVx,pVy,1.0/(double)params.numParticles);
        p_set_.push_back(*newP);
        rnd += 1.0/(double)params.numParticles;
        ii++;
        delete newP;
    }
        
    //erase former particles of the set
    jjP++;
    p_set_.erase(p_set_.begin(),jjP);
}

void CpersonParticleFilter::updateEstimate()
{
    std::list<CpersonParticle>::iterator iiP;
    double xx, yy; 
    double vx, vy;
    double xxCov, yyCov, xyCov;
    double vxCov, vyCov, vxyCov;

    //sets time stamp of the current estimate
    estimate.ts.setToNow();
    
    //normalizes particle wieghts.
    this->normalizePset(); 
    
    //sets estimated mean
    xx = 0; yy = 0; vx = 0; vy = 0;
    for (iiP=p_set_.begin();iiP!=p_set_.end();iiP++)
    {
            xx += iiP->position.getX()*iiP->getW();
            yy += iiP->position.getY()*iiP->getW();
            vx += iiP->velocity.getX()*iiP->getW();
            vy += iiP->velocity.getY()*iiP->getW();
    }
    
    //compute estimated position/velocity uncertainties as the sample covariance matrix
    xxCov = 0; yyCov = 0; xyCov = 0;
    vxCov = 0; vyCov = 0; vxyCov = 0;
    for (iiP=p_set_.begin();iiP!=p_set_.end();iiP++)
    {
            xxCov += (iiP->position.getX()-xx)*(iiP->position.getX()-xx)*iiP->getW();
            yyCov += (iiP->position.getY()-yy)*(iiP->position.getY()-yy)*iiP->getW();
            xyCov += (iiP->position.getX()-xx)*(iiP->position.getY()-yy)*iiP->getW();
            vxCov += (iiP->velocity.getX()-vx)*(iiP->velocity.getX()-vx)*iiP->getW();
            vyCov += (iiP->velocity.getY()-vy)*(iiP->velocity.getY()-vy)*iiP->getW();
            vxyCov += (iiP->velocity.getX()-vx)*(iiP->velocity.getY()-vy)*iiP->getW();              
    }
    
    //sets values to this->target object
    this->estimate.position.setXYZ(xx,yy,0);
    this->estimate.velocity.setXYZ(vx,vy,0);              
    this->estimate.position.setXYcov(xxCov,yyCov,xyCov);
    this->estimate.velocity.setXYcov(vxCov,vyCov,vxyCov);
}

void CpersonParticleFilter::setMotionMode()
{
    if ( this->estimate.velocity.norm() < MAX_SPEED_STOPPED) motionMode = MODE_STOP;
    else motionMode = MODE_GO;
}

unsigned int CpersonParticleFilter::getMotionMode()
{
    return motionMode;
}

// double CpersonParticleFilter::legMatchingFunction(Cpoint3d & p1)
// {
//     return legMatchingFunction(p1,this->estimate.position);
//     //return this->estimate.position.mahalanobisDistance(p1); 
// }
// 
// double CpersonParticleFilter::legMatchingFunction(Cpoint3d & p1, Cpoint3d & p2)
// {
//     double dd, score;
// 
//     dd = p1.d2point(p2);
//     if ( dd <= params.personRadiusLegs )
//     {
//         score = dConstants.legsK1*dd*dd+1;
//     }
//     else
//     {
//         score = params.matchingLegsAlpha*exp( (params.personRadiusLegs-dd)*params.matchingLegsBeta );
//     }
//     return  score;
// }
/*
double CpersonParticleFilter::bodyMatchingFunction(Cpoint3d & pD)
{
    return bodyMatchingFunction(pD,this->estimate.position);
}

double CpersonParticleFilter::bodyMatchingFunction(Cpoint3d & pD, Cpoint3d & pT)
{
    double scBearing,scRange;
    double deltaAlpha, alphaD, alphaT, alphaDist, k1;
            
    //bearing score
    alphaD = atan2(pD.getY(), pD.getX());
    alphaT = atan2(pT.getY(), pT.getX());
    alphaDist = fabs(alphaD - alphaT);
    deltaAlpha = fabs(atan2(params.personRadiusBody, pT.norm()));
    k1 = -params.matchingBearingAlpha/(deltaAlpha*deltaAlpha);
    if ( alphaDist <= deltaAlpha )
    {
            scBearing = k1*alphaDist*alphaDist+1;
    }
    else
    {
            scBearing = params.matchingBearingAlpha*exp( (deltaAlpha-alphaDist)*params.matchingBearingBeta );
    }       

    //range score
    if ( pT.norm() < 2 ) scRange = 0;
    else scRange = 0.5 - (1/M_PI)*atan(1000*(2.-pT.norm()));
    
    //return score
    return scBearing*scRange;
}

double CpersonParticleFilter::faceMatchingFunction(Cpoint3d & pD)
{
    return faceMatchingFunction(pD,this->estimate.position);
}
double CpersonParticleFilter::faceMatchingFunction(Cpoint3d & pD, Cpoint3d & pT)
{
    double sc;
    double dd;
            
    //compute distance on the ground plane
    dd = sqrt( pow( pD.getX()-pT.getX(), 2 ) + pow( pD.getY()-pT.getY(), 2 ) );
    sc = 0.5 + (1/M_PI)*atan(10000*(params.personRadiusBody-dd));

    //return score
    return sc;
}*/

// double CpersonParticleFilter::body3dMatchingFunction(Cpoint3d & p1)
// {
//     return body3dMatchingFunction(p1,this->estimate.position);
// }
// 
// double CpersonParticleFilter::body3dMatchingFunction(Cpoint3d & p1, Cpoint3d & p2)
// {
//     double dd, score;
// 
//     //at the moment, using the same model as for legs ( see CpersonParticleFilter::legMatchingFunction() above )
//     dd = p1.d2point(p2);
//     if ( dd <= params.personRadiusBody )
//     {
//         score = dConstants.body3dK1*dd*dd+1; 
//     }
//     else
//     {
//         score = params.matchingBody3dAlpha*exp( (params.personRadiusLegs-dd)*params.matchingBody3dBeta );
//     }
//     return  score;
// }

// double CpersonParticleFilter::d2point2(const Cpoint3d & _pt)
// {
//     return this->estimate.position.d2point2(_pt); 
// }

double CpersonParticleFilter::getAzimuth() const
{
    return atan2(estimate.position.getY(), estimate.position.getX()); 
}

void CpersonParticleFilter::print(unsigned int tId) const
{
    std::cout << "----------- Filter " << tId << " --------- " << std::endl;
    estimate.position.printPointCov();
    estimate.velocity.printPointCov();
}

void CpersonParticleFilter::printParticleSet() const 
{
    std::list<CpersonParticle>::iterator iiP;

    for (iiP=p_set_.begin();iiP!=p_set_.end();iiP++)
    {
        iiP->printParticle();
    }
}


void CpersonParticleFilter::addDetection(DetectionBase * _det_ptr)
{
	detection_list_.push_back(_det_ptr);
}

void CpersonParticleFilter::clearDetections()
{
	std::list<DetectionBase*>::iterator iiD;
	
	for (iiD=detection_list_.begin();iiD!=detection_list_.end();iiD++)
	{
		delete (*iiD); 
	}
	
	detection_list_.clear(); 
}

void CpersonParticleFilter::removeDetections(const TimeStamp & _ts)
{
	std::list<DetectionBase*>::iterator iiD;
	
	for (iiD=detection_list_.begin();iiD!=detection_list_.end();iiD++)
	{
		if ( (*iiD)->time_stamp_.get() < _ts.get() )
		{
			delete (*iiD);
			//detection_list_.clear(iiD);
		}
	}
}
