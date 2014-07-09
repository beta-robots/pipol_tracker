#include "personParticleFilter.h"

CpersonParticleFilter::CpersonParticleFilter()
{
        //init iteration counters
        countIterations = 0;
        countConsecutiveUncorrected = 0;
        countVisuallyCorrected = 0;
        countToBeRemoved = 0;
        
        //initializes random generator;
        srand ( time(NULL) ); 
        
        //initializes parameters
        setDefaultParameters();
}

CpersonParticleFilter::~CpersonParticleFilter()
{
        //clears particle set
        pSet.clear();
}

void CpersonParticleFilter::setDefaultParameters()
{
        params.numParticles = DEFAULT_NP;
        params.initDeltaXY = INIT_DELTA_XY;
        params.initDeltaVxy = INIT_DELTA_VXY;
        params.sigmaResamplingXY = SIGMA_FIXED_RESAMPLING_XY;
        params.sigmaRatioResamplingVxy = SIGMA_RATIO_RESAMPLING_VXY;
        params.sigmaMinResamplingVxy = SIGMA_MIN_RESAMPLING_VXY;
        params.personRadiusLegs = PERSON_RADIUS_LEGS;
        params.personRadiusBody = PERSON_RADIUS_BODY;
        params.matchingLegsAlpha = MATCHING_LEGS_ALPHA;
        params.matchingLegsBeta = MATCHING_LEGS_BETA;
        params.matchingBearingAlpha = MATCHING_BODY_ALPHA;
        params.matchingBearingBeta = MATCHING_BODY_BETA;
        params.matchingBody3dAlpha = MATCHING_BODY3D_ALPHA;
        params.matchingBody3dBeta = MATCHING_BODY3D_BETA;        
        dConstants.legsK1 = -params.matchingLegsAlpha/(params.personRadiusLegs*params.personRadiusLegs);
        dConstants.body3dK1 = -params.matchingBody3dAlpha/(params.personRadiusBody*params.personRadiusBody);
}

void CpersonParticleFilter::setParameters(const pFilterParameters & pfp)
{
        params.numParticles = pfp.numParticles;
        params.initDeltaXY = pfp.initDeltaXY;
        params.initDeltaVxy = pfp.initDeltaVxy;
        params.sigmaResamplingXY = pfp.sigmaResamplingXY;
        params.sigmaRatioResamplingVxy = pfp.sigmaRatioResamplingVxy;
        params.sigmaMinResamplingVxy = pfp.sigmaMinResamplingVxy;
        params.personRadiusLegs = pfp.personRadiusLegs;
        params.personRadiusBody = pfp.personRadiusBody;
        params.matchingLegsAlpha = pfp.matchingLegsAlpha; 
        params.matchingLegsBeta = pfp.matchingLegsBeta; 
        params.matchingBearingAlpha = pfp.matchingBearingAlpha;
        params.matchingBearingBeta = pfp.matchingBearingBeta;
        params.matchingBody3dAlpha = pfp.matchingBody3dAlpha;
        params.matchingBody3dBeta = pfp.matchingBody3dBeta;                
        dConstants.legsK1 = -params.matchingLegsAlpha/(params.personRadiusLegs*params.personRadiusLegs);
        dConstants.body3dK1 = -params.matchingBody3dAlpha/(params.personRadiusBody*params.personRadiusBody);
}

unsigned int CpersonParticleFilter::getNP()
{
        return params.numParticles;
}

double CpersonParticleFilter::getPersonRadius()
{
        return params.personRadiusLegs;
}

unsigned int CpersonParticleFilter::getIterations()
{
        return countIterations;
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
        return pSet;
}

void CpersonParticleFilter::updateCounters(bool corrected, bool visual, bool occluded)
{
        countIterations ++;
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
//         countIterations ++;
// }
// 
// void CpersonParticleFilter::countAsUncorrected()
// {
//         countConsecutiveUncorrected ++;
//         countIterations ++;
// }

void CpersonParticleFilter::init(Cpoint3dObservation & pDet)
{
        unsigned int ii;
        double px,py,pvx,pvy;

        pSet.clear();
        for(ii=0; ii<params.numParticles; ii++)
        {
                px = pDet.point.getX() + params.initDeltaXY*(double)rand()/((double)RAND_MAX)-params.initDeltaXY/2.0;
                py = pDet.point.getY() + params.initDeltaXY*(double)rand()/((double)RAND_MAX)-params.initDeltaXY/2.0;
                pvx = 0 + params.initDeltaVxy*(double)rand()/((double)RAND_MAX)-params.initDeltaVxy/2.0;
                pvy = 0 + params.initDeltaVxy*(double)rand()/((double)RAND_MAX)-params.initDeltaVxy/2.0;
                pSet.push_back(CpersonParticle(px,py,pvx,pvy,1.0/(double)params.numParticles));
                //newPart->printParticle();
        }
        countIterations ++;//init() counts as a first iteration
//         tsInit.setToNow();
        tsLastPrior.setToNow();
}

void CpersonParticleFilter::predictPset(CodometryObservation & odo)
{
        std::list<CpersonParticle>::iterator iiP;
        Vector3f vp1, vp2;
        Matrix3f homT1; //current local frame wrt to past local frame
        Matrix3f homT2; //past local frame wrt to current local frame
        
        homT1 << cos(odo.getDeltaH()), -sin(odo.getDeltaH()), odo.getDeltaTrans()*cos(odo.getDeltaH()/2.0),
                   sin(odo.getDeltaH()),  cos(odo.getDeltaH()), odo.getDeltaTrans()*sin(odo.getDeltaH()/2.0),
                   0, 0, 1;
        homT2 = homT1.inverse();

        //updates particle points
        for (iiP=pSet.begin();iiP!=pSet.end();iiP++)
        {
                //particle position
                vp1 << iiP->position.getX(), iiP->position.getY(), 1;
                vp2 = homT2*vp1;
                iiP->position.setX(vp2(0));
                iiP->position.setY(vp2(1));
                
                //particle velocities
                vp1 << iiP->velocity.getX(), iiP->velocity.getY(), 0; //Last component to 0 because velocity is a direction vector (without offsets)
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
        
        now.setToNow();
        dT = now.get() - tsLastPrior.get();
        tsLastPrior.set(now.get());

        //call prediction model according current motion mode and transition probabilities between STOP & GO
        switch(motionMode)
        {
            case MODE_STOP:
                for (iiP=pSet.begin();iiP!=pSet.end();iiP++)
                {
                    rnd = ((double)rand()) / ((double)RAND_MAX);
                    if ( rnd < PROB_STOP2STOP ) iiP->predictStopped(dT);
                    else iiP->predictVlinear(dT);
                }
                break;
                
            case MODE_GO:
                for (iiP=pSet.begin();iiP!=pSet.end();iiP++)
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

void CpersonParticleFilter::computeWeights(Cpoint3dObservation & pDet, vector<double> & ww)
{
        std::list<CpersonParticle>::iterator iiP;
        unsigned int ii;
        
        for (iiP=pSet.begin(), ii=0;iiP!=pSet.end();iiP++, ii++)
        {
                //dd = pDet.point.mahalanobisDistance2D(iiP->position);
                //ww.at(ii) += assocP*erfc(dM/SQRT_2);
                ww.at(ii) = legMatchingFunction(iiP->position, pDet.point);
        }
}

void CpersonParticleFilter::computeWeights(CbodyObservation & pDet, vector<double> & ww)
{
        std::list<CpersonParticle>::iterator iiP;
        unsigned int ii;
        
        for (iiP=pSet.begin(), ii=0;iiP!=pSet.end();iiP++, ii++)
        {
                ww.at(ii) = bodyMatchingFunction(pDet.direction, iiP->position);
        }       
}

void CpersonParticleFilter::computeWeights(CfaceObservation & pDet, vector<double> & ww)
{
        std::list<CpersonParticle>::iterator iiP;
        unsigned int ii;
        
        for (iiP=pSet.begin(), ii=0;iiP!=pSet.end();iiP++, ii++)
        {
                ww.at(ii) = faceMatchingFunction(pDet.faceLoc, iiP->position);
        }       
}

void CpersonParticleFilter::computeWeightsBody3d(Cpoint3dObservation & pDet, vector<double> & ww)
{
        std::list<CpersonParticle>::iterator iiP;
        unsigned int ii;
        
        for (iiP=pSet.begin(), ii=0;iiP!=pSet.end();iiP++, ii++)
        {
                ww.at(ii) = body3dMatchingFunction(iiP->position, pDet.point);
        }       
}

void CpersonParticleFilter::setWeights(const vector<double> & ww)
{
        std::list<CpersonParticle>::iterator iiP;
        unsigned int ii=0;
        
        for (iiP=pSet.begin();iiP!=pSet.end();iiP++)
        {
                iiP->setW(ww.at(ii));
                ii++;
        }
}

void CpersonParticleFilter::normalizePset()
{
        double sumW=0;
        std::list<CpersonParticle>::iterator iiP;
        
        for (iiP=pSet.begin();iiP!=pSet.end();iiP++){sumW+=iiP->getW();}
        if (sumW < ZERO_LIKELIHOOD) //case with extreme low likelihood, set all particles to 1/NP again
        {
                for (iiP=pSet.begin();iiP!=pSet.end();iiP++){iiP->setW(1.0/(double)params.numParticles);}
        }
        else//usual case
        {
                for (iiP=pSet.begin();iiP!=pSet.end();iiP++){iiP->setW((iiP->getW())/sumW);}
        }
}

void CpersonParticleFilter::resamplePset()
{
    double rnd, cw;
    unsigned int ii=0;
    std::list<CpersonParticle>::iterator iiP, jjP;
    double pX, pY, pVx, pVy; //the sample values to generate a new particle
    CpersonParticle *newP; //to generate new particles
    
    //normalize pSet (It could be not normalized due to particle deletion)
    this->normalizePset(); 

    //draws a random unmber in [0,1/NP]
    rnd=((double)rand()) / (((double)RAND_MAX)*(double)params.numParticles);
    
    //sets pSet pointers
    jjP = pSet.end();
    jjP--;//points to the last particle of the current set. Used at end of the function to delete the old set
    iiP = pSet.begin();
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
        pVx = iiP->velocity.getX() + random_normal(0,params.sigmaRatioResamplingVxy*iiP->velocity.getX()+params.sigmaMinResamplingVxy);
        pVy = iiP->velocity.getY() + random_normal(0,params.sigmaRatioResamplingVxy*iiP->velocity.getY()+params.sigmaMinResamplingVxy);                
        newP = new CpersonParticle(pX,pY,pVx,pVy,1.0/(double)params.numParticles);
        pSet.push_back(*newP);
        rnd += 1.0/(double)params.numParticles;
        ii++;
        delete newP;
    }
        
    //erase former particles of the set
    jjP++;
    pSet.erase(pSet.begin(),jjP);
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
    for (iiP=pSet.begin();iiP!=pSet.end();iiP++)
    {
            xx += iiP->position.getX()*iiP->getW();
            yy += iiP->position.getY()*iiP->getW();
            vx += iiP->velocity.getX()*iiP->getW();
            vy += iiP->velocity.getY()*iiP->getW();
    }
    
    //compute estimated position/velocity uncertainties as the sample covariance matrix
    xxCov = 0; yyCov = 0; xyCov = 0;
    vxCov = 0; vyCov = 0; vxyCov = 0;
    for (iiP=pSet.begin();iiP!=pSet.end();iiP++)
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

double CpersonParticleFilter::legMatchingFunction(Cpoint3d & p1)
{
    return legMatchingFunction(p1,this->estimate.position);
    //return this->estimate.position.mahalanobisDistance(p1); 
}

double CpersonParticleFilter::legMatchingFunction(Cpoint3d & p1, Cpoint3d & p2)
{
    double dd, score;

    dd = p1.d2point(p2);
    if ( dd <= params.personRadiusLegs )
    {
        score = dConstants.legsK1*dd*dd+1;
    }
    else
    {
        score = params.matchingLegsAlpha*exp( (params.personRadiusLegs-dd)*params.matchingLegsBeta );
    }
    return  score;
}

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
}

double CpersonParticleFilter::body3dMatchingFunction(Cpoint3d & p1)
{
    return body3dMatchingFunction(p1,this->estimate.position);
}

double CpersonParticleFilter::body3dMatchingFunction(Cpoint3d & p1, Cpoint3d & p2)
{
    double dd, score;

    //at the moment, using the same model as for legs ( see CpersonParticleFilter::legMatchingFunction() above )
    dd = p1.d2point(p2);
    if ( dd <= params.personRadiusBody )
    {
        score = dConstants.body3dK1*dd*dd+1; 
    }
    else
    {
        score = params.matchingBody3dAlpha*exp( (params.personRadiusBody-dd)*params.matchingBody3dBeta );
    }
    return  score;
}

void CpersonParticleFilter::print(unsigned int tId)
{
    std::cout << "----------- Filter " << tId << " --------- " << std::endl;
    estimate.position.printPointCov();
    estimate.velocity.printPointCov();
}

void CpersonParticleFilter::printParticleSet()
{
    std::list<CpersonParticle>::iterator iiP;

    for (iiP=pSet.begin();iiP!=pSet.end();iiP++)
    {
        iiP->printParticle();
    }
}
