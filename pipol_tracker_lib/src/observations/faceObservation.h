#ifndef faceObservation_H
#define faceObservation_H

#include "basicObservation.h"
#include "geometry/point3d.h"

/**
 *
 * \brief CfaceObservation implements 2D posCov observation
 * 
 * CfaceObservation implements a metric/appearance model for human face detections
 * 
*/
class CfaceObservation : public CbasicObservation
{
      public:
            /**
            * \brief 3d point of the face, wrt robot base. (X->fwd, Y->left, Z->up)
            *
            * 3d point of the face, wrt robot base. (X->fwd, Y->left, Z->up)
            *
            */          
            Cpoint3d faceLoc;
            
            double bbX,bbY; //up-left limits of the bounding box
            double bbW, bbH; //width and height of the bounding box
            
      public:
            /**
            * \brief Constructor
            *
            * Constructor
            *
            */                      
            CfaceObservation();

            /**
            * \brief Destructor
            *
            * Destructor
            *
            */                                  
            ~CfaceObservation();
};    
#endif
