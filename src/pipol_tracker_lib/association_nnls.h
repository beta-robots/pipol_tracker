
#ifndef association_nnls_H
#define association_nnls_H

//std
#include <iostream>
#include <vector>

//pipol tracker
#include "association_solver.h"

//consts 
const double MAX_DIST_DEFAULT = 0.5; //units (meters in pt case)

/** \brief Nearest neighbour linear search
 * 
 * Nearest neighbour linear search to solve data association problems, given a table of distances 
 * 
*/
class AssociationNNLS : public AssociationSolver
{
    protected:
        double max_dist_; //maximum distance to allow association 
        std::vector<bool> i_mask_; // mask already allocated detections (rows)
        std::vector<bool> j_mask_; // mask already allocated targets (columns)
        
    public:
        /** \brief Constructor
        * 
        * Constructor 
        * 
        */        
        AssociationNNLS();            
        
        /** \brief Destructor
        * 
        * Destructor
        * 
        */        
        virtual ~AssociationNNLS();
        
        /** \brief Sets max_dist_
         * 
         * Sets max_dist_
         * 
         **/
        void setMaxDist(const double _max_dist);
        
        /** \brief Resets problem
        * 
        * Resets problem
        * 
        */        
        void reset();                    
            
        /** \brief Resizes the problem
        * 
        * Resizes the problem
        * 
        */        
        void resize(const unsigned int _n_det, const unsigned int _n_tar);
               
        /** \brief Solves the problem
         * 
         * Solves the asscoiation problem followinf nearest neighbor linear search
         * Return values are: 
         * \param _pairs Returned pairs: vector of pairs (d_i, t_j)
         * \param _unassoc Returned unassociated detections: vector of (d_i)
         * \param _associated_mask Resized to nd_. Marks true at i if detection d_i has been associated, otherwise marks false
         * 
         * Assumes i/j_mask_ vector class members and scores_ matrix are correctly sized, by a previous call to resize()
         * 
         **/
        //void solve(std::vector<std::pair<unsigned int, unsigned int> > & _pairs, std::vector<unsigned int> & _unassoc);
        void solve(std::vector<std::pair<unsigned int, unsigned int> > & _pairs, std::vector<bool> & _associated_mask);
        
};
#endif            
