
#ifndef association_nnls_H
#define association_nnls_H

//pipol tracker
#include "association_solver.h"

/** \brief The whole decision tree
 * 
 * The whole decision tree
 * 
*/
class AssociationNNLS : public AssociationSolver
{
    protected:
        double min_dist_; //minimum distance to allow association 
        vector<bool> i_mask_; // mask already allocated detections (rows)
        vector<bool> j_mask_; // mask already allocated targets (columns)
        
    public:
        /** \brief Constructor
        * 
        * Constructor 
        * 
        */        
        AssociationNNLS(_const double _min_dist);            
        
        /** \brief Destructor
        * 
        * Destructor
        * 
        */        
        virtual ~AssociationNNLS();
        
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
         * 
         **/
        void solve(std::vector<std::pair<unsigned int, unsigned int> > & _pairs, std::vector<unsigned int> & _unassoc);
        
};
#endif            
