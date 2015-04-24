
#ifndef association_solver_H
#define association_solver_H

//std
#include <iostream>
#include <vector>

//matrix class
#include "matrix.h"

/** \brief A pure virtual solver for the association problem
 * 
 * A pure virtual solver for the association problem
 * 
*/
class AssociationSolver
{
    protected:
        unsigned int nd_; //num detections
        unsigned int nt_; //num targets, without counting the void target
//         std::vector< std::vector<double> > scores_;//scores table. Size is (nd_) x (nt_+1), to account for the void target
        Matrixx<double> scores_;//scores table. Size is (nd_) x (nt_+1), to account for the void target

    public:
        /** \brief Constructor
        * 
        * Constructor 
        * 
        */        
        AssociationSolver();            
        
        /** \brief Destructor
        * 
        * Destructor
        * 
        */        
        virtual ~AssociationSolver();
        
        /** \brief Returns num of detections nd_
         * 
         * Returns num of detections nd_
         * 
         **/
        unsigned int numDetections();
        
        /** \brief Returns num of actual targets nt_
         * 
         * Returns num of actual targets nt_
         * 
         **/
        unsigned int numTargets();
        
        /** \brief Sets values to scores_ table
         * 
         * Sets value to score table, at cell ij, corresponding to detection_i and target_j
         * 
         **/
        void setScore(const unsigned int _det_i, const unsigned int _tar_j, const double _m_ij);
        
        /** \brief Prints the score table
        * 
        * Prints the score table
        * 
        */                
        void printScoreTable() const;        
        
        /** \brief Resets the problem
        * 
        * Deletes and clears the problem
        * 
        */        
        virtual void reset() = 0;    

        /** \brief Resizes the problem
        * 
        * Resizes the problem: 
        * \param _n_det num of detections
        * \param _n_tar num of targets
        * 
        */        
        virtual void resize(const unsigned int _n_det, const unsigned int _n_tar) = 0;        
        
        /** \brief Solves and sets decision pairs
         * 
         * Solves and sets decision pairs
         * Return values are: 
         * \param _pairs Returned pairs: vector of pairs (d_i, t_j)
         * \param _unassoc Returned unassociated detections: vector of (d_i)
         * \param _associated_mask Resized to nd_. Marks true at i if detection d_i has been associated, otherwise marks false
         * 
         **/
        //virtual void solve(std::vector<std::pair<unsigned int, unsigned int> > & _pairs, std::vector<unsigned int> & _unassoc) = 0;
        virtual void solve(std::vector<std::pair<unsigned int, unsigned int> > & _pairs, std::vector<bool> & _associated_mask) = 0;
        
};
#endif            
