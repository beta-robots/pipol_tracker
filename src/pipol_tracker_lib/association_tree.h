
#ifndef association_tree_H
#define association_tree_H

//std
// #include <list>
// #include <vector>
//#include <pair>
//#include <memory>

//pipol tracker
#include "matrixx.h"
#include "association_solver.h"
#include "association_node.h"

/** \brief The whole decision tree
 * 
 * The whole decision tree
 * 
*/
class AssociationTree : public AssociationSolver
{
    protected:
        AssociationNode root_;
        std::list<AssociationNode*> terminus_node_list_;

    public:
        /** \brief Constructor
        * 
        * Constructor 
        * 
        */        
        AssociationTree();            
        
        /** \brief Destructor
        * 
        * Destructor
        * 
        */        
        virtual ~AssociationTree();

        /** \brief Reset
        * 
        * Deletes all nodes and clears scores and association list
        * 
        */        
        void reset();            
        
        /** \brief Resizes the problem
        * 
        * Resizes the problem: 
        * \param _n_det num of detections
        * \param _n_tar num of targets
        * Resizes the scores_ matrix which will allocate _n_det rows and _n_tar+1 columns to take into account void target
        * 
        */        
        void resize(const unsigned int _n_det, const unsigned int _n_tar);                
        
        /** \brief Build tree from scores
        * 
        * Build tree from scores
        * 
        */        
        void growTree();

        /** \brief Computes tree probabilities
        * 
        * Computes tree probabilities
        * 
        */        
        void computeTree();
        
        /** \brief Normalizes node probabilities
        * 
        * Normalizes node probabilities
        * 
        */        
        void normalizeTree();        
        
        /** \brief choose best terminus node
         * 
         * Choose best terminus node based on the best tree probability
         * \param _best_node a reference to an iterator to a list of pointers, where returned result is placed. 
         * At output, _best_node points the bets node in the terminus_node_list_
         * 
         **/
        void chooseBestTerminus(std::list<AssociationNode*>::iterator & _best_node);
        
        /** \brief Gets tree decision
         * 
         * Decides best hypothesis according tree computation made by computeTree()
         * Return values are: 
         * \param _pairs Returned pairs: vector of pairs (d_i, t_j)
         * \param _unassoc Returned unassociated detections: vector of (d_i)
         * \param _associated_mask Resized to nd_. Marks true at i if detection d_i has been associated, otherwise marks false
         * 
         **/
        //void solve(std::vector<std::pair<unsigned int, unsigned int> > & _pairs, std::vector<unsigned int> & _unassoc);
        void solve(std::vector<std::pair<unsigned int, unsigned int> > & _pairs, std::vector<bool> & _associated_mask);
        
        /** \brief Prints the tree
        * 
        * Prints the tree
        * 
        * TODO: this function should be const. See comments on printTree() at association_node.h
        */                        
        void printTree();       
        
        /** \brief Prints terminus_node_list_
        * 
        * Prints terminus_node_list_
        * 
        */                        
        void printTerminusNodes();       
};
#endif            
