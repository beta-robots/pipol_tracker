
#ifndef association_tree_H
#define association_tree_H

//std
// #include <list>
// #include <vector>
//#include <pair>
//#include <memory>

//pipol tracker
#include "association_node.h"

/** \brief The whole decision tree
 * 
 * The whole decision tree
 * 
*/
class AssociationTree
{
    protected:
        unsigned int nd_; //num detections
        unsigned int nt_; //num actual targets, without counting the void target
        std::vector< std::vector<double> > scores_;//scores table. Size is (nd_) x (nt_+1), to account for the void target
        AssociationNode root_;
        std::list<AssociationNode*> terminus_node_list_;
//         std::list<std::weak_ptr<AssociationNode> > terminus_node_list_; //TODO Use c++11 compiler !! After RoboCup!!

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
        
        /** \brief Resizes tree
        * 
        * Sets nd_ and nt_ and resizes score table
        * 
        */        
        void resize(const unsigned int _n_det, const unsigned int _n_tar);

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
        
        /** \brief Gets tree decision
         * 
         * Decides best hypothesis according tree computation made by computeTree()
         * Return values are: 
         * \param _pairs Returned pairs: vector of pairs (d_i, t_j)
         * \param _unassoc Returned unassociated detections: vector of (d_i)
         * 
         **/
        void treeDecision(std::vector<std::pair<unsigned int, unsigned int> > & _pairs, std::vector<unsigned int> & _unassoc);
        
        /** \brief Prints the score table
        * 
        * Prints the score table
        * 
        */                
        void printScoreTable() const;

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
