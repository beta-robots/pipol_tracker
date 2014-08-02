
#ifndef association_tree_H
#define association_tree_H

//std
#include <list>
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
        std::vector< std::vector<double> > scores_;//scores table. Size is (num_detections_) x (num_targets_+1)
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
        
        /** \brief Resize
        * 
        * Resize score table given num detections and num targets
        * 
        */        
        void resizeScoreTable(const unsigned int _n_det, const unsigned int _n_tar);

        /** \brief Returns num of detections
         * 
         * Returns scores.size(), which is the num of detections accounted.
         * 
         **/
        unsigned int numDetections();
        
        /** \brief Returns num of targets
         * 
         * Returns scores.at(0).size(), which is the num of targets, counting also the "void" target.
         * If no detections, it returns 0. 
         * 
         **/
        unsigned int numTargets();

        
        /** \brief Sets values to scores_ table
         * 
         * Sets value to score table, at cell ij, corresponding to detection_i and target_j
         * 
         **/
        void setScore(const unsigned int _det_i, const unsigned int _tar_j, const double _s_ij);
        
        /** \brief Build tree from scores
        * 
        * Build tree from scores
        * 
        */        
        void buildTree();

        /** \brief Computes tree probabilities
        * 
        * Computes tree probabilities
        * 
        */        
        void computeTree();
        
        /** \brief Decides best hypothesis
         * 
         * Decides best hypothesis according tree computation made by computeTree()
         * Pairs are returned in the param _pairs:
         * \param _pairs Returned pairs
         * 
         **/
        void bestHypothesis(std::vector<std::pair<unsigned int, unsigned int> > & _pairs);
        
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
        */                        
        void printTree() const;       
        
        /** \brief Prints the tree
        * 
        * Prints the tree
        * 
        */                        
        void printTerminusNodes();       
};
#endif            
