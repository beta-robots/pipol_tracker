

#ifndef association_node_H
#define association_node_H

//std
#include <iostream>
#include <vector>
#include <algorithm> //find()

//constants
const double PROB_ZERO_ = 1e-6;

//enums
// /** \brief node types
//  * 
//  * Node types: 
//  *    - ROOT indicates top of the tree
//  *    - BRANCH indicates mid-level
//  *    - LEAF indicates last-level
//  *
//  **/
// typedef enum 
// {
//     ROOT, //top node
//     BRANCH, //mid-level nodes
//     LEAF //end nodes
// } NodeType;

/** \brief A node in the association decision tree 
 * 
 * A node in the association decision tree. A node associates the pair between a detection index and a target index, but not the detection Id and target Id. Therefore, Id to index mapping has to be implemented outside of this class.
 * 
*/
class AssociationNode
{
    protected:
        //NodeType node_type_;
        unsigned int det_idx_; ///< detection node index
        unsigned int tar_idx_; ///< target node index  
        double node_prob_; ///< Node Probability. Probability that detection associates to target.
        double tree_prob_; ///< Tree Probability. Joint Probability from the root node up to this (product of node probabilities)
        std::vector<AssociationNode> node_list_; ///< List of nodes below of this in the association tree

    public:
        /** \brief Constructor
        * 
        * Constructor with arguments _det_id and _tar_id which indicates association of detection and target, 
        * with the probability _prob;
        * 
        */        
        AssociationNode(const unsigned int _det_idx, const unsigned int _tar_idx, const double _prob);            
        
        /** \brief Destructor
        * 
        * Destructor
        * 
        */        
        virtual ~AssociationNode();
        
        /** \brief Returns node_type_
         * 
         * Returns node_type_
         * 
         **/
        //NodeType type() const;
        
        /** \brief Returns det_idx_
         * 
         * Returns det_idx_
         * 
         **/
        unsigned int getDetectionIndex() const;
        
        /** \brief Returns tar_idx_
         * 
         * Returns tar_idx_
         * 
         **/
        unsigned int getTargetIndex() const;
        
        /** \brief Returns node_prob_
         * 
         * Returns node_prob_
         * 
         **/
        double getNodeProb() const;

        /** \brief Returns tree_prob_
         * 
         * Returns tree_prob_
         * 
         **/
        double getTreeProb() const;
        
        /** \brief Computes node probability
         * 
         * Computes probability that detection_i associates to target_j, given the scores table _stab.
         * \param _di detection index (not id)
         * \param _tj target index (not id)
         * \param _stab score table
         * Returns the probability. 
         * Nodes require computing other ij probs to decide if they continue growing or not at function growTree().
         * 
         **/
        double computeNodeProb(const unsigned int _di, const unsigned int _tj, const std::vector< std::vector<double> > & _stab);

        /** \brief Computes tree probabilities recursively
         * 
         * Computes tree probabilities recursively, while setting tree_prob_ data member
         * \param _up_prob probability of upper node
         * 
         **/
        double computeTreeProb(const double & _up_prob);
        
        /** \brief Grows tree recursively
         * 
         * Grows tree recursively according the association probability table provided
         * \param _det_i: detection index
         * \param _p_tab: table of association probabilities between detections and targets
         * \param _ex_vec: vector of target index for which the tree should not continue growing 
         * 
         **/        
        void growTree(const unsigned int _det_i, const std::vector< std::vector<double> > & _stab, std::vector<unsigned int> & _ex_vec);
        
        /** \brief Prints node info recursively
         * 
         * Prints node info
         * \param _ntabs Number of tabulators befor printing. Useful for recursively print a whole tree
         * 
         **/
        void print(const unsigned int _ntabs = 0) const;
};
#endif
