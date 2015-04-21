

#ifndef association_node_H
#define association_node_H

//std
#include <iostream>
#include <vector>
#include <list>
#include <algorithm> //find()

//pipol tracker
#include "matrix.h"

//constants
const double PROB_ZERO_ = 1e-3;

/** \brief A node in the association decision tree 
 * 
 * A node in the association decision tree. A node associates a pair between a detection index and a target index, which is
 * usually diferent from detection Id and target Id. Therefore, Id to index mapping has to be implemented outside of this class.
 * 
*/
class AssociationNode
{
    protected:
        bool is_root_;///<true if the node is root
        unsigned int det_idx_; ///< detection node index
        unsigned int tar_idx_; ///< target node index  
        double node_prob_; ///< Node Probability. Normalized->Conditional Probability that detection associates to target. Non-normalizeNodeProbs->Product of likelihoods.
        double tree_prob_; ///< Tree Probability. Joint Probability from the root node up to this (product of node probabilities)
        AssociationNode * up_node_ptr_; ///< Pointer to up node
        std::list<AssociationNode> node_list_; ///< List of nodes below of this in the association tree
        

    public:
        /** \brief Constructor
        * 
        * Constructor with arguments _det_idx and _tar_idx which indicates association of detection and target, 
        * with the probability _prob;
        * 
        */        
        AssociationNode(const unsigned int _det_idx, const unsigned int _tar_idx, const double _prob, AssociationNode * _un_ptr, bool _is_root = false);            
        
        /** \brief Destructor
        * 
        * Destructor
        * 
        */        
        virtual ~AssociationNode();
        
        /** \brief True if this is the root node
         * 
         * True if this is the root node, which is equivalent to check if up_node_ptr_ == NULL
         * 
         **/
        bool isRoot() const;

        /** \brief True if this is node is terminus (no more nodes below)
         * 
         * True if this is node is terminus (no more nodes below), which is equivalent to check node_list_.empty()
         * 
         **/        
        bool isTerminus() const; 
        
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
        
        /** \brief Sets node_prob_
         * 
         * Sets node_prob_
         * 
         **/
        void setNodeProb(double _np);        

        /** \brief Returns tree_prob_
         * 
         * Returns tree_prob_
         * 
         **/
        double getTreeProb() const;

        /** \brief Returns a copy of up_node_ptr_
         * 
         * Returns a copy of up_node_ptr_
         * 
         **/
        AssociationNode * upNodePtr() const;
        
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
        //double computeNodeProb(const unsigned int _nd, const unsigned int _nt, const unsigned int _di, const unsigned int _tj, const std::vector< std::vector<double> > & _stab) const;
        double computeNodeProb(const unsigned int _nd, const unsigned int _nt, const unsigned int _di, const unsigned int _tj, const Matrixx<double> & _stab) const;
        
        /** \brief Normalizes node probabilities recursively
         * 
         * Normalizes node probabilities recursively.
         * All node probs of node_list_ should sum 1
         * 
         **/
        void normalizeNodeProbs();        

        /** \brief Computes tree probabilities recursively
         * 
         * Computes tree probabilities recursively, while setting tree_prob_ data member
         * Updates the terminus node list, passed as second parameter, with all nodes that are terminus
         * \param _up_prob probability of upper node
         * \param _tn_list: List of terminus nodes. Filled with terminus nodes, while recurisve computing tree
         * 
         **/
        double computeTreeProb(const double & _up_prob, std::list<AssociationNode*> & _tn_list);
                
        /** \brief Grows tree recursively
         * 
         * Grows tree recursively according the association probability table provided
         * \param _det_i: detection index
         * \param _p_tab: table of association probabilities between detections and targets
         * \param _ex_vec: vector of target index for which the tree should not continue growing 
         * 
         **/        
        //void growTree(const unsigned int _nd, const unsigned int _nt, const unsigned int _det_i, const std::vector< std::vector<double> > & _stab, std::vector<unsigned int> & _excluded);
        void growTree(const unsigned int _nd, const unsigned int _nt, const unsigned int _det_i, const Matrixx<double> & _stab, std::vector<unsigned int> & _excluded);
        
        /** \brief Destroys tree
         * 
         * Recursively destroys tree
         * 
         **/
        void destroyTree();
        
        /** \brief Prints node info
         * 
         * Prints node info
         * 
         **/
        void printNode() const;
        
        /** \brief Prints the tree, by printing node info recursively
         * 
         * Prints the tree, by printing node info recursively
         * \param _ntabs Number of tabulators before printing. Useful for recursively print a whole tree
         * 
         * TODO: This function should be const, but we run recursively with an iterator over the node_list_ and compiler 
         * claims saying it can't return a const iterator.
         **/
        void printTree(const unsigned int _ntabs = 0);
};
#endif
