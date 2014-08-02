
#include "association_node.h"

AssociationNode::AssociationNode(const unsigned int _det_idx, const unsigned int _tar_idx, const double _prob, AssociationNode * _un_ptr) :
    det_idx_(_det_idx),
    tar_idx_(_tar_idx),
    node_prob_(_prob),
    tree_prob_(1.)
{
    up_node_ptr_ = _un_ptr;
}

AssociationNode::~AssociationNode()
{
    //
}

bool AssociationNode::isRoot() const
{
    if ( up_node_ptr_ == NULL ) return true;
    else return false; 
}

bool AssociationNode::isTerminus() const
{
    if ( node_list_.empty() ) return true;
    else return false;
}

unsigned int AssociationNode::getDetectionIndex() const
{
    return det_idx_;
}

unsigned int AssociationNode::getTargetIndex() const
{
    return tar_idx_;
}

double AssociationNode::getNodeProb() const
{
    return node_prob_;
}

double AssociationNode::getTreeProb() const
{
    return tree_prob_;
}

AssociationNode* AssociationNode::upNodePtr() const
{
    return up_node_ptr_;
}

double AssociationNode::computeNodeProb(const unsigned int _di, const unsigned int _tj, const std::vector< std::vector<double> > & _stab)
{
    double p_ij = 1.0; 
    
    if ( _tj == 0 ) //Case target 0 -> unassociated detection
    {
        //Prob detection _di does not match to other targets than _tj
        for (unsigned int kk=1; kk<_stab.at(_di).size(); kk++)
        {
            p_ij *= 1 - _stab.at(_di).at(kk); 
        }
    }
    else //General case
    {
        //step 1. Positive event
        p_ij *= _stab.at(_di).at(_tj); 
        
        //step2. Prob detection _di does not match to other targets than _tj
        for (unsigned int kk=0; kk<_stab.at(_di).size(); kk++)
        {
            if ( kk!=_tj ) p_ij *= 1 - _stab.at(_di).at(kk); 
        }
        
        //step3. Prob target _tj does not match to other detections than _di
        for (unsigned int kk=0; kk<_stab.size(); kk++)
        {
            if ( kk!=_di ) p_ij *= 1 - _stab.at(kk).at(_tj); 
        }
    }
    return p_ij;    
}

double AssociationNode::computeTreeProb(const double & _up_prob, std::list<AssociationNode*> & _tn_list)
{
    //compute joint probability
    tree_prob_ = _up_prob * node_prob_;
    
    //if terminus node, we have to add it to the terminus node list
    if ( isTerminus() ) 
    {
        _tn_list.push_back( &(*this) );
    }
    else //otherwise carry on recursivity
    {
        for (unsigned int ii=0; ii<node_list_.size(); ii++)
            node_list_.at(ii).computeTreeProb(tree_prob_, _tn_list);
    }
}

void AssociationNode::growTree(const unsigned int _det_i, const std::vector< std::vector<double> > & _stab, std::vector<unsigned int> & _ex_vec)
{
    unsigned int tar_j; //target index (not target id!)
    unsigned int nt; //num of targets
    double p_ij;//probability that detection i comes from target j
    
    if ( (_stab.size() == 0 ) || (_stab.at(_det_i).size() == 0 ) ) //check if no detections or no targets
    {
        return;
    }
    
    //Recursive growing loop
    nt = _stab.at(_det_i).size();//get num of target from the table size, including target void
    for (tar_j=0; tar_j<nt; tar_j++) //for each target, including target void
    {
        //check if target is at excluded vector, meaning that it is already associated at this branch. Not for tar_j=0 (void target)
        if ( (tar_j == 0) || ( std::find(_ex_vec.begin(), _ex_vec.end(), tar_j) == _ex_vec.end() ) )
        {
            //compute probability from the score table
            p_ij = computeNodeProb(_det_i, tar_j, _stab); //std::cout << __LINE__ << ": p_ij: " << p_ij << std::endl; 

            //Create node if prob is relevant, and carry on recursivity. Otherwise this current branch stops growing
            if (p_ij > PROB_ZERO_)
            {
                node_list_.push_back(AssociationNode(_det_i, tar_j, p_ij, this));
                if (_det_i+1 < _stab.size() ) //smaller than num detections
                {
                    _ex_vec.push_back(tar_j);
                    node_list_.back().growTree(_det_i+1, _stab, _ex_vec); //recursivity
                    _ex_vec.pop_back();
                }
            }
        }
    }    
}

void AssociationNode::printNode() const
{
    std::cout << "D" << det_idx_ << ",T" << tar_idx_ << ", np=" << node_prob_ << ", tp=" << tree_prob_ << std::endl;
}

void AssociationNode::printTree(const unsigned int _ntabs) const
{
    for(unsigned int ii=0; ii<_ntabs; ii++) std::cout << "\t";
    printNode();
    for (unsigned int ii=0; ii<node_list_.size(); ii++)
    {
            node_list_.at(ii).printTree(_ntabs+1);
    }
}
