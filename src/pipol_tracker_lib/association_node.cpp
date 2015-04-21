
#include "association_node.h"

AssociationNode::AssociationNode(const unsigned int _det_idx, const unsigned int _tar_idx, const double _prob, AssociationNode * _un_ptr, bool _is_root) :
    is_root_(_is_root),
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
//     if ( up_node_ptr_ == NULL ) return true;
//     else return false; 
    return is_root_;
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

void AssociationNode::setNodeProb(double _np)
{
    node_prob_ = _np;
}

double AssociationNode::getTreeProb() const
{
    return tree_prob_;
}

AssociationNode* AssociationNode::upNodePtr() const
{
    //return &(*up_node_ptr_);
    return up_node_ptr_;
}

//double AssociationNode::computeNodeProb(const unsigned int _nd, const unsigned int _nt, const unsigned int _di, const unsigned int _tj, const std::vector< std::vector<double> > & _stab) const
double AssociationNode::computeNodeProb(const unsigned int _nd, const unsigned int _nt, const unsigned int _di, const unsigned int _tj, const Matrixx<double> & _stab) const
{
    double p_ij = 1.0; 
    
    if ( _tj == _nt ) //Case void target -> unassociated detection
    {
        //Prob detection _di does not match to other targets than _tj
        for (unsigned int kk=0; kk<_nt; kk++)
        {
            //p_ij *= ( 1.0 - _stab.at(_di).at(kk) );
            p_ij *= ( 1.0 - _stab(_di,kk) ); 
        }
    }
    else //General case
    {
        //step 1. Positive matching _di with _tj
//         p_ij *= _stab.at(_di).at(_tj);
        p_ij *= _stab(_di,_tj); 
        
        //step2. Prob detection _di does not match to other targets than _tj
        for (unsigned int kk=0; kk<_nt; kk++)
        {
//             if ( kk!=_tj ) p_ij *= 1 - _stab.at(_di).at(kk);
            if ( kk!=_tj ) p_ij *= 1 - _stab(_di,kk); 
        }
        
        //step3. Prob target _tj does not match to other detections than _di
        for (unsigned int kk=0; kk<_nd; kk++)
        {
//             if ( kk!=_di ) p_ij *= 1 - _stab.at(kk).at(_tj);
            if ( kk!=_di ) p_ij *= 1 - _stab(kk,_tj); 
        }
        
        //step4. Prob detection _di does not remain unassociated
        double p_un = 1.0;
        for (unsigned int kk=0; kk<_nt; kk++)
        {
            //p_un *= ( 1.0 - _stab.at(_di).at(kk) );
            p_un *= ( 1.0 - _stab(_di,kk) );                        
        }
        p_ij *= ( 1 - p_un );
    }
    return p_ij;    
}

void AssociationNode::normalizeNodeProbs()
{
    double pSum = 0;
    std::list<AssociationNode>::iterator it;
    
    for(it = node_list_.begin(); it != node_list_.end(); it++)
        pSum += it->getNodeProb();

    for(it = node_list_.begin(); it != node_list_.end(); it++)
    {
        it->setNodeProb(it->getNodeProb()/pSum);
        if (!isTerminus()) 
            it->normalizeNodeProbs();
    }
}

double AssociationNode::computeTreeProb(const double & _up_prob, std::list<AssociationNode*> & _tn_list)
{
    std::list<AssociationNode>::iterator it;
    
    //compute joint probability
    tree_prob_ = _up_prob * node_prob_;
    
    //if terminus node, we have to add it to the terminus node list
    if ( isTerminus() ) 
    {
        _tn_list.push_back( &(*this) );
    }
    else //otherwise carry on recursivity
    {
        for(it = node_list_.begin(); it != node_list_.end(); it++)
            it->computeTreeProb(tree_prob_, _tn_list);
    }
}

//void AssociationNode::growTree(const unsigned int _nd, const unsigned int _nt, const unsigned int _det_i, const std::vector< std::vector<double> > & _stab, std::vector<unsigned int> & _excluded)
void AssociationNode::growTree(const unsigned int _nd, const unsigned int _nt, const unsigned int _det_i, const Matrixx<double> & _stab, std::vector<unsigned int> & _excluded)
{
    unsigned int tar_j; //target index (not target id!)
    double p_ij;//probability that detection i comes from target j
        
    //Recursive growing loop
    for (tar_j=0; tar_j<_nt+1; tar_j++) //for each target, including target void
    {
        //Carry on growing if target is void OR if target is not at excluded vector
        if ( (tar_j == _nt) || ( std::find(_excluded.begin(), _excluded.end(), tar_j) == _excluded.end() ) )
        {
            //compute probability from the score table
            p_ij = computeNodeProb(_nd, _nt, _det_i, tar_j, _stab); //std::cout << __LINE__ << ": p_ij: " << p_ij << std::endl; 

            //Create node if prob is relevant, and carry on recursivity. Otherwise this current branch stops growing
            if (p_ij > PROB_ZERO_)
            {
                node_list_.push_back(AssociationNode(_det_i, tar_j, p_ij, this));
                if (_det_i+1 < _nd ) //check if there is more detections to carry on recursivity 
                {
                    _excluded.push_back(tar_j);//push back target to excluded vector
                    node_list_.back().growTree(_nd, _nt, _det_i+1, _stab, _excluded); //recursivity
                    _excluded.pop_back();//pop back target to excluded vector
                }
            }
        }
    }    
}

void AssociationNode::destroyTree()
{
    node_list_.clear();
}

void AssociationNode::printNode() const
{
    std::cout << "D" << det_idx_ << ",T" << tar_idx_ << ", np=" << node_prob_ << ", tp=" << tree_prob_ << std::endl;
    //std::cout << "D" << det_idx_ << ",T" << tar_idx_ << ", np=" << node_prob_ << ", tp=" << tree_prob_ << ", this=" << this << ", up_ptr=" << up_node_ptr_ << std::endl;
}

void AssociationNode::printTree(const unsigned int _ntabs) 
{
    std::list<AssociationNode>::iterator it;
    
    for(unsigned int ii=0; ii<_ntabs; ii++) std::cout << "\t";
    printNode();
    
    //for (unsigned int ii=0; ii<node_list_.size(); ii++)
    //  node_list_.at(ii).printTree(_ntabs+1);
    for(it = node_list_.begin(); it != node_list_.end(); it++) it->printTree(_ntabs+1);
}
