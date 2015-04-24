
#include "association_tree.h"

AssociationTree::AssociationTree() :
    AssociationSolver(),
    root_(0,0,1, NULL, true)
{
    //
}

AssociationTree::~AssociationTree()
{
    //
}

void AssociationTree::reset()
{
    nd_ = 0; 
    nt_ = 0;
    scores_.clear(); 
    terminus_node_list_.clear();
    root_.destroyTree();
}

void AssociationTree::resize(const unsigned int _n_det, const unsigned int _n_tar)
{
    nd_ = _n_det;
    nt_ = _n_tar;
    scores_.resize(nd_,nt_+1); //"+1" to account for void target, which manages unassociated detections
}

             
void AssociationTree::growTree()
{
    std::vector<unsigned int> ex_vec;
    
    if ( nd_ != 0 ) //check if detections
        root_.growTree(nd_, nt_, 0,scores_, ex_vec);
}

void AssociationTree::computeTree()
{
    if ( nd_ != 0 ) //check if detections
        root_.computeTreeProb(1., terminus_node_list_);
}

void AssociationTree::normalizeTree()
{
    root_.normalizeNodeProbs(); 
}

void AssociationTree::chooseBestTerminus(std::list<AssociationNode*>::iterator & _best_node)
{
    std::list<AssociationNode*>::iterator it;
    double bestProb = 0.;
    //double sum = 0;
    
    for (it = terminus_node_list_.begin(); it != terminus_node_list_.end(); it++)
    {
        if ( (*it)->getTreeProb() > bestProb ) 
        {
            _best_node = it;
            bestProb = (*it)->getTreeProb();
        }
        
        //debugging 
        //sum += (*it)->getTreeProb();
    }   
}

//void AssociationTree::solve(std::vector<std::pair<unsigned int, unsigned int> > & _pairs, std::vector<unsigned int> &  _unassoc)
void AssociationTree::solve(std::vector<std::pair<unsigned int, unsigned int> > & _pairs, std::vector<bool> & _associated_mask)
{
    std::list<AssociationNode*>::iterator best_node;
    bool rootReached = false;
    AssociationNode *anPtr;
    
    //grows tree exploring all likely hypothesis
    growTree();
    
    //computes tree probs
    computeTree();
    
    //normalizes tree probs
    normalizeTree();

    //if terminus_node_list_ is empty exit withou pairing
    if ( terminus_node_list_.empty() ) return;
    
    //choose best node based on best tree probability
    chooseBestTerminus(best_node);
    
    //resize _associated_mask and resets it to false
    _associated_mask.resize(nd_,false);

    //set pairs
    anPtr = *best_node; //init pointer
    int ii=0;
    while( ! anPtr->isRoot() ) //set pairs
    {
//         if ( anPtr->getTargetIndex() == nt_) //detection with void target -> unassociated detection
//         {
//             _unassoc.push_back(anPtr->getDetectionIndex());
//         }
//         else
//         {
//             _pairs.push_back( std::pair<unsigned int, unsigned int>(anPtr->getDetectionIndex(), anPtr->getTargetIndex()) );
//         }
        if ( anPtr->getTargetIndex() < nt_ ) //association pair
        {
            _associated_mask.at(anPtr->getDetectionIndex()) = true; 
            _pairs.push_back( std::pair<unsigned int, unsigned int>(anPtr->getDetectionIndex(), anPtr->getTargetIndex()) );
        }
        anPtr = anPtr->upNodePtr();
    }        
}
    
void AssociationTree::printTree()
{
    if ( scores_.size() != 0 )
    {
        std::cout << "Nd: " << nd_ << "; Nt: " << nt_ << std::endl;
        root_.printTree();
    }
}

void AssociationTree::printTerminusNodes()
{
    std::list<AssociationNode*>::iterator it;
    unsigned int ii; 
    
    for (it = terminus_node_list_.begin(), ii=0; it != terminus_node_list_.end(); it++, ii++)
    {
        (*it)->printNode();
    }
}
