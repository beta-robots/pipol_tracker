
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

void AssociationTree::solve(std::vector<std::pair<unsigned int, unsigned int> > & _pairs, std::vector<unsigned int> &  _unassoc)
{
    std::list<AssociationNode*>::iterator it, bestNode;
    double bestProb = 0.;
    bool rootReached = false;
    AssociationNode *anPtr;
    
    double sum = 0;
    
    //check if terminus_node_list_ is empty
    if ( terminus_node_list_.empty() ) return;
    
    //choose best node based on best tree probability
    for (it = terminus_node_list_.begin(); it != terminus_node_list_.end(); it++)
    {
        if ( (*it)->getTreeProb() > bestProb ) 
        {
            bestNode = it;
            bestProb = (*it)->getTreeProb();
        }
        
        //debugging 
        sum += (*it)->getTreeProb();
    }
    //std::cout << "treeDecision(): "; (*bestNode)->printNode();
    //std::cout << "sum: " << sum << std::endl;
    //std::cout << "bestProb: " << bestProb << std::endl;
    
    //set pairs
    anPtr = *bestNode; //init pointer
    int ii=0;
    while( ! anPtr->isRoot() ) //set pairs
    {
        if ( anPtr->getTargetIndex() == nt_) //detection with void target -> unassociated detection
        {
            _unassoc.push_back(anPtr->getDetectionIndex());
        }
        else
        {
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
