
#include "association_tree.h"

AssociationTree::AssociationTree() :
    root_(0,0,1)
{
    //
}

AssociationTree::~AssociationTree()
{
    //
}

void AssociationTree::reset()
{
    for ( unsigned int ii = 0; ii < scores_.size(); ii++)
    {
        scores_.at(ii).clear();
    }
    scores_.clear();
}
        
void AssociationTree::resizeScoreTable(const unsigned int _n_det, const unsigned int _n_tar)
{
    scores_.resize(_n_det);
    for ( unsigned int ii = 0; ii < scores_.size(); ii++)
    {
        scores_.at(ii).resize(_n_tar+1);//"+1" to account for the unassociated detections (target "void")
    }
}

unsigned int AssociationTree::numDetections()
{
    return scores_.size();
}
     
unsigned int AssociationTree::numTargets()
{
    if (scores_.size() == 0 )
        return 0;
    else
        return scores_.at(0).size();
}
     
void AssociationTree::setScore(const unsigned int _det_i, const unsigned int _tar_j, const double _p_ij)
{
    scores_.at(_det_i).at(_tar_j) = _p_ij;
}
     
void AssociationTree::buildTree()
{
    std::vector<unsigned int> ex_vec;
    root_.growTree(0,scores_, ex_vec);
}

void AssociationTree::computeTree()
{
    root_.computeTreeProb(1.);
}

void AssociationTree::printScoreTable() const
{
    for (unsigned int ii=0; ii<scores_.size(); ii++)
    {
        for (unsigned int jj=0; jj<scores_.at(ii).size(); jj++)
        {
            std::cout << scores_.at(ii).at(jj) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}
    
void AssociationTree::printTree() const
{
    root_.print();
}

