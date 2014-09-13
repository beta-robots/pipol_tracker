
#include <stdlib.h> //srand, rand 
#include <time.h>  //time
#include <iostream>
#include "../association_tree.h"


int main(int argc, char *argv[])
{
    //variables
    AssociationTree tree_;
    unsigned int nd_ = 4; //# detections
    unsigned int nt_ = 2; //# targets
    std::vector<std::pair<unsigned int, unsigned int> > associations_;
    std::vector<unsigned int> unassociated_;
    
    //inits
    srand ( time(NULL) );
    
    //splash
    std::cout << "-------------------------------------------------" << std::endl;
    std::cout << "             TEST ASSOCIATION TREE" << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;
        
    std::cout << "TEST 1. Resize and Fill score table" << std::endl;
    tree_.resize(nd_,nt_);
    for (unsigned int ii=0; ii<tree_.numDetections(); ii++)
        for (unsigned int jj=0; jj<tree_.numTargets()+1; jj++)
            tree_.setScore(ii,jj,(double)rand()/(double)RAND_MAX);
    tree_.printScoreTable();
    
    std::cout << "TEST 2. Reset, Resize & Fill score table  " << std::endl;
    tree_.reset();
    tree_.resize(nd_-1,nt_);
    for (unsigned int ii=0; ii<tree_.numDetections(); ii++)
        for (unsigned int jj=0; jj<tree_.numTargets()+1; jj++)
            tree_.setScore(ii,jj,(double)rand()/(double)RAND_MAX);    
    tree_.printScoreTable();
    tree_.printTerminusNodes();

    std::cout << "TEST 3. Grow the tree" << std::endl;
    tree_.growTree();
std::cout << __LINE__ << std::endl;
    tree_.computeTree();
std::cout << __LINE__ << std::endl;    
    tree_.treeDecision(associations_,unassociated_);
std::cout << __LINE__ << std::endl;    
    
    //display tree
    tree_.printTree();

    //display associations
    std::cout << "BEST ASSOCIATION EVENT: " << std::endl;
    std::cout << "   PAIRS: ";
    for(unsigned int ii=0; ii< associations_.size(); ii++)
        std::cout << associations_.at(ii).first << "," << associations_.at(ii).second << " ";
    std::cout << std::endl; 
    std::cout << "   UNASSOCIATED DETs: ";
    for(unsigned int ii=0; ii< unassociated_.size(); ii++)
        std::cout << unassociated_.at(ii) << ", ";
    std::cout << std::endl; 
    tree_.reset();
    
    //end
    return 0;
}

