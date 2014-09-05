
#include <stdlib.h> //srand, rand 
#include <time.h>  //time
#include <iostream>
#include "../association_tree.h"


int main(int argc, char *argv[])
{
    //variables
    AssociationTree tree_;
    unsigned int nd_ = 2; //2 detections
    unsigned int nt_ = 3; //3 targets
    double prob_;
    std::vector<std::pair<unsigned int, unsigned int> > associations_;
    
    //inits
    srand ( time(NULL) );
    
    //splash
    std::cout << "-------------------------------------------------" << std::endl;
    std::cout << "             TEST ASSOCIATION TREE" << std::endl;
    std::cout << "-------------------------------------------------" << std::endl;
        
    std::cout << "TEST 1. Resize and Fill score table" << std::endl;
    tree_.resizeScoreTable(nd_,nt_);
    for (unsigned int ii=0; ii<nd_; ii++)
    {
        for (unsigned int jj=0; jj<nt_+1; jj++)
        {
            prob_ = (double)rand()/(double)RAND_MAX;
            tree_.setScore(ii,jj,prob_);
        }
    }
    tree_.printScoreTable();
    
    std::cout << "TEST 2. Reset " << std::endl;
    tree_.reset();
    tree_.printScoreTable();
    tree_.printTerminusNodes();

    std::cout << "TEST 3. Grow the tree" << std::endl;
    tree_.resizeScoreTable(nd_,nt_);
    for (unsigned int ii=0; ii<nd_; ii++)
    {
        for (unsigned int jj=0; jj<nt_+1; jj++)
        {
            prob_ = (double)rand()/(double)RAND_MAX;
            tree_.setScore(ii,jj,prob_);
        }
    }    
    tree_.printScoreTable();
    tree_.buildTree();
    tree_.computeTree();
    tree_.printTree();
    //tree_.printTerminusNodes();
    tree_.bestHypothesis(associations_);
    std::cout << "BEST ASSOCIATION EVENT: " << std::endl;
    for(unsigned int ii=0; ii< associations_.size(); ii++)
        std::cout << associations_.at(ii).first << "," << associations_.at(ii).second << " ";
    std::cout << std::endl; 
    tree_.reset();
    
    //end
    return 0;
}
