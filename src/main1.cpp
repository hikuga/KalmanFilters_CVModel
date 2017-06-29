#include <iostream>
#include <algorithm>
#include <numeric>
#include <string>
#include <vector>
#include "tools.h"
using namespace std;

void test_rmse()
{
    Tools tools;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;
    
    //the input list of estimations
    VectorXd e(4);
    e << 1, 1, 0.2, 0.1;
    estimations.push_back(e);
    e << 2, 2, 0.3, 0.2;
    estimations.push_back(e);
    e << 3, 3, 0.4, 0.3;
    estimations.push_back(e);
    
    //the corresponding list of ground truth values
    VectorXd g(4);
    g << 1.1, 1.1, 0.3, 0.2;
    ground_truth.push_back(g);
    g << 2.1, 2.1, 0.4, 0.3;
    ground_truth.push_back(g);
    g << 3.1, 3.1, 0.5, 0.4;
    ground_truth.push_back(g);
    
    cout << "estimations: " << endl << accumulate(begin(estimations), end(estimations), string{""},[](string ans,VectorXd  itm){
        return ans +  " , " + to_string(itm(0)) + to_string(itm(1)) + to_string(itm(2)) + to_string(itm(3));
        
    })<< endl;
    //cout << "ground_truth: " << endl << ground_truth << endl;
    //call the CalculateRMSE and print out the result
    cout << tools.CalculateRMSE(estimations, ground_truth) << endl;
    
    
    
}

int main()
{
    vector<int> v1(4);
    iota( begin(v1), end(v1), 2);
    
    copy(begin(v1), end(v1), std::ostream_iterator<int>(std::cout, " "));
    cout << endl << "version: " <<__cplusplus << endl;
    test_rmse();

return 0;
}


