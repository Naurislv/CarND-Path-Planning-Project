#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>

using namespace std;

class GNB {
public:

	// vector<string> possible_labels = {"left","keep","right"};

	// bayesian params consist of of all parameters needed for
	// prediction phase structure is as follows:
	// bayesian_params['label_name'] = <vector>(label_prob, feature_0_mean,
	// feature_0_variance, feature_1_mean, feature_1_variance, ...)
	std::map<string, std::vector< double >> bayesian_params;


	/**
  	* Constructor
  	*/
 	GNB();

	/**
 	* Destructor
 	*/
 	virtual ~GNB();

 	void train(vector<vector<double> > data, vector<string>  labels);

  	string predict(vector<double> data);

};

#endif
