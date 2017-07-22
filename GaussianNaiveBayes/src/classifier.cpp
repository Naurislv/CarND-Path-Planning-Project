#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <algorithm>
#include <map>
#include "classifier.h"

/**
 * Initializes GNB
 */
GNB::GNB() {

}

GNB::~GNB() {}

void GNB::train(vector<vector<double> > data, vector<string> labels)
{

	/*
		Trains the classifier with N data points and labels.

		INPUTS
		data - array of N observations
		  - Each observation is a tuple with 4 values: s, d, 
		    s_dot and d_dot.
		  - Example : [
			  	[3.5, 0.1, 5.9, -0.02],
			  	[8.0, -0.3, 3.0, 2.2],
			  	...
		  	]

		labels - array of N labels
		  - Each label is one of "left", "keep", or "right".
	*/

	int n_data_points = data.size();
	int n_features = data[0].size();

	cout << endl << "Training with " << n_features << " features" << endl << endl;

	std::map<string, vector< double > > unique_labels;
	std::map<string, vector<vector<double> > > label_features;

	for (int i = 0; i < n_data_points; i++) {

		// Count labels
		string label = labels[i];
		if(unique_labels.count(label) == 0) {
			unique_labels[label].push_back(1); // label count
			
			for (int k = 0; k < n_features; k++) {  // kth feature sum position
				unique_labels[label].push_back(0);
			}
		}
		else {
			unique_labels[label][0] += 1;
		}

		label_features[label].push_back(data[i]);
		for (int k = 0; k < n_features; k++) {  // kth feature
			unique_labels[label][k + 1] += data[i][k];
		}
	}
	
	for (const auto &p : unique_labels) {
		// calculate probability of each feature
		double label_prob = unique_labels[p.first][0] / n_data_points;
		bayesian_params[p.first].push_back(label_prob);
		
		std::cout << "unique_labels[" << p.first << "] = " << p.second[0];

		for (int k = 0; k < n_features; k++) {  // kth feature
			// calculate mean value of each feature for each label
			double theta = unique_labels[p.first][k + 1] / p.second[0];
			bayesian_params[p.first].push_back(theta);

			// iterate through label features to calculate population variance
			//  because it's based on mean
			double sigma = 0;
			
			for (int i = 0; i < label_features[p.first].size(); i++) {
				sigma += pow((label_features[p.first][i][k] - theta), 2);
			}
			bayesian_params[p.first].push_back(sigma / p.second[0]);
		}
		
		// Print all bayesian parameters
		//for (int k = 0; k < bayesian_params[p.first].size(); k++) {  // kth feature
		//	std::cout << " " << bayesian_params[p.first][k];
		//}
		std::cout << '\n';
	}
}

string GNB::predict(vector<double> data)
{
	/*
		Once trained, this method is called and expected to return 
		a predicted behavior for the given observation.

		INPUTS

		observation - a 4 tuple with s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
		# TODO - complete this
	
	Implementation explanation:
		P(left| feature1, feature2, feature3, feature4) =
		P(left) * P(feature1| left) * P(feature2| left) *
		P(feature3| left) * P(feature4| left)

		We are assuming that each feature is independed (althoght it is
		not necessarly true) so: P(feature1, feature2, feature3, feature4) = 
		P(feature1)*P(feature2)*P(feature3)*P(feature4)

	*/

	// Iterate through each class

	float highest_prob = -100;

	string pred_label = "NONE";
	for (const auto &p : bayesian_params) {
		string label = p.first;
		double class_log_prob = log(p.second[0]);

		// Ierate through each feature (theta, sigma)
		float n_ij_sum = 0;
		float n_ij_sum2 = 0;
		for (int i = 1; i < p.second.size(); i+=2) {
			// calcule sum of all features
			float theta = p.second[i];
			float sigma = p.second[i + 1];

			n_ij_sum += log(2.0 * M_PI * sigma);
			n_ij_sum2 += pow((data[(i - 1) / 2] - theta), 2) / sigma;
		}

		float log_prob = - 0.5 * n_ij_sum;
		log_prob -= 0.5 * n_ij_sum2;
		log_prob += class_log_prob;

		// cout << label << " => Prob: " << log_prob << " Class Prob: " << class_log_prob << endl;
		
		if (log_prob > highest_prob) {
			pred_label = label;
			highest_prob = log_prob;
		}

		//n_ij = - 0.5 * np.sum(np.log(2. * np.pi * self.sigma_[i, :]))
		//n_ij -= 0.5 * np.sum(((X - self.theta_[i, :]) ** 2) /
		//						(self.sigma_[i, :]), 1)
		//joint_log_likelihood.append(jointi + n_ij)
	}

	//cout << "Predicted " << pred_label << " " << exp(highest_prob); 
	//cout << endl;

	return pred_label.c_str();

}
