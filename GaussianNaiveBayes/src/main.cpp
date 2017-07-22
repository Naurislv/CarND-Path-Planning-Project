#include "classifier.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

vector<vector<double> > Load_State(string file_name)
{
    ifstream in_state_(file_name.c_str(), ifstream::in);
    vector< vector<double > > state_out;
    string line;
    
    while (getline(in_state_, line)) 
    {
    	istringstream iss(line);
    	vector<double> x_coord;

		string token;
		getline(iss, token, ',');
    	double state1 = stod(token);
		getline(iss, token, ',');
	    double state2 = stod(token);;
		getline(iss, token, ',');
	    double state3 = stod(token);;
		getline(iss, token, ',');
	    double state4 = stod(token);;
	    
		x_coord.push_back(state1);
	    x_coord.push_back(state2);
    	x_coord.push_back(state3);
	    x_coord.push_back(state4);
    
	    state_out.push_back(x_coord);
    }
    return state_out;
}
vector<string> Load_Label(string file_name)
{
    ifstream in_label_(file_name.c_str(), ifstream::in);
    vector< string > label_out;
    string line;
    while (getline(in_label_, line)) 
    {
    	istringstream iss(line);
    	string label;
	    iss >> label;
    
	    label_out.push_back(label);
    }
    return label_out;
    
}

void AddFeatures(vector<vector<double>>& data) {
	/*
	But d % lane_width might be helpful since it gives the relative
	position of a vehicle in it's lane regardless of which lane the
	vehicle is in.
	*/
	float lane_width = 4;

	cout << "Adding Features" << endl;
	int n_data_points = data.size();
	
	for (int i = 0; i < n_data_points; i++) {
		double s = data[i][0];  // longitudial distance along road
		double d = data[i][1];  // lateral distance along road
		double s_dot = data[i][2];  // longitudial acceleration
		double d_dot = data[i][3];  // lateral acceleration
		
		float relative_pos = fmod(d, lane_width);
		// cout << d << " " << relative_pos << endl;

		//data[i].push_back(relative_pos);
		//data[i].push_back(s * s_dot);
		data[i].push_back(d * d_dot);
	}
}

int main() {

    vector< vector<double> > X_train = Load_State("../nd013_pred_data/train_states.txt");
    vector< vector<double> > X_test  = Load_State("../nd013_pred_data/test_states.txt");
    vector< string > Y_train  = Load_Label("../nd013_pred_data/train_labels.txt");
    vector< string > Y_test   = Load_Label("../nd013_pred_data/test_labels.txt");

	cout << "X_train number of elements " << X_train.size() << endl;
	cout << "X_train element size " << X_train[0].size() << endl;
	cout << "Y_train number of elements " << Y_train.size() << endl;

	GNB gnb = GNB();
	
	AddFeatures(X_train);
	gnb.train(X_train, Y_train);

	cout << "X_test number of elements " << X_test.size() << endl;
	cout << "X_test element size " << X_test[0].size() << endl;
	cout << "Y_test number of elements " << Y_test.size() << endl;
	
	int score = 0;
	AddFeatures(X_test);

	cout << endl << "Testing with " << X_test[0].size() << " features" << endl << endl;
	for(int i = 0; i < X_test.size(); i++)
	{
		vector<double> coords = X_test[i];
		string predicted = gnb.predict(coords);
		if(predicted.compare(Y_test[i]) == 0)
		{
			score += 1;
		}
	}

	float fraction_correct = float(score) / Y_test.size();
	cout << "You got " << (100*fraction_correct) << " correct" << endl;

	return 0;
}
