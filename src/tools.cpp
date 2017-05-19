#include <fstream>
#include <iostream>
#include <sstream>
#include "tools.h"
#include "matplotlibcpp.h"


using namespace std;
namespace plt = matplotlibcpp;


Tools::Tools() {
	load_waypoints(m_xvals, m_yvals);
}

Tools::~Tools() {}


void Tools::load_waypoints(vector<double>& xvals, vector<double>&  yvals) {

	string in_file_name_ = "../lake_track_waypoints.csv";
	ifstream in_file_(in_file_name_.c_str(), ifstream::in);
	cout << "process file" << in_file_name_ <<  endl;
	if (!in_file_.is_open()) {
		cerr << "Cannot open input file: " << in_file_name_ << endl;
		return;
	}

	string line;
	bool first_line =true;
	while (getline(in_file_, line)) {
		if(first_line){
			//pass the first line, which is label x,y
			first_line = false;
			continue;
		}
		istringstream iss(line);
		double x;

		double y;
		iss >> x;
		if (iss.peek() == ',')
			iss.ignore();
		iss >> y;
		xvals.push_back(x);
		yvals.push_back(y);

	}
}
void Tools::show_reference_trajectory(const vector<double>& xvals, const vector<double>&  yvals){

	plt::title("referecne trajectory");
	plt::plot(xvals, yvals);
	plt::plot(xvals, yvals, "ro");
	plt::show();

}
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd Tools::polyfit(const vector<double>& xvals_vec, const vector<double>&  yvals_vec,
                        int order) {
  assert(xvals_vec.size() == yvals_vec.size());
  assert(order >= 1 && order <= xvals_vec.size() - 1);

  Eigen::VectorXd xvals;
  Eigen::VectorXd yvals;

  xvals.resize(xvals_vec.size());
  yvals.resize(yvals_vec.size());

  for (int i = 0; i < xvals.size(); i++) {
	  xvals[i] = xvals_vec[i];
	  yvals[i] = yvals_vec[i];
  }

  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

/**
 * get some of the closetest way points for the vehicle.
 * @param x vehicle position x
 * @param y vehicle position y
 * @param N number of reference waypoints to select
 * @output xvals slected waypoints's position
 * @output yvals slected waypoints's position
 */
void Tools::get_reference_points(vector<double>& xvals, vector<double>&  yvals, double x, double y,  int N){
	const int total_size = m_xvals.size();
	int start_pos = -1;
	double clostest_dist = -1;
	for(int i=0; i< total_size; i++ ){
		double dist = pow(x -m_xvals[i], 2) + pow(y -m_yvals[i], 2);
		if(clostest_dist == -1 || dist < clostest_dist ){
			clostest_dist = dist;
			start_pos = i;
		}
	}

	int num =0;
	while(num < N){
		xvals.push_back(m_xvals[start_pos]);
		yvals.push_back(m_yvals[start_pos]);

		start_pos++;
		if(start_pos == total_size){
			start_pos = 0;
		}
		num++;
	}
}
/**
 * Transform the points from map coordiante to the vehicle coordinate system.
 * @param xvals, map coordinate, x position
 * @param yvals, map coordinate, y position
 * @param vehicle_x, vehicle position
 * @param vehicle_y, vehicle position
 * @param vehicle_theta, vehicle orientation
 * @output xvals, transformed output, x position
 * @output yvals, transformed output, y position
 */
void Tools::transform_map_coord(vector<double>& xvals,vector<double>& yvals, double vehicle_x, double vehicle_y, double vehicle_theta) {

	vector<double> transformed_x;
	vector<double> transformed_y;
	int total_size = xvals.size();

	for (int i = 0; i < total_size; i++) {

		double new_x;
		double new_y;

		double cos_theta = cos(vehicle_theta - M_PI / 2);
		double sin_theta = sin(vehicle_theta - M_PI / 2);
		new_x = -(xvals[i] - vehicle_x) * sin_theta + (yvals[i] - vehicle_y) * cos_theta;
		new_y = -(xvals[i] - vehicle_x) * cos_theta - (yvals[i] - vehicle_y) * sin_theta;

		transformed_x.push_back(new_x);
		transformed_y.push_back(new_y);
	}
	xvals= transformed_x;
	yvals = transformed_y;

	return;
}

void Tools::test(){
	vector<double> xvals;
	vector<double> yvals;
	load_waypoints(xvals, yvals);
	show_reference_trajectory(xvals, yvals);

}


