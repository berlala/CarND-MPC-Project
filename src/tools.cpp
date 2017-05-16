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
		exit(EXIT_FAILURE);
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
// Evaluate a polynomial.
double Tools::polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

/**
 * get some of the closetest way points for the vehicle.
 * @param x vehicle position x
 * @param y vehicle position y
 * @param phi vehicle orientation
 * @param N number of reference waypoints to select
 * @param conversion whether to convert the map coordinates to vehicle coordinate
 * @output xvals slected waypoints's position
 * @output yvals slected waypoints's position
 */
void Tools::get_reference_points(vector<double>& xvals, vector<double>&  yvals, double x, double y, double psi, int N, bool conversion){
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
	if(!conversion){
		return;
	}

}
void Tools::test(){
	vector<double> xvals;
	vector<double> yvals;
	load_waypoints(xvals, yvals);
	show_reference_trajectory(xvals, yvals);

}


