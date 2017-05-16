#include <fstream>
#include <iostream>
#include <sstream>
#include "tools.h"
#include "matplotlibcpp.h"


using namespace std;
namespace plt = matplotlibcpp;


Tools::Tools() {}

Tools::~Tools() {}


void Tools::load_waypoints(Eigen::VectorXd& xvals_res, Eigen::VectorXd& yvals_res) {

	vector<double> xvals;
	vector<double>  yvals;
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
	xvals_res.resize(xvals.size());
	yvals_res.resize(xvals.size());
	for (int i = 0; i < xvals.size(); i++) {
		xvals_res[i] = xvals[i];
		yvals_res[i] = yvals[i];
//		cout<<xvals_res[i] <<","<< yvals_res[i] <<endl;
	}

}
void Tools::show_reference_trajectory(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals){
	vector<double> xvals_vec;
	vector<double>  yvals_vec;
	for (int i = 0; i < xvals.size(); i++) {
		xvals_vec.push_back(xvals[i]);
		yvals_vec.push_back(yvals[i]);
	}
	plt::title("referecne trajectory");
	plt::plot(xvals_vec, yvals_vec);
	plt::plot(xvals_vec, yvals_vec, "ro");
	plt::show();

}
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd Tools::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
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
void Tools::test(){
	Eigen::VectorXd xvals;
	Eigen::VectorXd yvals;
	load_waypoints(xvals, yvals);
	show_reference_trajectory(xvals, yvals);

}


