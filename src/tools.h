#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class StringException : public std::exception
{
public:
   std::string s;
   StringException(std::string ss) : s(ss) {}
   ~StringException() throw () {} // Updated
   const char* what() const throw() { return s.c_str(); }
};

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  void load_waypoints(vector<double>& xvals, vector<double>&  yvals);
  void test();
  void show_reference_trajectory(const vector<double>& xvals, const vector<double>&  yvals);
  Eigen::VectorXd polyfit(const vector<double>& xvals_vec, const vector<double>&  yvals_vec,
                          int order);
  double polyeval(Eigen::VectorXd coeffs, double x);


};

#endif /* TOOLS_H_ */
