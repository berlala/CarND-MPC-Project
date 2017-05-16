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
  void load_waypoints(Eigen::VectorXd& xvals_res, Eigen::VectorXd& yvals_res);
  void test();
  void show_reference_trajectory(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals);
  Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                          int order);
  double polyeval(Eigen::VectorXd coeffs, double x);


};

#endif /* TOOLS_H_ */
