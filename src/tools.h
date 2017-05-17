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
  * load way points for the track
  */
  void load_waypoints(vector<double>& xvals, vector<double>&  yvals);
  void test();
  void show_reference_trajectory(const vector<double>& xvals, const vector<double>&  yvals);
  Eigen::VectorXd polyfit(const vector<double>& xvals_vec, const vector<double>&  yvals_vec,
                          int order);
  /**
    * get some of the closetest way points for the vehicle
    */
  void get_reference_points(vector<double>& xvals, vector<double>&  yvals, double x, double y, int N=5);
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
  void transform_map_coord(vector<double>& xvals,vector<double>& yvals, double vehicle_x, double vehicle_y, double vehicle_theta);
private:
  /**
  * reference way points for the track
  */
  vector<double> m_xvals;
  vector<double> m_yvals;

};

#endif /* TOOLS_H_ */
