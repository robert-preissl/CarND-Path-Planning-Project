
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <unordered_map>
#include <string.h>
#include <vector>

#include "behavior.h"
#include <math.h>

#include "spline.h"


const double T_INC            = 0.02; // time increment: Every 20 ms the car moves to the next point on the provided list of x,y points
const double MAX_S            = 6945.554; // length of track
const double MAX_D            = 12.0; // width of road
const double MAX_V            = 20.5; // maximum speed of our car in m/s
const double MIN_V            = 12.0; // minimum speed of our car in m/s
const double SPEED_UP_FACTOR  = 1.15; // speed up by this factor (if below speed limit)
const double SLOW_DOWN_FACTOR = 0.85; // slow down by this factor (if above min limit)
const double START_GOAL_V     = 5.0; // when the vehicle starts, this is its goal velocity
const int N_PREV_PATH         = 35; // if the previous path size goes below this number, we generate new points and extend the previous path. (we do not wait til 0, to have a more smooth path)

class Trajectory {

  private:
    std::unordered_map< std::string, std::vector<double> > map_data;

    tk::spline spline_x;
    tk::spline spline_y;
    tk::spline spline_dx;
    tk::spline spline_dy;

    // --------------------------------------------------------------

    tk::spline compute_spline(const double& start_s,
                              const std::string& map_key);

    tk::spline compute_spline(std::vector<double>& x,
                              std::vector<double>& y);

    void plot_spline_data(const double& start_s,
                          const double& end_s,
                          const tk::spline& spline,
                          const std::string name);

    std::vector<double> JMT(const std::vector<double>& start,
                            const std::vector<double>& end,
                            const double& T);

    std::tuple<double, double> frenet_to_cartesian(const double& s,
                                                   const double& d);

    void extend_trajectory(const double& T,
                           const std::vector<double>& jmt_s,
                           const std::vector<double>& jmt_d,
                           std::vector<double>& next_x_vals,
                           std::vector<double>& next_y_vals);

  public:
    Trajectory() {}
    Trajectory(std::unordered_map< std::string,
               std::vector<double> > map_data_) : map_data(map_data_) {}

    void calculate_trajectory(const traffic& own_car,
                              const int& state,
                              const int& goal_d,
                              const std::vector<double>& previous_path_x,
                              const std::vector<double>& previous_path_y,
                              std::vector<double>& next_x_vals,
                              std::vector<double>& next_y_vals,
                              double& prev_car_speed,
                              double& prev_car_s,
                              double& prev_car_d);
};



#endif // TRAJECTORY_H
