
#include "trajectory.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"


//#define _PLOTTING_ 1

#ifdef _PLOTTING_
  #include "matplotlibcpp.h"
  namespace plt = matplotlibcpp;
#endif



// input: state about our vehicle and goal lane from the behavior analyzer
// output: an extended trajectory (append to previous non-visited points of trajectory)
void Trajectory::calculate_trajectory(const traffic& own_car,
                                      const int& state,
                                      const int& goal_d,
                                      const std::vector<double>& previous_path_x,
                                      const std::vector<double>& previous_path_y,
                                      std::vector<double>& next_x_vals,
                                      std::vector<double>& next_y_vals,
                                      double& prev_car_speed,
                                      double& prev_car_s,
                                      double& prev_car_d) {

  int path_size = previous_path_x.size();

  // Copy old path
  for(int i = 0; i < path_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double long_start, long_goal, long_start_v, long_goal_v, long_start_a, long_goal_a;
  double lat_start, lat_goal, lat_start_v, lat_goal_v, lat_start_a, lat_goal_a;

  long_start_a = long_goal_a = 0.0;
  lat_start_a = lat_goal_a = 0.0;

  double T; // time horizon

  // Initialize path to current position if no path
  if(path_size == 0) {
    T = 2.0; // to make the start a bit smoother, we choose a prediction horizon of 2 seconds. -> 100 = 50 * 2 points in the trajectory
    long_start   = own_car.s;
    long_goal_v  = START_GOAL_V;
    long_goal    = long_start + long_goal_v * T;
    long_start_v = own_car.vel;

    lat_start = own_car.d;
    lat_goal  = 6.0;
    lat_start_v = 0.0;
    lat_goal_v  = 0.0;

    prev_car_speed = long_goal_v;
    prev_car_s = long_goal;
    prev_car_d = lat_goal;
  }
  else if(path_size < N_PREV_PATH) { // Calculate angle and get position from previous data
    T = 1.5;
    long_start   = prev_car_s;

    if( state == STATE_KL )
      long_goal_v = fmin(prev_car_speed * SPEED_UP_FACTOR, MAX_V); // acceleration
    else if( state == STATE_KL_SD || state == STATE_LC )
      long_goal_v = fmax(prev_car_speed * SLOW_DOWN_FACTOR, MIN_V); // slow down

    if (state == STATE_LC)
      T = 3.0; // for a lane chane use a longer prediction horizon to allow for smoother lane change

    long_goal    = long_start + long_goal_v * T;
    long_start_v = prev_car_speed;

    lat_start = prev_car_d;
    lat_goal  = goal_d;
    lat_start_v = 0.0;
    lat_goal_v  = 0.0;

    prev_car_speed = long_goal_v;
    prev_car_s = long_goal;
    prev_car_d = lat_goal;
  }

  if( path_size < N_PREV_PATH ) {

    // compute splines (with frenet s as "x" values) around the current s values of the car
    spline_x = compute_spline(long_start, "x");
    spline_y = compute_spline(long_start, "y");
    spline_dx = compute_spline(long_start, "dx");
    spline_dy = compute_spline(long_start, "dy");

    // plot_spline_data(long_start, long_goal, spline_x, "x_s");

    // feed in start-states and end-states into polynomial solver along with the desired duration
    //  and we get a jerk minimizing trajectory to the goal
    std::vector<double> jmt_start_state(3);
    std::vector<double> jmt_goal_state(3);

    jmt_start_state[0] = long_start;
    jmt_start_state[1] = long_start_v;
    jmt_start_state[2] = long_start_a;

    jmt_goal_state[0] = long_goal;
    jmt_goal_state[1] = long_goal_v;
    jmt_goal_state[2] = long_goal_a;

    printf("\n XX1 -- long. traj: long_start = %f, long_start_v = %f, long_start_a = %f ", jmt_start_state[0], jmt_start_state[1], jmt_start_state[2]);
    printf("\n XX2 -- long. traj: long_goal = %f, long_goal_v = %f, long_goal_a = %f ", jmt_goal_state[0], jmt_goal_state[1], jmt_goal_state[2]);

    // longitudinal trajectory
    std::vector<double> jmt_s = JMT(jmt_start_state, jmt_goal_state, T);

    jmt_start_state[0] = lat_start;
    jmt_start_state[1] = lat_start_v;
    jmt_start_state[2] = lat_start_a;

    jmt_goal_state[0] = lat_goal;
    jmt_goal_state[1] = lat_goal_v;
    jmt_goal_state[2] = lat_goal_a;

    printf("\n XX3 -- lat. traj: lat_start = %f, lat_start_v = %f, lat_start_a = %f ", jmt_start_state[0], jmt_start_state[1], jmt_start_state[2]);
    printf("\n XX4 -- long. traj: lat_goal = %f, lat_goal_v = %f, lat_goal_a = %f \n", jmt_goal_state[0], jmt_goal_state[1], jmt_goal_state[2]);

    // lateral trajectory
    std::vector<double> jmt_d = JMT(jmt_start_state, jmt_goal_state, T);

    // extend the trajectory (consisting of previous x,y path) by new cartesian coordinates
    //  using the jerk minimal s and d trajectories
    extend_trajectory(T,
                      jmt_s,
                      jmt_d,
                      next_x_vals,
                      next_y_vals);

  }

}

// input:  frenet s and d coordinates
// output: cartesian coordinates
std::tuple<double, double> Trajectory::frenet_to_cartesian(const double& s, const double& d) {
  return std::make_tuple(spline_x(s) + spline_dx(s) * d, spline_y(s) + spline_dy(s) * d);
}

// input: starting s value of new trajectory and the identifier of map data we want to approximate
// output: spline through map data
tk::spline Trajectory::compute_spline(const double& start_s, const std::string& map_key ) {
  std::vector<double> x;
  std::vector<double> y;

  std::vector<double> s_map_data = map_data["s"];
  int s_length = s_map_data.size();
  std::vector<double> mapped_data = map_data[map_key];

  int index;
  for( index = 0; index < s_map_data.size(); ++index ) {
    // printf("\n   SS2 index = %d ", index);
    if( s_map_data[index] > start_s ) {
      break;
    }
  }

  // take two before and four data points after middle
  x.push_back(s_map_data[ (index - 2 + s_length) % s_length ]);
  x.push_back(s_map_data[ (index - 1 + s_length) % s_length ]);
  x.push_back(s_map_data[ index % s_length ]);
  x.push_back(s_map_data[ (index + 1) % s_length ]);
  x.push_back(s_map_data[ (index + 2) % s_length ]);
  x.push_back(s_map_data[ (index + 3) % s_length ]);

  y.push_back(mapped_data[ (index - 2 + s_length) % s_length ]);
  y.push_back(mapped_data[ (index - 1 + s_length) % s_length ]);
  y.push_back(mapped_data[ index % s_length ]);
  y.push_back(mapped_data[ (index + 1) % s_length ]);
  y.push_back(mapped_data[ (index + 2) % s_length ]);
  y.push_back(mapped_data[ (index + 3) % s_length ]);

  return compute_spline(s_map_data, mapped_data);
}

// input: start and end s value and a spline for y-data
// output: graph showing (x,y) data points, x is in between start & end, and where y are points on spline.
void Trajectory::plot_spline_data(const double& start_s,
                                  const double& end_s,
                                  const tk::spline& spline,
                                  const std::string name) {
  std::vector<double> y_data;
  std::cout << " x : ";
  for(double i = start_s; i < end_s; i = i + 0.5) {
    y_data.push_back( spline( i ) );
    std::cout << i << ", ";
  }
  std::cout << std::endl;

  std::cout << " y : ";
  for(int i = 0; i < y_data.size(); ++i) {
    std::cout << y_data[i] << ", ";
  }
  std::cout << std::endl;

  // plt::named_plot(name, x_data, y_data);
}

// input: x and y floating point vectors
// output: spline from spline library
tk::spline Trajectory::compute_spline(std::vector<double>& x, std::vector<double>& y) {
  tk::spline spline;
  spline.set_points(x, y);
  return spline;
}

// input: - time horizon T (in seconds)
//        - jmt for s and d
// ouput: - extend the existing x,y cartesian coordinates with new x,y along the s,d jmts
void Trajectory::extend_trajectory(const double& T,
                                   const std::vector<double>& jmt_s,
                                   const std::vector<double>& jmt_d,
                                   std::vector<double>& next_x_vals,
                                   std::vector<double>& next_y_vals) {

  // for plotting s, d graphs
  // std::vector<double> vec_t; std::vector<double> vec_s; std::vector<double> vec_d;

  #pragma omp parallel for shared(T, T_INC, next_x_vals, next_y_vals)
  for(int i = 0; i < (int)(T / T_INC) ; i++) {

    double t = T_INC * i;
    // vec_t.push_back(t);

    double s = fmod( jmt_s[0] + jmt_s[1] * t + jmt_s[2] * pow(t, 2) + jmt_s[3] * pow(t, 3) +
                     jmt_s[4] * pow(t, 4) + jmt_s[5] * pow(t, 5), MAX_S);
    // vec_s.push_back(s);
    double d = fmod( jmt_d[0] + jmt_d[1] * t + jmt_d[2] * pow(t, 2) + jmt_d[3] * pow(t, 3) +
                     jmt_d[4] * pow(t, 4) + jmt_d[5] * pow(t, 5), MAX_D);
    // vec_d.push_back(d);
    auto cartesian_coords = frenet_to_cartesian( s, d );

    next_x_vals.push_back( std::get<0>(cartesian_coords) );
    next_y_vals.push_back( std::get<1>(cartesian_coords) );
  }

  /*printf("\n t = ");
  for (int i = 0; i < (int)(T / T_INC) ; i++) {
    printf("%f, ", vec_t[i]);
  }
  printf("\n");

  printf("\n s = ");
  for (int i = 0; i < (int)(T / T_INC) ; i++) {
    printf("%f, ", vec_s[i]);
  }
  printf("\n");

  printf("\n d = ");
  for (int i = 0; i < (int)(T / T_INC) ; i++) {
    printf("%f, ", vec_d[i]);
  }
  printf("\n"); printf("\n");*/

}

//   Calculate the Jerk Minimizing Trajectory that connects the initial state
//   to the final state in time T.
//
// input:   start - the vehicles start location given as a length three array
//                  corresponding to initial values of [s, s_dot, s_double_dot]
//          end   - the desired end state for vehicle. Like "start" this is a
//                  length three array.
//          T     - The duration, in seconds, over which this maneuver should occur.
//
// output:  an array of length 6, each value corresponding to a coefficent in the polynomial
//           s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
std::vector<double> Trajectory::JMT(const std::vector<double>& start,
                                    const std::vector<double>& end,
                                    const double& T)
{
  Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;

	Eigen::MatrixXd B = Eigen::MatrixXd(3,1);
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];

	Eigen::MatrixXd Ai = A.inverse();

	Eigen::MatrixXd C = Ai*B;

	std::vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}

  return result;
}
