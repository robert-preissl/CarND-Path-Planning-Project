

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <unordered_map>
#include <string.h>
#include <vector>

#include "json.hpp"
using json = nlohmann::json;

// states
#define STATE_KL     1 // keep lane state (and go as fast as possible)
#define STATE_KL_SD  2 // keep lane state, but slow down
#define STATE_LC     3 // lane change state

struct traffic {
  double id;
  double x;
  double y;
  double vel_x;
  double vel_y;
  double vel;
  double s;
  double d;
  double yaw;

  traffic() {}

  traffic(double id_, double x_, double y_, double vel_x_, double vel_y_, double s_, double d_, double yaw_) :
           id(id_), x(x_), y(y_), vel_x(vel_x_), vel_y(vel_y_), s(s_), d(d_), yaw(yaw_) { vel = sqrt(vel_x*vel_x + vel_y*vel_y); }

  traffic(double id_, double x_, double y_, double vel_, double s_, double d_ , double yaw_) :
           id(id_), x(x_), y(y_), vel(vel_), s(s_), d(d_), yaw(yaw_) { vel_x = 0.0; vel_y = 0.0; }

  traffic(const json &sensor_fusion, int json_index) {
    auto sens = sensor_fusion[json_index];
    id = sens[0];
    x = sens[1];
    y = sens[2];
    vel_x = sens[3];
    vel_y = sens[4];
    vel = sqrt(vel_x*vel_x + vel_y*vel_y);
    s = sens[5];
    d = sens[6];
  }

  void print() {
    printf( "Car Id: %f => x: %f, y: %f, vel_x: %f, vel_y: %f, vel: %f, s: %f, d: %f \n",
                 id, x, y, vel_x, vel_y, vel, s, d);
  }

};

#define LEFT_BEHIND  1
#define LEFT_AHEAD   2
#define CTR_BEHIND   3
#define CTR_AHEAD    4
#define RIGHT_BEHIND 5
#define RIGHT_AHEAD  6

const std::vector<std::string> traffic_labels = {"LEFT_BEHIND", "LEFT_AHEAD", "CTR_BEHIND", "CTR_AHEAD", "RIGHT_BEHIND", "RIGHT_AHEAD"};

const int LANE_WIDTH = 4;

const int LC_DIST_AHEAD = 20;
const int LC_DIST_BACK = 25;

class Behavior {

  private:
    int state;
    double goal_d;

    std::unordered_map<int, traffic> surrounding_traffic;
    void print_surrounding_traffic();

    void assign_close_traffic(const double& sens_s,
                              const double& car_s,
                              const int& ahead,
                              const int& behind,
                              const json& sensor_fusion,
                              const int& i,
                              double& min_s_front,
                              double& max_s_back);

    void deal_with_slow_traffic(const double& my_s,
                                const double& my_v,
                                const double& ahead_v,
                                const int& behind,
                                const int& ahead,
                                const double& goal_d_);

    bool is_middle_lane(const double& d);

    bool is_left_lane(const double& d);

    bool is_right_lane(const double& d);

  public:
    Behavior();

    int get_state() {
      return state;
    }

    double get_goal_d() {
      return goal_d;
    }

    void sense_traffic(const json &sensor_fusion, double car_s);

    void clear_surrounding_traffic() {
      surrounding_traffic.clear();
    }

    void check_slowdown(const double& my_s, const double& my_d, const double& my_v);

};


#endif // BEHAVIOR_H
