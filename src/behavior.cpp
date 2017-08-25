
#include "behavior.h"
#include <float.h>


Behavior::Behavior() {
  state = STATE_KL;
  goal_d = 6.0;
}

bool Behavior::is_middle_lane(const double& d) {
  return ( (d > LANE_WIDTH) && ( d < LANE_WIDTH * 2) );
}

bool Behavior::is_left_lane(const double& d) {
  return ( (d >= 0.0) && ( d <= LANE_WIDTH) );
}

bool Behavior::is_right_lane(const double& d) {
  return ( (d >= LANE_WIDTH * 2) && ( d <= LANE_WIDTH * 3) );
}

// input: Sensor Fusion Data, a list of all other cars on the same side of the road.
// output: a map of surrounding traffic keeping closest cars for each lane (ahead, and behind our car)
void Behavior::sense_traffic(const json &sensor_fusion, double car_s) {

  std::vector<double> min_s_ahead = {DBL_MAX, DBL_MAX, DBL_MAX};
  std::vector<double> max_s_behind = {-DBL_MAX, -DBL_MAX, -DBL_MAX};

  // finding closest cars in all 3 lanes (ahead and behind)
  for (int i = 0; i < sensor_fusion.size(); ++i)
  {
    auto sens = sensor_fusion[i];
    double sens_s = sens[5];
    double sens_d = sens[6];

    if( is_middle_lane(sens_d) ) // middle lane
    {
      assign_close_traffic(sens_s,
                           car_s,
                           CTR_AHEAD,
                           CTR_BEHIND,
                           sensor_fusion,
                           i,
                           min_s_ahead[1],
                           max_s_behind[1]);
    }
    else if( is_left_lane(sens_d) ) { // car in left lane
        assign_close_traffic(sens_s,
                             car_s,
                             LEFT_AHEAD,
                             LEFT_BEHIND,
                             sensor_fusion,
                             i,
                             min_s_ahead[0],
                             max_s_behind[0]);
    }
    else if( is_right_lane(sens_d) ) {
      assign_close_traffic(sens_s,
                           car_s,
                           RIGHT_AHEAD,
                           RIGHT_BEHIND,
                           sensor_fusion,
                           i,
                           min_s_ahead[2],
                           max_s_behind[2]);
    }
  }

  print_surrounding_traffic();
}

// helper method to find the closest car ahead and behind us
void Behavior::assign_close_traffic(const double& sens_s,
                                    const double& car_s,
                                    const int& ahead,
                                    const int& behind,
                                    const json& sensor_fusion,
                                    const int& i,
                                    double& min_s_ahead,
                                    double& max_s_behind) {

  double s_dist = sens_s - car_s;  //  front: 10 - 5; 12 - 5  .. want min value  distance(car_x,car_y,sens_x,sens_y);
                                   //  back:  3  - 5; 1  - 5  .. want max value

  if (sens_s > car_s)
  {
    if (s_dist < min_s_ahead) // find closest car ahead
    {
      min_s_ahead = s_dist;
      traffic left_ahead = traffic( sensor_fusion, i );
      surrounding_traffic[ahead] = left_ahead;
    }
  }
  else // closest car behind
  {
    if (s_dist > max_s_behind)
    {
      max_s_behind = s_dist;
      traffic left_behind = traffic( sensor_fusion, i );
      surrounding_traffic[behind] = left_behind;
    }
  }
}

// helper method to print surrounding traffic
void Behavior::print_surrounding_traffic() {
  int label_value = 1;
  printf("\n Surrounding Traffic: \n");
  for( auto label_it = traffic_labels.begin(); label_it != traffic_labels.end(); ++label_it, ++label_value ) {
    auto found_label_it = surrounding_traffic.find( label_value );
    if( found_label_it != surrounding_traffic.end() ) {
      printf("  %s: ", label_it->c_str());
      surrounding_traffic[label_value].print();
    }
  }
  printf("\n");
}

// check for slow traffic ahead of us. if there is, try to change lanes if possible
void Behavior::check_slowdown(const double& my_s, const double& my_d, const double& my_v) {

  int ahead;
  if( is_middle_lane( my_d ) ) {
    ahead = CTR_AHEAD;
    goal_d = 6.0;
  } else if ( is_left_lane( my_d ) ) {
    ahead = LEFT_AHEAD;
    goal_d = 2.0;
  } else if ( is_right_lane( my_d ) ) {
    ahead = RIGHT_AHEAD;
    goal_d = 10.0;
  }

  state = STATE_KL; // this is the default desired state: we always prefer to stay in the lane

  std::unordered_map<int, traffic> :: const_iterator cit_c_a = surrounding_traffic.find(ahead);

  // if there is surrounding traffic (if none, stay in lane and go as fast as allowed)
  if( cit_c_a != surrounding_traffic.end() ) {

    double ahead_s = cit_c_a->second.s;
    double ahead_v = cit_c_a->second.vel;

    // if ahead traffic is too close
    if( fabs(ahead_s - my_s) < LC_DIST_AHEAD ) { // !! note, my_s, is the previous_s goal s. so if things get very close, i could be ahead
      // printf("\n AA1 -- traffic too close ahead!! my s = %f, my_v = %f, ahead_s = %f, ahead_v = %f \n", my_s, my_v, ahead_s, ahead_v);

      // if on center lane, try to get to left or right lane if possible
      //  if not possible, we slow down
      if( ahead == CTR_AHEAD ) {
        deal_with_slow_traffic(my_s, my_v, ahead_v, LEFT_BEHIND, LEFT_AHEAD, 3.0);

        if( state != STATE_LC ) // only deal with a potential lane change to the right if left one does not work.
          deal_with_slow_traffic(my_s, my_v, ahead_v, RIGHT_BEHIND, RIGHT_AHEAD, 9.0);
      }
      else {
        // if on outer lane, the goal is to get into the middle lane if there is slow traffic ahead
        deal_with_slow_traffic(my_s, my_v, ahead_v, CTR_BEHIND, CTR_AHEAD, 6.0);
      }

    }

  }

}

// input: my car's s and velocity. plus, the velocity of the vehicle ahead of us on same lane.
// output: determine if lane change is possible. if yes, set new state and new goal d value. (if applicable. no d change if we have to slow down)
void Behavior::deal_with_slow_traffic(const double& my_s,
                                      const double& my_v,
                                      const double& ahead_v,
                                      const int& behind,
                                      const int& ahead,
                                      const double& goal_d_) {

  std::unordered_map<int, traffic> :: const_iterator cit_b = surrounding_traffic.find(behind);
  std::unordered_map<int, traffic> :: const_iterator cit_a = surrounding_traffic.find(ahead);

  bool lane_change_ok = false;

  // no cars ahead and behind
  if( cit_b == surrounding_traffic.end() && cit_a == surrounding_traffic.end() ) {
    lane_change_ok = true;
    printf("\n no traffic ahead or behind on future lane, ahead = %d \n", ahead);
  }

  // cars ahead and behind
  else if( cit_b != surrounding_traffic.end() && cit_a != surrounding_traffic.end() ) {
    printf("\n my_s = %f, my_v = %f, left_behind_s = %f, left_ahead_s = %f, ahead_v = %f, behind_v = %f \n", my_s, my_v, cit_b->second.s, cit_a->second.s, cit_a->second.vel, cit_b->second.vel);
    // with enough distance
    if( (fabs(cit_a->second.s - my_s) > LC_DIST_AHEAD) &&
        (fabs(cit_b->second.s - my_s) > LC_DIST_BACK) &&
        (ahead_v < cit_a->second.vel ) && // only change lanes if ahead vehicle on current lane is slower as vehicle ahead on passing lane
        (my_v + 5 > cit_b->second.vel ) ) {  // and if the car behind is on passing lane is not way faster than we are
      lane_change_ok = true;
      // printf("\n AA2 \n");
    }
  }

  // no car behind, but car ahead.
  else if( cit_b == surrounding_traffic.end() ) {
    printf("\n left_ahead_s = %f, ahead_v = %f \n", cit_a->second.s, cit_a->second.vel);
    // car ahead far enough away
    if( (fabs(cit_a->second.s - my_s) > LC_DIST_AHEAD) &&
        (ahead_v < cit_a->second.vel ) ) {
      lane_change_ok = true;
      // printf("\n AA3 \n");
    }
  }

  // no car ahead, but car behind.
  else if( cit_a == surrounding_traffic.end() ) {
    printf("\n my_s = %f, my_v = %f, left_behind_s = %f, behind_v = %f \n", my_s, my_v, cit_b->second.s, cit_b->second.vel);
    // car behind far enough away
    if( (fabs(cit_b->second.s - my_s) > LC_DIST_BACK) &&
        (my_v + 5 > cit_b->second.vel ) ) { // if the car behind is on passing lane is not way faster than we are
      lane_change_ok = true;
      // printf("\n AA4 \n");
    }
  }

  if( lane_change_ok ) {
    state = STATE_LC;
    goal_d = goal_d_;
  }
  // no lane change possible. at least behind or ahead car too close
  else {
    // printf("\n AA5 \n");
    state = STATE_KL_SD;
  }

}
