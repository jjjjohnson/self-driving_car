vector<double(Vehicle::*)(vector<State>, map<int,vector <vector<int>>>, TrajectoryData)> cost_functions;
cost_functions.push_back(&Vehicle::change_lane_cost)

tk::spline spline_x, spline_y;
spline_x.set_points(map_waypoints_s, map_waypoints_x);
spline_y.set_points(map_waypoints_s, map_waypoints_y);

vector<double> smooth_waypoints_s;
vector<double> smooth_waypoints_x;
vector<double> smooth_waypoints_y;

// The track is 6945.554 meters around (about 4.32 miles).
// Refine the path with spline:
int spline_samples = 24000;
for (size_t i = 0; i < spline_samples; ++i) {
    smooth_waypoints_x.push_back(spline_x(i));
    smooth_waypoints_y.push_back(spline_y(i));
    smooth_waypoints_s.push_back(i);
}