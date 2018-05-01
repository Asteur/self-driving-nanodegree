#ifndef ROAD_H
#define ROAD_H

#include <string>
#include <vector>
#include <cmath>
#include "vehicle.h"
#include "constants.h"

using namespace std;

class Road {

protected:
	vector<Vehicle> left_lane_vehicles;
	vector<Vehicle> center_lane_vehicles;
	vector<Vehicle> right_lane_vehicles;

public:
	Road() {
	}
	;
	~Road() {
	}
	;

	void update_road(vector<Vehicle> left_lane_vehicles,
			vector<Vehicle> center_lane_vehicles,
			vector<Vehicle> right_lane_vehicles);
	vector<Vehicle> get_lane_vehicles(LANE lane);

	bool is_safe_lane(Vehicle& car, LANE lane);
	double get_free_lane(Vehicle& car, LANE lane);
	LANE get_best_lane(Vehicle& car);
	int get_lane_value(LANE lane);
	bool possible_collision_in_lane_ahead(Vehicle& car, int waypoints,LANE lane);
	bool possible_collision_in_lane_behind(Vehicle& car, int waypoints,LANE lane);
};

#endif // ROAD_H
