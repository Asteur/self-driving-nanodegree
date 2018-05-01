#include "road.h"
#include "constants.h"

using namespace std;

//update vehicles per lane
void Road::update_road(vector<Vehicle> left_lane_vehicles,
		vector<Vehicle> center_lane_vehicles,
		vector<Vehicle> right_lane_vehicles) {
	this->left_lane_vehicles = left_lane_vehicles;
	this->center_lane_vehicles = center_lane_vehicles;
	this->right_lane_vehicles = right_lane_vehicles;
}

//get vehicles per lane
vector<Vehicle> Road::get_lane_vehicles(LANE lane) {
	vector<Vehicle> rlane;
	if (lane == LANE::LEFT) {
		rlane = this->left_lane_vehicles;
	} else if (lane == LANE::CENTER) {
		rlane = this->center_lane_vehicles;
	} else {
		rlane = this->right_lane_vehicles;
	}
	return rlane;
}

//get int value for lane
int Road::get_lane_value(LANE lane) {
	int lane_val = 0;
	if (lane == LANE::LEFT) {
		lane_val = 0;
	} else if (lane == LANE::CENTER) {
		lane_val = 1;
	} else {
		lane_val = 2;
	}
	return lane_val;
}

//check if the current lane is safe
bool Road::is_safe_lane(Vehicle& car, LANE lane) {
	vector<Vehicle> r_car_lane = this->get_lane_vehicles(lane);
	bool safe = true;
	for (int i = 0; i < r_car_lane.size(); i++) {
		double distance = r_car_lane[i].get_s() - car.get_s();
		if (distance > 0 && distance < SAFETY_DISTANCE) {
			safe = false;
		}
	}
	return safe;
}

//get minimum distance available per lane
double Road::get_free_lane(Vehicle& car, LANE lane) {
	vector<Vehicle> target_lane = this->get_lane_vehicles(lane);
	bool is_free = true;
	double min_dist = 999999999;
	for (int i = 0; i < target_lane.size(); i++) {
		double distance = std::abs(car.get_s()-target_lane[i].get_s());
		if (distance < min_dist) {
			min_dist = distance;
		}

	}
	return min_dist;
}

//check if possible collision in the future with the car ahead
bool Road::possible_collision_in_lane_ahead(Vehicle& car, int waypoints,LANE lane) {

	vector<Vehicle> target_lane = this->get_lane_vehicles(lane);


	double car_s = car.get_s();
	for (int i = 0; i < target_lane.size(); i++) {

		double future_s = target_lane[i].get_s()
				+ ((double) waypoints * AT * target_lane[i].get_v());

		if ((future_s > car_s) && (future_s - car_s < SAFETY_DISTANCE)) {
			return true;

		}

	}
	return false;
}
//check if possible collision in the future with the car behind
bool Road::possible_collision_in_lane_behind(Vehicle& car, int waypoints,LANE lane) {

	vector<Vehicle> target_lane = this->get_lane_vehicles(lane);


	double car_s = car.get_s();
	for (int i = 0; i < target_lane.size(); i++) {

		double future_s = target_lane[i].get_s()
				+ ((double) waypoints * AT * target_lane[i].get_v());

		if ((future_s < car_s) && ( car_s-future_s < SAFETY_DISTANCE)) {
			return true;

		}

	}
	return false;
}
//get best lane to switch to
LANE Road::get_best_lane(Vehicle& car) {
	LANE car_lane = car.lane();
	LANE target_lane = car_lane;

	//if current lane is left or right
	if (car_lane == LANE::LEFT || car_lane == LANE::RIGHT) {
		double car_lane_dist = this->get_free_lane(car, car_lane);
		double center_lane_dist = this->get_free_lane(car, LANE::CENTER);

		//select lane with maximum distance between the s values
		if (center_lane_dist >= SAFETY_DISTANCE_FOR_LANE_CHANGE && center_lane_dist > car_lane_dist) {

			target_lane = LANE::CENTER;
		} else {
			target_lane = car_lane;
		}

	} else {

		//if the lane is center lane
		double left_lane_dist = this->get_free_lane(car, LANE::LEFT);
		double right_lane_dist = this->get_free_lane(car, LANE::RIGHT);
		double car_lane_dist = this->get_free_lane(car, car_lane);

		//select lane with maximum distance between the s values
		if (left_lane_dist >= SAFETY_DISTANCE_FOR_LANE_CHANGE && left_lane_dist > right_lane_dist
				&& left_lane_dist > car_lane_dist) {
			target_lane = LANE::LEFT;
		} else if (right_lane_dist >= SAFETY_DISTANCE_FOR_LANE_CHANGE && right_lane_dist > left_lane_dist
				&& right_lane_dist > car_lane_dist) {
			target_lane = LANE::RIGHT;
		} else {
			target_lane = car_lane;
		}
	}
	return target_lane;
}
