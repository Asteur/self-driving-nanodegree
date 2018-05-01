#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include "constants.h"

using namespace std;

class Vehicle {


  protected:

    int id;
    double x;
    double y;
    double v;
    double s;
    double d;
    double yaw;

    vector<double> previous_s;
    vector<double> previous_d;

  public:

    Vehicle();
    Vehicle(int id, double x, double y, double v, double s, double d);
    ~Vehicle(){};

    int get_id();
    double get_x();
    double get_y();
    double get_v();
    double get_s();
    double get_d();
    double get_yaw();
    LANE lane();

    void update_vehicle_values(double x, double y, double v, double s, double d, double yaw);
    void set_previous_s(vector<double> previous_s);
    void set_previous_d(vector<double> previous_d);
    vector<double> prev_s();
    vector<double> prev_d();

};

#endif // VEHICLE_H
