#ifndef CONTANTS_H
#define CONTANTS_H

#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <string>

using namespace std;

const double TRACK_DISTANCE = 6945.554; //m
const double LANE_WIDTH = 4.0; //m
const double REF_VEL = 49.5; //miles/hour
const double VEL_INCR=.224*2;
const double AT = 0.02; //s
const double POINTS = 50;
const double SAFETY_DISTANCE = 20.0; //m
const double SPLINE_DISTANCE = 30.0; //m
const double SAFETY_DISTANCE_FOR_LANE_CHANGE = 20.0; //m

enum class LANE { LEFT, CENTER, RIGHT };

#endif // CONTANTS_H
