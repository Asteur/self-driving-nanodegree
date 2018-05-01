#include "PID.h"
#include <limits>
#include <iostream>
using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {
}

PID::~PID() {
}

void PID::Init(double Kp, double Ki, double Kd) {
	//initialize parameters
	K = {Kp,Ki,Kd};
	//initialize dp for Kp,Kd and Ki
	dp = {0.1*Kp,0.1*Ki,0.1*Kd};
	//terminating criteria for twiddle termination
	tol = .01;
	//best error initialization
	best_error = numeric_limits<double>::max();

}

void PID::UpdateError(double cte) {
	//update p ,d and i errors
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;

}

double PID::TotalError() {
	//return sterring angle
	double steer = -K[0] * p_error - K[2] * d_error - K[1] * i_error;
	return steer;

}
//twiddle to learn parameters
double PID::Twiddle(bool twiddle, double cte) {

	UpdateError(cte);

	if (twiddle) {

		std::cout << " Kp:  " << K[0] << "  Kd:  " << K[2] << "  Ki:  " << K[1]
				<< " best_error " << best_error << " twiddleState "
				<< twiddleState << "total d " << dp[0] + dp[1] + dp[2]
				<< std::endl;

		//check if termination is reached
		if (dp[0] + dp[1] + dp[2] > tol) {

			double error = cte;
			//if initial state increment parameter by dp
			if (twiddleState == 0) {
				K[index] += dp[index];
				//change state to incremented state
				twiddleState = 1;
			} else if (twiddleState == 1) {
				//if state is previously incremented and error lower then best error
				if (error < best_error) {
					//set best error
					best_error = error;
					//increment probing interval
					dp[index] *= 1.1;
					//set state to initial state
					twiddleState = 0;
					//change parameter index
					index = (index + 1) % 3;

				} else {
					//if state is previously incremented and error not lower decrement by 2*dp
					K[index] -= 2 * dp[index];
					//change state to previously to decremented
					twiddleState = 2;
				}

			} else if (twiddleState == 2) {
				//if state is previously decremented and  and error lower then best error
				if (error < best_error) {
					//set best error
					best_error = error;
					//increment probing interval
					dp[index] *= 1.1;

				} else {
					//restore parameter value
					K[index] += dp[index];
					//decrement probing interval
					dp[index] *= .9;

				}
				//change parameter index
				index = (index + 1) % 3;
				//set state to initial state
				twiddleState = 0;

			}
			//return steering angle
			return TotalError();

		} else {
			//if model converged
			if (cte < best_error) {

				best_error = cte;
			}
			//keep state as initial state
			twiddleState = 0;
			//return steering angle
			return TotalError();
		}
	} else {
		//if twiddle no used
		//return steering angle based on initialized parameter
		return TotalError();
	}

}

