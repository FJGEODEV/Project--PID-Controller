#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double KpInit, double KiInit, double KdInit,
		double alpha_p,double alpha_i,double alpha_d) {
	PID:: Kp = KpInit;
	PID:: Ki = KiInit;
	PID:: Kd = KdInit;

	d_error = alpha_d;
	p_error = alpha_p;
	i_error = alpha_i;

}

void PID::UpdateError(double cte, double dt) {

	//Derivative control
	d_error = (cte - p_error) /dt ;

	//Proportional control
	p_error = cte;

	//Intergrated crosscheck error
	i_error += (cte*dt);

}

double PID::TotalError() {

	double t_error;
	t_error = Kp * p_error + Kd * d_error + Ki * i_error;
	return t_error;
}

