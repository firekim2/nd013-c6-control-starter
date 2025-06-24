/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  
   cte = 0;
   diff_cte = 0;
   int_cte = 0;
  
   Kp = Kpi;
   Ki = Kii;
   Kd = Kdi;
   output_lim_max = output_lim_maxi;
   output_lim_min = output_lim_mini;
}


void PID::UpdateError(double _cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   diff_cte = (_cte - cte)/delta_time;   
   cte = _cte;
   int_cte += cte * delta_time;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */

    double control = Kp * cte + Kd * diff_cte + Ki * int_cte;
    if (control > output_lim_max){
      return output_lim_max;
    }
    if (control < output_lim_min){
      return output_lim_min;
    }

    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  delta_time = new_delta_time;
  return delta_time;
}