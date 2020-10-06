#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
    this->Kp = Kp_;
    this->Ki = Ki_;
    this->Kd = Kd_;
    
    this->prev_cte = 0;
    this->cte = 0;
    this->diff_cte = 0;
    this->int_cte = 0;
  
  this->iCounter = 0;
  VectCTE = vector<double>(N, 0.0);

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
    
    this->cte = cte;
    this->diff_cte = this->cte - this->prev_cte;
    this->int_cte += cte;
    
    this->prev_cte = cte;
  
  this->VectCTE[iCounter % N] = cte*cte;
  this->iCounter += 1;  

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
    double steering = -Kp * cte - Ki * int_cte - Kd * diff_cte;
    
    return steering;  // TODO: Add your total error calc here!
}

 // mean squared error (MSE) 
double PID::MSE(){
   
   if (this->iCounter < N)
     return -1.0;
   else{
     double sse = 0.0;
     for(std::vector<double>::iterator it = this->VectCTE.begin(); it != this->VectCTE.end(); ++it)
       sse += *it;
     
     return (sse / N);
   }
 }
