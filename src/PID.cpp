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
  
  this->N = 100;
  this->iCounter = 0;
  VectCTE = vector<double>(N, 0.0);
  
  Vect_P = {Kp_,  Ki_, Kd_};
  Vect_dP = {0.1, 0.001, 1};
  idx = 0;
  best_error = 10000.0;
  flag2 = false;
  flagNext = false;
  tol = 0.2;

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

void PID::UpdateP(vector<double> P){
  
  this->Kp = P[0];
  this->Ki = P[1];
  this->Kd = P[2];
  
}

void PID::Twiddle(){
  
  if ( (iCounter > N) && (Vect_P[0]+Vect_P[1]+Vect_P[2] > tol) && (iCounter % N==0) ){
    
    if(iCounter == 2*N){
      best_error = MSE();
      Vect_P[idx] += Vect_dP[idx];
      UpdateP(Vect_P);
    }
    else{
      double err = MSE();
      if (err < best_error){
        best_error = err;
        Vect_dP[idx] *= 1.1;
        if(flag2)
          flag2 = false;
        Vect_P[idx] += Vect_dP[idx];
        UpdateP(Vect_P);
        flagNext = true;
      }
      else{
        if(!flag2){
          Vect_P[idx] -= 2 * Vect_dP[idx];
          flag2 = true;
        }
        else {
          Vect_P[idx] += Vect_dP[idx];
          Vect_dP[idx] *= 0.9;
          Vect_P[idx] += Vect_dP[idx];
          flagNext = true;
        }
        UpdateP(Vect_P);
      } 
      
      if (flagNext){
        idx = (idx+1) % 3;
        flagNext = false;
      }
    }
    
     // DEBUG
     std::cout << "Index: " << idx << ", CurPID: " << Vect_P[0] << ", "  << Vect_P[1] << ", " << Vect_P[2] << ", "<< "MSE: "<< MSE() << std::endl;
  }
}
