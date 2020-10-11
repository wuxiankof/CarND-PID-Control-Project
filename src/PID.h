#ifndef PID_H
#define PID_H
#include <iostream>
#include <vector>

using std::vector;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
    
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  
  // mean squared error (MSE) 
  double MSE();
  
  // update PID
  void UpdateP(vector<double> P);
  
  // twiddle
  void Twiddle();

  // number of timesteps passed
  int iCounter; 
  
 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
    
  //other varialbes defined
  double prev_cte;
  double cte;
  double diff_cte;
  double int_cte;
  
  // variables used in the twiddle function
    
  int N;  // sample size to calcuate MSE
  vector<double> VectCTE; // vector of ctes
  
  vector<double> Vect_P;  // vector of [P, I, D] values
  vector<double> Vect_dP; // vector of [dP, dI, dD] values
    
  int idx; // index of current hyperprameters being fine-tuned
  double best_error; // current best error
    
  bool flag2;         
  bool flagNext; 
    
};

#endif  // PID_H
