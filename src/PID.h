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
  
  void UpdateP(vector<double> P);
  
  // twiddle
  void Twiddle();

  int iCounter; // number of timesteps passed
  
  
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
    
  //WX defined
  double prev_cte;
  double cte;
  double diff_cte;
  double int_cte;
  
  int N = 100;  // sample size to calcuate MSE
  vector<double> VectCTE; // vector of cte
  
  vector<double> Vect_P; 
  vector<double> Vect_dP;
  int idx;
  double best_error;
  bool flag2; 
  bool flagNext; 
  double tol; 
    
};

#endif  // PID_H
