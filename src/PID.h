#ifndef PID_H
#define PID_H


#include<vector>


enum STATE {
	PHASE_1,
	PHASE_2,
	PHASE_3,
	PHASE_4,
	PHASE_5
};

class PID {
 public:
  /**
   * Constructor
   */
  PID(bool isTwiddle, double tolerance);

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

  bool Twiddle(double error);

 private:
  /**
   * PID Errors
   */
  std::vector<double> cte_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  bool m_twiddleMode;
  double m_bestError;
  std::vector<double> m_dp;
  std::vector<double> m_p;
  double m_tolerance;

  STATE state;
  int p_iteration;

};

#endif  // PID_H