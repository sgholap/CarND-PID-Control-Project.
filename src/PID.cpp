#include "PID.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

PID::PID(bool isTwiddleMode, double tolerance):m_twiddleMode(isTwiddleMode), m_tolerance(tolerance) {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {

	// Initialize coefficient.
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
	cte_error.resize(3);
	m_p.resize(3);
	m_dp.resize(3);
	m_p[0] = Kp_;
	m_p[1] = Ki_;
	m_p[2] = Kd_;
	for (size_t i = 0; i < cte_error.size(); i++)
	{
		cte_error[i] = 0;
		m_dp[i] = 1;
	}
	state = PHASE_1;
	p_iteration = 0;
	m_bestError = std::numeric_limits<double>::max();
}

// THis is twiddle algorithm implement as per lecture.
bool PID::Twiddle(double error)
{
	std::cout << error << std::endl;
	while (true)
	{
		if (state == PHASE_1)
		{
			std::cout << "m_dp = " << m_dp[0] << " m_dp = " << m_dp[1] << " m_dp = " << m_dp[2] << std::endl;
			std::cout << "m_p = " << m_p[0] << " m_p = " << m_p[1] << " m_p = " << m_p[2] << std::endl;
			if ((abs(m_dp[0]) + abs(m_dp[1]) + abs(m_dp[2])) < m_tolerance)
			{
				std::cout << "Final parameter: P" << m_p[0] << " I " << m_p[1] << " D " << m_p[2] << std::endl;
				break;
			}
			else
			{
				state = PHASE_2;
			}
		}
		if (state == PHASE_2)
		{
			m_p[p_iteration] += m_dp[p_iteration];
			state = PHASE_3;
			break;
		}
		if (state == PHASE_3)
		{
			if (error < m_bestError)
			{
				m_bestError = error;
				m_dp[p_iteration] *= 1.1;
				state = PHASE_5;
			}
			else
			{
				m_p[p_iteration] -= 2 * m_dp[p_iteration];
				state = PHASE_4;
				break;
			}
		}
		if (state == PHASE_4)
		{
			if (error < m_bestError)
			{
				m_bestError = error;
				m_dp[p_iteration] *= 1.1;
				state = PHASE_5;
			}
			else
			{
				m_p[p_iteration] += m_dp[p_iteration];
				m_dp[p_iteration] *= 0.9;
				state = PHASE_5;
			}
		}
		if (state == PHASE_5)
		{
			p_iteration++;
			if (p_iteration >= m_p.size())
			{
				state = PHASE_1;
			}
			else
			{
				state = PHASE_2;
			}
			p_iteration = p_iteration % m_p.size();
		}
	}
	Kp = m_p[0];
	Ki = m_p[1];
	Kd = m_p[2];
	std::cout << "KP = " << Kp << "Ki = " << Ki << "Kd = " << Kd << " BestError" << m_bestError << std::endl;
	for (size_t i = 0; i < cte_error.size(); i++)
	{
		cte_error[i] = 0;
	}
	return true;
}

void PID::UpdateError(double cte) {
	cte_error[2] = cte - cte_error[0];
	cte_error[1] += cte;
	cte_error[0] = cte;
}

double PID::TotalError() {
  return  -Kp * cte_error[0] - Ki * cte_error[1] - Kd * cte_error[2];
}