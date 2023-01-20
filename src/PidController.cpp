/*
 * PidController.cpp
 *
 *  Created on: 23 Feb 2017
 *      Author: sleutene
 */

#include <arp/PidController.hpp>
#include <cmath>       /* pow */
#include <stdexcept>

namespace arp {

// Set the controller parameters
void PidController::setParameters(const Parameters & parameters)
{
  parameters_ = parameters;
}

// Implements the controller as u(t)=c(e(t))
double PidController::control(uint64_t timestampMicroseconds, double e,
                              double e_dot)
{
  // TODO: implement...
  
  uint64_t deltaT = timestampMicroseconds * pow(10,-6) - lastTimestampMicroseconds_ * pow(10,-6); 

  if(deltaT > 0.1){
    deltaT = 0.1;
  }

  // do some smoothing (numeric derivatives are noisy):
  integratedError_ = 0.8 * integratedError_ + 0.2 * e_dot;
  // compute output:
  double output = parameters_.k_p * e + parameters_.k_i * integratedError_ + parameters_.k_d * e_dot;
  
  // saturate:
  if (output < minOutput_) {
  output = minOutput_; // clamp -- and DO NOT INTEGRATE ERROR (anti-reset windup)
  } else if (output > maxOutput_) {
  output = maxOutput_; // clamp -- and DO NOT INTEGRATE ERROR (anti-reset windup)
  } else {
    integratedError_ += e * deltaT; // safe to keep integrating
  }
  // save:
  lastTimestampMicroseconds_ = timestampMicroseconds;

  return output;

}

// Reset the integrator to zero again.
void PidController::setOutputLimits(double minOutput, double maxOutput)
{
  minOutput_ = minOutput;
  maxOutput_ = maxOutput;
}

/// \brief
void PidController::resetIntegrator()
{
  integratedError_ = 0.0;
}

}  // namespace arp
