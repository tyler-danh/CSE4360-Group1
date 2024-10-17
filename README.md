# cs4360-proj1
 EV3 Lego Robotics

```c
double PD_control(theta, theta_dot, theta_ref, theta_dot_ref)
double theta, theta_dot, theta_ref, theta_dot_ref;
{
  static double tau = 0;
  double Kp = 150.0;                                 // Proportional gain
  double Kd = 24.494897428;                          // Derivative gain
  double T0 = 3.10110;                               // Torque at theta ~ 0

  // Calculate error and its derivative
  // Approx derivative with difference
  double error = theta_ref - theta;                  // Position error
  double error_dot = theta_dot_ref - theta_dot;      // approx Velocity error

  // USE AS CONTROLLER: Apply PD control law
  tau = Kp * error + Kd * error_dot + T0;

  return tau;                                        // Return the computed torque
}
```
