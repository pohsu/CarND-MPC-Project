## CarND-Controls-MPC
---
### The Model
We adopt the same model given by the MPC quiz:
```c++
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] = cte1 - ((y0 - f0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] =  epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```
The states are: `x, y, psi, v, cte, epsi` and the actuations are: `a, delta`(throttle and steering angle). It should be noted that we define error states as: `cte = y - y_des` and `epsi = psi-psi_des`, which are not the same as the one provided. Also, we flip the sign of the steering angle `double steer_value = -vars[0]/0.436332;` at the time sending to the simulator so that the original equations can be used.

### Timestep Length and Elapsed Duration (N & dt)
First we set `dt = 0.1` since we want to make it simpler for adapting to the actuation latency. We also pick `N=8` to balance the efficiency and accuracy. We mainly tried different Ns with the fixed dt: `N = 5,8,10,15,25` and found that a large N helps to predict the future better while taking more computation power to for solving the optimal solutions. Finally, we think `N=8` is a better choice among these numbers.

### Polynomial Fitting and MPC Preprocessing
We perform the preprocessing of the waypoints using the coordinate transformation:
```c++
//transfrom from global to car referenced frame
vector<double> ptsx_car;
vector<double> ptsy_car;
for (int i = 0; i < ptsx.size(); i++) {
  ptsx_car.push_back( (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi));
  ptsy_car.push_back( -(ptsx[i] - px) * sin(psi) + (ptsy[i] - py) * cos(psi));
}
```
where `px, py` are the car coordinate and `ptsx, ptsy` are transformed be taking the car position and orientation as the new referenced frame. This can help to provide data points for rendering on the simulator and, more importantly, can help to avoid some singularity issues using global coordinate systems (high coefficients of polynomials for some orientations).  

### Model Predictive Control with Latency
This is the most challenging part, so in order to handle latency the adaptions are developed in two folds:
#### 1. Modification of the model:
With latency of 100ms, the actuations are not gonna affect the next time step but the one after. Therefore, we have the following changes of `FG_eval`:
```c++
void operator()(ADvector& fg, const ADvector& vars) {
  ...
  for (int t = 1; t < N; t++) {
    ...
    // Conseder the delays.
    AD<double> delta0 =  actuations[0];
    AD<double> a0 = actuations[1];
    AD<double> delta0;
    AD<double> a0;
    if (t > 1){
      delta0 = vars[delta_start + t - 2];
      a0 = vars[a_start + t - 2];
    }
    else
    {
      delta0 =  actuations[0];
      a0 = actuations[1];
    }
  }
}
```
where `actuations` are the actuation signals at t = -1 stored in `mpc` object. When updating `fg(t = 1)` the code is using `actuations`, which come from the time two step ahead; for `t > 1` the code is then using `vars[t - 2]`. These are all done based on passing `actuations` from the previously executed solutions:
```c++
MPC::MPC() {
  this->actuations.push_back(0.0);
  this->actuations.push_back(0.0);
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
...
FG_eval fg_eval(coeffs,actuations);
...
actuations[0] = solution.x[delta_start];
actuations[1] = solution.x[a_start];
}
...
```
What has been implemented is exactly equivalent to what the reviewer suggests! I just did it with extra one data point inside the MPC solver:
```c++
psi = delta; // in coordinate now, so use steering angle to predict x and y
px = px + v*cos(psi)*latency;
py = py + v*sin(psi)*latency;
cte= cte + v*sin(epsi)*latency;
epsi = epsi + v*delta*latency/Lf;
psi = psi + v*delta*latency/Lf;
v = v + a*latency;
```
#### 2. Cost function tuning
We develop the following cost function (revised):
```c++
// The part of the cost based on the reference state.
for (int t = 0; t < N; t++) {
  fg[0] += 8000*CppAD::pow(vars[cte_start + t], 2);
  fg[0] += 1000*CppAD::pow(vars[epsi_start + t], 2);
  fg[0] += 30*CppAD::pow(vars[v_start + t] - ref_v, 2);
}
// Minimize the use of actuators.
for (int t = 0; t < N - 1; t++) {
  fg[0] += 6000 * CppAD::pow(vars[v_start + t], 2) * CppAD::pow(vars[delta_start + t], 2);
  fg[0] += CppAD::pow(vars[a_start + t], 2);
}
// Minimize the value gap between sequential actuations.
for (int t = 0; t < N - 2; t++) {
  fg[0] += 80*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```

It should be emphasized that the weighted cost of `fg[0] += 500*CppAD::pow(vars[v_start + t], 2)*CppAD::pow(vars[delta_start + t], 2);` is particularity useful (thanks to the discussions in the Udacity forum). This is because whenever the car is in a high speed, the steering angle should always be limited so as to limit the deviation of the model dynamical terms: `v0*delta0`. Other parameters then become less sensitive to the tuning after the introduction of this cost term. The tests on the simulator seem pretty good with the speed reference of 50mph.
