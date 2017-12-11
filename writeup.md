##Writeup
In this writeup, I have discussed the project implementation items covered inside rubric.

### The Model
In this project I have use the kinematic project taught in lessons. The implementation can be found in lines 55 to 138 of MPC.cpp. Snapshots here:

#### Cost calculation
The cost of deviating from reference is assigned to the function with weights for each respective parameter.

```c++
//***************************
// Cost calculation
//***************************
// The part of the cost based on the reference state.
for (int t = 0; t < N; t++) {
	fg[0] += 2000 * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
	fg[0] += 2000 * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
	fg[0] += 1 * CppAD::pow(vars[v_start + t] - ref_v, 2);
}

// Minimize the use of actuators.
for (int t = 0; t < N - 1; t++) {
	fg[0] += 5 * CppAD::pow(vars[delta_start + t], 2);
	fg[0] += 5 * CppAD::pow(vars[a_start + t], 2);
	fg[0] += 700*CppAD::pow(vars[delta_start+t]*vars[v_start+t],2);
}

// Minimize the value gap between sequential actuations.
for (int t = 0; t < N - 2; t++) {
	fg[0] += 200 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
	fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```

#### Kinematic Model
The kinematic model equations implemented here:
```c++
fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[2 + psi_start + t] = psi1 - (psi0 - (v0 * delta0 / Lf) * dt);
fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
fg[2 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[2 + epsi_start + t] = epsi1 - ((psi0 - psides0) - (v0 * delta0 / Lf) * dt);
```

### Timestep Length and Elapsed Duration
Assigned weights to each parameter of cost function depending upon their priority and then found the timestep length (dt) and elapsed duration (N*dt) compatible with the weights. 

```c++
size_t N = 10;
double dt = 0.1;
```

The dt values of 0.1, 0.2 and 0.3 with same number of timesteps used for calculations and it was concluded that timestep values greater than 0.1 is too long interval between two control outputs and make the car go off-track. 
A larger value of elapsed duration or a smaller interval time would mean unnecessary computation overhead so this was avoided.

### Polynomial Fitting and MPC Preprocessing
The waypoints were transformed to the vehicle coordinate system and then polynomial was fitted to the transformed x and y waypoints. Code in main.cpp lines 112 to 129.

```c++
for (int i=0; i<ptsx.size(); i++)
{
	//shift car reference angle to 90 degrees
	double shift_x = ptsx[i]-delayed_px;
	double shift_y = ptsy[i]-delayed_py;

	ptsx[i] = (shift_x * cos(0-delayed_psi) - shift_y * sin(0-delayed_psi));
	ptsy[i] = (shift_x * sin(0-delayed_psi) + shift_y * cos(0-delayed_psi));
}

double* ptrx = &ptsx[0];
double* ptry = &ptsy[0];

//convert to VectorXd
Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
```


### Latency Considerations
The latency was tackled by two considerations:

1. The delayed state was calculated for polynomial fit in main.cpp:

```c++
double delayed_px = px + v * cos(psi) * delay/1000;
double delayed_py = py + v * sin(psi) * delay/1000;
double delayed_psi = psi + (v * tan(-steer_value) / Lf) * delay/1000 + ( (throttle_value * 5 * tan(-steer_value) / (2*Lf)) * pow(delay/1000,2));
double delayed_v = v + throttle_value * 5 * delay/1000;
```

2. The delayed actuation was used as input for model equations:
```c++
// actuation at time t-1 for latency consideration
if(t > 0){
	a0 = vars[a_start + t - 1];
	delta0 = vars[delta_start + t - 1];
}
```