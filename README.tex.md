# Convex Optimal Robot Drive Selection (CORDS)

CORDS is a tool for optimal selection of motors and gearboxes for robotic applications. CORDS runs a brute force optimization over motor and gearbox combinations specified in a [motor/gearbox data base folder](database). 

The database includes this/that and is always getting bigger and we encourage users to add to it (add links), Likely want this to be its own repo hosted in the database folder. 
* Maxon
* Faulhaber 
* Tmotor 
* Allied Motion 

CORDS requires the user to specify a trajectory of robot joint velocities $\omega$ and to parametrize the desired joint torques through an optimization problem. For simple cases where both the joint trajectory and the joint torques are known, check out the CORDS web interface (A LINK). 


## What can CORDS solve?  

CORDS can solve any motor/gearbox selection that can be cast as the problem type below. This can include optimizing much more than motors/gearboxes such as battery mass, parallel springs, or robot size. We reccomend that users utilize the code provided in [examples](examples) to get started using CORDS. These example problems include:
 * [optimizing parallel elasticity for minimal power consumption](examples/example1.m)
 * [optimizing battery mass to maximize run time for a quadruped robot](examples/example2.m)
 * [optimizing exoskeleton design parameters]

Specifically CORDS solves 
$$
\begin{align*}
\underset{x, I}{\text{minimize}} \quad  I^T Q_0 I  + c_0^T I + x^T M_0 x &+ r_0^T x + \beta_0      \\
   \text{subject to} \quad \qquad \qquad \qquad \qquad \qquad &                  \\
      \text{motor/gearbox output torque}  &=   T x + \tau_c                      \\
 \end{align*}
 $$ 
 with optional contraints 
 $$ 
 \begin{align*}
            I^T Q_j I + c_j^T I +  x^T M_j x + r_j^T x + \beta_j &\leq 0,\quad \text{for}\;j = 1...m \\
           G_{eq} x + h_{eq} &= 0                                                            \\
           G_{ineq} x + h_{ineq} &\preceq 0                                                  \\
                x_{lb} \preceq x &\preceq x_{ub}                                             \\
\end{align*}
$$

with motor currents $I \in \mathbf{R}^n$ and auxiliary varaibles $x \in \mathbf{R}^w$ where the inputs satsify: 
* $Q_j \in \mathbf{R}^{n \times n}$, diagonal PSD matrix for $j = 0...m$
* $M_0 \in \mathbf{R}^{w \times w}$, symmetric PSD
* $M_j \in \mathbf{R}^{w \times w}$, for $j=1...m$ symmetric PSD or encodes SOC constraint (see cords.update_problem)
* $c_j \in \mathbf{R}^n$ satisfies $c_{j}^{i} \omega^i \geq 0$ for $j = 1...m$ and $i = 1...n$

CORDS can also solve linear fractional programs where the minimization objective is replaced with $\left(r_{\text{num}}^T x + \beta_{\text{num}}\right)/\left(r_{\text{den}}^T x + \beta_{\text{den}}\right)$. 


## Problem Interfaces
We provide interfaces to simplify using CORDS for common optimization objectives including:
* [minimum power consumption](/src/interfaces/min_power_consumption.m)
* [minimum mass](/src/interfaces/min_mass.m)
* [minimum effective inertia](/src/interfaces/min_effective_inertia.m)
* [minimum peak torque](/src/interfaces/min_peak_torque.m)


## A Simple Example - Minimizing Joule Heating 
First we build a structure of problem data to pass to the CORDS optimizer. Dependence on motor/gearbox properties can be accomplished using anonymous functions (eg. ``total_mass = @(motor, gearbox) motor.mass + gearbox.mass``. For a list of valid ``motor`` and ``gearbox`` properties see the FFF documentation. Minimizing joule heating means minimizing $\sum_{i = 1}^{n} R (I^{i})^2$ where $R$ is the motor resistance and $I^{i}$ is the current at timestep $i$. 
```
>> load('example_data.mat', 'theta', 'omega', 'omega_dot', 'tau_des');   % load in data
>> data = struct();        % empty struct with no fields that we will fill in 
>> data.omega = omega;
>> data.omega_dot = omega; 
>> data.Q0 = @(motor, gearbox) motor.R * ones(length(omega), 1);   % specify the DIAGONAL of Q0
>> data.c0 = [];
>> data.M0 = [];
>> data.r0 = [];
>> data.I_max = 80;     % set 80 Amp current limit
>> data.V_max = 24;     % set 24 Volt voltage limit 
>> data.tau_c = tau_des;
>> data.T = [];         % simple case, no x variable 
```
Adding a parallel elastic element with stiffness $k_p$, the torque equality becomes
$$
     \text{motor/gearbox output torque}  = - k_p \theta  + \tau_{des} 
$$

letting the optimation vector $x$ encode the parallel stiffness $k_p$ we can rewrite the last line above as:
```
>> data.T = [-theta];      % include coupling of motor torque and spring torque
```
We now pass this data to the CORDS optimizer
```
>> prob = cords();                  % create a new cords object with default settings  
>> prob.update_problem(data);       % attach the data to the problem
>> solutions = prob.optimize(10);   % get the 10 best motor/gearbox combinations 
```


## Requirements
* Matlab 20something or later, ill figure it out 
* [Gurobi 8.0 or later](https://www.gurobi.com/academia/academic-program-and-licenses/) or [ECOS](https://github.com/embotech/ecos)
Gurobi offers better performance than the ECOS solver but is only free for academic use. 
* A database (explain)

## Installation

clone or download   TODO ( test on another laptop) 
add to matlab path  TODO 
```
git clone something 
```
add the files
```
something something
```
## Documentation 

Detailed documentation for CORDS can be acessed by typing
```
>> doc cords
```
or 
```
>> help cords
```
at the Matlab command prompt. 




Using CORDS in your research? Cite our not yet extant paper: 
```

formated bib.tex


```
