# Convex Optimal Robot Drive Selection (CORDS)

CORDS is a tool for optimal selection of motors and gearboxes for robotic applications. CORDS runs a brute force optimization over tens of thousands motor and gearbox combinations specified in a [motor/gearbox database folder](https://github.com/ekrimsk/MGDB/). 

CORDS requires the user to specify a trajectory of robot joint velocities $\omega$ and to parametrize the desired joint torques through an optimization problem. For simple cases where both the joint trajectory *and the joint torques* are known, check out the CORDS web interface (A LINK - keenon make this website plz). 


## What can CORDS solve?  

CORDS can solve any motor/gearbox selection problem that can be cast as the problem type below. This can include optimizing other robot features such as battery mass or parallel springs in conjunction with choosing the optimal motor and gearbox. We reccomend that users utilize the code provided in [examples](examples) to get started using CORDS. These example problems include:
 * [optimizing parallel elasticity for minimal power consumption](examples/parallel_elastic.m)
 * [optimizing battery mass to maximize run time for a quadruped robot](examples/quadruped.m)
 * [optimizing exoskeleton design parameters](https://www.youtube.com/watch?v=dQw4w9WgXcQ&ab_channel=RickAstleyVEVO)
 
 Examples should be run from the root CORDS directory. 

CORDS solves 
$$
\begin{align*}
\underset{x, I}{\text{minimize}} \quad   p_0^T i_{sq}  + c_0^T i + x^T M_0 x &+ f_0^T x + \beta_0      \\
   \text{subject to} \quad \qquad \qquad \qquad \qquad \qquad &                  \\
      \text{motor/gearbox output torque}  &=   T x + \tau_c                      \\
 \end{align*}
 $$ 
 with optional contraints 
 $$ 
 \begin{align*}
            P (i_{sq}) + C i  +  F x + \beta &\preceq 0  \\
                      G x + h &= 0             \\
                x_{lb} \preceq x &\preceq x_{ub}         \\
\end{align*}
$$

with motor currents $i \in \mathbf{R}^n$, motor currents *squared* $i_{sq} \in \mathbf{R}^n$, and auxiliary variables $x \in \mathbf{R}^w$ where the problem inputs satsify: 
* $p_0 \in \mathbf{R}_{+}^{n \times 1}$, vector of non-negative coefficients 
* $c_0 \in \mathbf{R}^n$ satisfies $c_{j}^{i} \omega^i \geq 0$ for $i = 1...n$
* $M_0 \in \mathbf{R}^{w \times w}$, symmetric PSD
* $P \in \mathbf{R}_{+}^{n \times m}$, matrix of non-negative coefficients 
* $C \in \mathbf{R}^{n \times m}$ satisfies $c_{j}^{i} \omega^i \geq 0$ for each row of C ($j = 1...m$) and $i = 1...n$
Quadratic and Second Order Cone constraints can also be applied on the optimization variable ($x$). 

CORDS can also solve linear fractional programs where the minimization objective is replaced with $\left(f_{\text{num}}^T x + \beta_{\text{num}}\right)/\left(f_{\text{den}}^T x + \beta_{\text{den}}\right)$. 


## Problem Interfaces
We provide interfaces to simplify using CORDS for common optimization objectives including:
* [minimum power consumption](/src/interfaces/min_power_consumption.m)
* [minimum mass](/src/interfaces/min_mass.m)
* [minimum effective inertia](/src/interfaces/min_effective_inertia.m)



## A Simple Example - Minimizing Joule Heating 
First we build a structure of problem data to pass to the CORDS optimizer. Dependence on motor/gearbox properties can be accomplished using anonymous functions (eg. ``total_mass = @(motor, gearbox) motor.mass + gearbox.mass``. For a list of valid ``motor`` and ``gearbox`` properties see the FFF documentation. Minimizing joule heating means minimizing $\sum_{i = 1}^{n} R (I^{i})^2$ where $R$ is the motor resistance and $I^{i}$ is the current at timestep $i$. 
```
>> load('example_data.mat', 'theta', 'omega', 'omega_dot', 'tau_des');   % load in data
>> data = struct();        % empty struct with no fields that we will fill in 
>> data.omega = omega;
>> data.omega_dot = omega_dot; 
>> data.p0 = @(motor, gearbox) motor.R * ones(length(omega), 1);   % specify the DIAGONAL of Q0
>> data.c0 = [];        % all other cost terms can be left empty
>> data.M0 = [];
>> data.f0 = [];
>> data.beta0 = []; 
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
>> data.T = [-theta(:)];      % include coupling of motor torque and spring torque
```
We now pass this data to the CORDS optimizer
```
>> prob = cords();                  % create a new cords object with default settings  
>> prob.update_problem(data);       % attach the data to the problem
>> solutions = prob.optimize(10);   % get the 10 best motor/gearbox combinations 
```


## Requirements
* Matlab 2017b or later but I dont really know, delaey does it work?
* [Gurobi 8.1 or later](https://www.gurobi.com/academia/academic-program-and-licenses/) or [ECOS](https://github.com/embotech/ecos)

Gurobi usually offers better performance than the ECOS solver but is only free for academic use. 

## Installation

Install CORDS by cloning the repository to your Matlab directory
```
git clone --recurse-submodules https://github.com/ekrimsk/CORDS.git
```
where adding `--recurse-submodules` will clone the motor/gearbox database as well. Alternatively, you can download and extract the zip and seperately download the [motor database](https://github.com/ekrimsk/MGDB/) and move it to `CORDS/MGDB` on your machine. 

CORDS needs to be added to the Matlab path. This can be done with `addpath(genpath(path_to_cords/CORDS))`. Running `savepath` will then add CORDS to your default Matlab path. On linux systems you may need to add `addpath path_to_cords/CORDS` to you `startup.m` file. 

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




Using CORDS in your research? Cite our not yet extant paper if it ever gets written and published: 
```

formated bib.tex


```
