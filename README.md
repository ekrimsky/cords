# Convex Optimal Robot Drive Selection (CORDS)

CORDS is a tool for optimal selection of motors and gearboxes for robotic applications. CORDS runs a brute force optimization over motor and gearbox combinations specified in a [motor/gearbox data base folder](database). 

The database includes this/that and is always getting bigger and we encourage users to add to it (add links), Likely want this to be its own repo hosted in the database folder. 
* Maxon
* Faulhaber 
* Tmotor 
* Allied Motion 

CORDS requires the user to specify a trajectory of robot joint velocities <img src="/tex/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode&sanitize=true" align=middle width=10.82192594999999pt height=14.15524440000002pt/> and to parametrize the desired joint torques through an optimization problem. For simple cases where both the joint trajectory and the joint torques are known, check out the CORDS web interface (A LINK). 


## What can CORDS solve?  

a human explanation before math 

interface to second order cone pr
split out optional constraints 
<p align="center"><img src="/tex/ef10577ea74a5e83fb58bb65d56cb8a0.svg?invert_in_darkmode&sanitize=true" align=middle width=445.39951664999995pt height=169.57442534999998pt/></p>

where 

also linear fractional programs too where the minimization objective is replaced with <img src="/tex/19ffd9b9832df33f02b2d35e752c83c9.svg?invert_in_darkmode&sanitize=true" align=middle width=73.1978412pt height=37.92139230000001pt/>. 


## A simple example - minimizing joule heating 
First we build a structure of problem data to pass to the CORDS optimizer. Dependence on motor/gearbox properties can be accomplished using anonymous functions (eg. ``total_mass = @(motor, gearbox) motor.mass + gearbox.mass``. For a list of valid ``motor`` and ``gearbox`` properties see the FFF documentation. 
```
>> load('example_data.mat', 'theta', 'omega', 'omega_dot', 'tau_des');   % load in data
>> data.omega = omega;
>> data.omega_dot = omega; 
>> data.Q0 = @(motor, gearbox) motor.R * ones(length(omega), 1);   % specify the DIAGONAL of Q0
>> data.c0 = [];
>> data.M0 = [];
>> data.r0 = [];
>> data.tau_c = tau_des;
>> data.T = [];      % simple case, no x variable 
```
We now pass this data to the CORDS optimizer
```
>> prob = cords();   % create a new cords object with default settings  
>> prob.update_problem(data);    % attach the data to the problem
>> solutions = prob.optimize(10);   % get the 10 best motor/gearbox combinations 
```
## A less simple example - minimizing joule heating with parallel elasticity
```
>> load('example_data.mat', 'theta', 'omega', 'omega_dot', 'tau_des');   % load in data
>> data.omega = omega;
>> data.omega_dot = omega; 
>> data.Q0 = @(motor, gearbox) motor.R * ones(length(omega), 1);   % specify the DIAGONAL of Q0
>> data.c0 = [];
>> data.M0 = [];
>> data.r0 = [];
>> data.tau_c = tau_des;
>> data.T = [];      % simple case, no x variable 
```



## Requirements

* Matlab 20something or later, ill figure it out 
* [Gurobi](https://www.gurobi.com/academia/academic-program-and-licenses/) or [ECOS](https://github.com/embotech/ecos)


## Installation

clone or download 
add to matlab path 

## Documentation 

Detailed documentation for CORDS can be acessed by tpying
```
>> doc cords
```
or 
```
>> help cords
```
at the Matlab command prompt. 



Also a PDF

some pdf and also embedded in matlab too 

some sweet gifs 

## Problem Interfaces
We also provide interfaces to simplify using CORDS for common optimization objective including:
* [minimum power consumption](/src/interfaces/min_power_consumption.m)
* [minimum mass](/src/interfaces/min_mass.m)
* [minimum effective inertia](/src/interfaces/min_effective_inertia.m)
* [minimum peak torque](/src/interfaces/min_peak_torque.m)



Using CORDS in your research cite our not yet extant paper: 
```

formated bib.tex


```
