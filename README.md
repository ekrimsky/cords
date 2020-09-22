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
<p align="center"><img src="/tex/72128932f32cecf4ff94d0aa0deb66dc.svg?invert_in_darkmode&sanitize=true" align=middle width=324.90884415pt height=68.07495089999999pt/></p> 
 with optional contraints 
 <p align="center"><img src="/tex/aa4a797a4bf8aa49a938ae56eeda2eb4.svg?invert_in_darkmode&sanitize=true" align=middle width=365.50158809999994pt height=93.11585249999999pt/></p>

where 

also linear fractional programs too where the minimization objective is replaced with <img src="/tex/19ffd9b9832df33f02b2d35e752c83c9.svg?invert_in_darkmode&sanitize=true" align=middle width=73.1978412pt height=37.92139230000001pt/>. 


## A simple example - minimizing joule heating 
First we build a structure of problem data to pass to the CORDS optimizer. Dependence on motor/gearbox properties can be accomplished using anonymous functions (eg. ``total_mass = @(motor, gearbox) motor.mass + gearbox.mass``. For a list of valid ``motor`` and ``gearbox`` properties see the FFF documentation. Minimizing joule heating means minimizing <img src="/tex/7c727e6e17d47b11d7d6e1155ea7e099.svg?invert_in_darkmode&sanitize=true" align=middle width=88.14323099999999pt height=27.15900329999998pt/> where <img src="/tex/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode&sanitize=true" align=middle width=12.60847334999999pt height=22.465723500000017pt/> is the motor resistance and <img src="/tex/8294c58cadf040e3716a7f6bc748cdde.svg?invert_in_darkmode&sanitize=true" align=middle width=13.16686469999999pt height=27.15900329999998pt/> is the current at timestep <img src="/tex/77a3b857d53fb44e33b53e4c8b68351a.svg?invert_in_darkmode&sanitize=true" align=middle width=5.663225699999989pt height=21.68300969999999pt/>. 
```
>> load('example_data.mat', 'theta', 'omega', 'omega_dot', 'tau_des');   % load in data
>> data.omega = omega;
>> data.omega_dot = omega; 
>> data.Q0 = @(motor, gearbox) motor.R * ones(length(omega), 1);   % specify the DIAGONAL of Q0
>> data.c0 = [];
>> data.M0 = [];
>> data.r0 = [];
>> data.T = [];      % simple case, no x variable 
>> data.tau_c = tau_des;
```
We now pass this data to the CORDS optimizer
```
>> prob = cords();   % create a new cords object with default settings  
>> prob.update_problem(data);    % attach the data to the problem
>> solutions = prob.optimize(10);   % get the 10 best motor/gearbox combinations 
```
## adding optimal parallel elasticity
If we add a parallel elastic element with stiffness <img src="/tex/b19efe18c84e5887c52c1c0fd15160eb.svg?invert_in_darkmode&sanitize=true" align=middle width=15.33435419999999pt height=22.831056599999986pt/> our torque equality becomes
<p align="center"><img src="/tex/9cd212a7e6bf780596bd55687c31999a.svg?invert_in_darkmode&sanitize=true" align=middle width=323.4032736pt height=17.031940199999998pt/></p>
letting the optimation vector <img src="/tex/332cc365a4987aacce0ead01b8bdcc0b.svg?invert_in_darkmode&sanitize=true" align=middle width=9.39498779999999pt height=14.15524440000002pt/> encode <img src="/tex/b19efe18c84e5887c52c1c0fd15160eb.svg?invert_in_darkmode&sanitize=true" align=middle width=15.33435419999999pt height=22.831056599999986pt/> 
```
>> data.T = [-1];      % include coupling of torque  
>> data.tau_c = tau_des; 
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
