# Convex Optimal Robot Drive Selection (CORDS)

CORDS is a tool for optimal selection of motors and gearboxes for robotic applications. CORDS runs a brute force optimization over tens of thousands motor and gearbox combinations specified in a [motor/gearbox data base folder](https://github.com/ekrimsk/MGDB/). 

CORDS requires the user to specify a trajectory of robot joint velocities <img src="/tex/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode&sanitize=true" align=middle width=10.82192594999999pt height=14.15524440000002pt/> and to parametrize the desired joint torques through an optimization problem. For simple cases where both the joint trajectory *and the joint torques* are known, check out the CORDS web interface (A LINK - keenon make this website plz). 


## What can CORDS solve?  

CORDS can solve any motor/gearbox selection problem that can be cast as the problem type below. This can include optimizing other robot features such as battery mass or parallel springs in conjunction with choosing the optimal motor and gearbox. We reccomend that users utilize the code provided in [examples](examples) to get started using CORDS. These example problems include:
 * [optimizing parallel elasticity for minimal power consumption](examples/parallel_elastic.m)
 * [optimizing battery mass to maximize run time for a quadruped robot](examples/quadruped.m)
 * [optimizing exoskeleton design parameters]

Specifically CORDS solves 
<p align="center"><img src="/tex/8e524d848db90a42ba21abfbb9832cfe.svg?invert_in_darkmode&sanitize=true" align=middle width=336.2595522pt height=78.28491164999998pt/></p> 
 with optional contraints 
 <p align="center"><img src="/tex/6724982e0f57646145cfc277f79bb4d1.svg?invert_in_darkmode&sanitize=true" align=middle width=393.63336045pt height=93.11585249999999pt/></p>

with motor currents <img src="/tex/cd427d5ecb99fcabde5a7fffa6103a1f.svg?invert_in_darkmode&sanitize=true" align=middle width=50.911134449999984pt height=22.55708729999998pt/> and auxiliary variables <img src="/tex/f948115fd1b4556bfe21e59235430b51.svg?invert_in_darkmode&sanitize=true" align=middle width=53.483454749999986pt height=22.55708729999998pt/> where the problem inputs satsify: 
* <img src="/tex/bad8cbb55822df2eb7bbf59df6190e30.svg?invert_in_darkmode&sanitize=true" align=middle width=80.71703969999999pt height=26.17730939999998pt/>, diagonal PSD matrix for <img src="/tex/52c03ebb6ac0c8e7f1261d96409b7cbc.svg?invert_in_darkmode&sanitize=true" align=middle width=65.97903014999999pt height=21.68300969999999pt/>
* <img src="/tex/1e47cf05617b340cea169d5c16925949.svg?invert_in_darkmode&sanitize=true" align=middle width=87.50376194999998pt height=26.17730939999998pt/>, symmetric PSD
* <img src="/tex/6f41c62b5c2794bfa9a2bef8f734c971.svg?invert_in_darkmode&sanitize=true" align=middle width=87.05570445pt height=26.17730939999998pt/>, for <img src="/tex/54f5851e55c7944fd243ff3e83828b82.svg?invert_in_darkmode&sanitize=true" align=middle width=65.97903014999999pt height=21.68300969999999pt/> symmetric PSD or encodes SOC constraint (see cords.update_problem)
* <img src="/tex/7da0268045d75f2db625838ec284e453.svg?invert_in_darkmode&sanitize=true" align=middle width=56.43537569999998pt height=22.55708729999998pt/> satisfies <img src="/tex/36b3758fe61b44487e6516f9d2e71001.svg?invert_in_darkmode&sanitize=true" align=middle width=60.47175089999999pt height=27.15900329999998pt/> for <img src="/tex/4884e50c3bb19744ca6785efc67fe03c.svg?invert_in_darkmode&sanitize=true" align=middle width=65.97903014999999pt height=21.68300969999999pt/> and <img src="/tex/be4dba4d5583203f4f9d3bee918a2c9e.svg?invert_in_darkmode&sanitize=true" align=middle width=59.365615649999995pt height=21.68300969999999pt/>

CORDS can also solve linear fractional programs where the minimization objective is replaced with <img src="/tex/b923d55946fe7624d6802fc5c79a0dda.svg?invert_in_darkmode&sanitize=true" align=middle width=230.18670014999998pt height=27.94539330000001pt/>. 


## Problem Interfaces
We provide interfaces to simplify using CORDS for common optimization objectives including:
* [minimum power consumption](/src/interfaces/min_power_consumption.m)
* [minimum mass](/src/interfaces/min_mass.m)
* [minimum effective inertia](/src/interfaces/min_effective_inertia.m)



## A Simple Example - Minimizing Joule Heating 
First we build a structure of problem data to pass to the CORDS optimizer. Dependence on motor/gearbox properties can be accomplished using anonymous functions (eg. ``total_mass = @(motor, gearbox) motor.mass + gearbox.mass``. For a list of valid ``motor`` and ``gearbox`` properties see the FFF documentation. Minimizing joule heating means minimizing <img src="/tex/7c727e6e17d47b11d7d6e1155ea7e099.svg?invert_in_darkmode&sanitize=true" align=middle width=88.14323099999999pt height=27.15900329999998pt/> where <img src="/tex/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode&sanitize=true" align=middle width=12.60847334999999pt height=22.465723500000017pt/> is the motor resistance and <img src="/tex/8294c58cadf040e3716a7f6bc748cdde.svg?invert_in_darkmode&sanitize=true" align=middle width=13.16686469999999pt height=27.15900329999998pt/> is the current at timestep <img src="/tex/77a3b857d53fb44e33b53e4c8b68351a.svg?invert_in_darkmode&sanitize=true" align=middle width=5.663225699999989pt height=21.68300969999999pt/>. 
```
>> load('example_data.mat', 'theta', 'omega', 'omega_dot', 'tau_des');   % load in data
>> data = struct();        % empty struct with no fields that we will fill in 
>> data.omega = omega;
>> data.omega_dot = omega_dot; 
>> data.Q0 = @(motor, gearbox) motor.R * ones(length(omega), 1);   % specify the DIAGONAL of Q0
>> data.c0 = [];        % all other cost terms can be left empty
>> data.M0 = [];
>> data.r0 = [];
>> data.beta0 = []; 
>> data.I_max = 80;     % set 80 Amp current limit
>> data.V_max = 24;     % set 24 Volt voltage limit 
>> data.tau_c = tau_des;
>> data.T = [];         % simple case, no x variable 
```
Adding a parallel elastic element with stiffness <img src="/tex/b19efe18c84e5887c52c1c0fd15160eb.svg?invert_in_darkmode&sanitize=true" align=middle width=15.33435419999999pt height=22.831056599999986pt/>, the torque equality becomes
<p align="center"><img src="/tex/9f504a77ad210a66062f24124eb64a2a.svg?invert_in_darkmode&sanitize=true" align=middle width=318.51739589999994pt height=17.031940199999998pt/></p>

letting the optimation vector <img src="/tex/332cc365a4987aacce0ead01b8bdcc0b.svg?invert_in_darkmode&sanitize=true" align=middle width=9.39498779999999pt height=14.15524440000002pt/> encode the parallel stiffness <img src="/tex/b19efe18c84e5887c52c1c0fd15160eb.svg?invert_in_darkmode&sanitize=true" align=middle width=15.33435419999999pt height=22.831056599999986pt/> we can rewrite the last line above as:
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
* [Gurobi 8.0 or later](https://www.gurobi.com/academia/academic-program-and-licenses/) or [ECOS](https://github.com/embotech/ecos)

Gurobi offers better performance than the ECOS solver but is only free for academic use. 

## Installation

Install CORDS by cloning the repository to your Matlab directory
```
git clone https://github.com/ekrimsk/CORDS.git
```
or download the zip. 

CORDS needs to be added to the Matlab path. This can be done with `addpath(genpath(path_to_cords/cords))`. To add CORDS to the Matlab search path on startup, you can add `addpath path_to_cords/cors` to you `startup.m` file.

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
