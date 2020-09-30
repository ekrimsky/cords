# Convex Optimal Robot Drive Selection (CORDS)

CORDS is a tool for optimal selection of motors and gearboxes for robotic applications. CORDS runs a brute force optimization over tens of thousands motor and gearbox combinations specified in a [motor/gearbox database folder](https://github.com/ekrimsk/MGDB/). 

CORDS requires the user to specify a trajectory of robot joint velocities <img src="/tex/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode&sanitize=true" align=middle width=10.82192594999999pt height=14.15524440000002pt/> and to parametrize the desired joint torques through an optimization problem. For simple cases where both the joint trajectory *and the joint torques* are known, check out the CORDS web interface (A LINK - keenon make this website plz). 


## What can CORDS solve?  

CORDS can solve any motor/gearbox selection problem that can be cast as the problem type below. This can include optimizing other robot features such as battery mass or parallel springs in conjunction with choosing the optimal motor and gearbox. We reccomend that users utilize the code provided in [examples](examples) to get started using CORDS. These example problems include:
 * [optimizing parallel elasticity for minimal power consumption](examples/parallel_elastic.m)
 * [optimizing battery mass to maximize run time for a quadruped robot](examples/quadruped.m)
 * [optimizing exoskeleton design parameters](https://www.youtube.com/watch?v=dQw4w9WgXcQ&ab_channel=RickAstleyVEVO)
 
 Examples should be run from the root CORDS directory. 

CORDS solves 
<p align="center"><img src="/tex/f841dd325a496627b7bb54f754e30fc5.svg?invert_in_darkmode&sanitize=true" align=middle width=338.20401119999997pt height=78.28491164999998pt/></p> 
 with optional contraints 
 <p align="center"><img src="/tex/09e274f44f167870ce28a0f8c59e80cf.svg?invert_in_darkmode&sanitize=true" align=middle width=200.89093035pt height=64.10956365pt/></p>

with motor currents <img src="/tex/5164633986a5c3e175fc7baf049cfb67.svg?invert_in_darkmode&sanitize=true" align=middle width=48.05839499999998pt height=22.55708729999998pt/>, motor currents *squared* <img src="/tex/984ff922824d8d0e22eed2ab762a7e8a.svg?invert_in_darkmode&sanitize=true" align=middle width=61.522490699999985pt height=22.55708729999998pt/>, and auxiliary variables <img src="/tex/f948115fd1b4556bfe21e59235430b51.svg?invert_in_darkmode&sanitize=true" align=middle width=53.483454749999986pt height=22.55708729999998pt/> where the problem inputs satsify: 
* <img src="/tex/51c48b7f3ccdeb9df714a595fd3b909a.svg?invert_in_darkmode&sanitize=true" align=middle width=74.86676339999998pt height=28.894955100000008pt/>, vector of non-negative coefficients 
* <img src="/tex/c29e2f29cc506a63f00fd5e5460a20b3.svg?invert_in_darkmode&sanitize=true" align=middle width=56.88343154999998pt height=22.55708729999998pt/> satisfies <img src="/tex/36b3758fe61b44487e6516f9d2e71001.svg?invert_in_darkmode&sanitize=true" align=middle width=60.47175089999999pt height=27.15900329999998pt/> for <img src="/tex/be4dba4d5583203f4f9d3bee918a2c9e.svg?invert_in_darkmode&sanitize=true" align=middle width=59.365615649999995pt height=21.68300969999999pt/>
* <img src="/tex/1e47cf05617b340cea169d5c16925949.svg?invert_in_darkmode&sanitize=true" align=middle width=87.50376194999998pt height=26.17730939999998pt/>, symmetric PSD
* <img src="/tex/f1ac71df28ea1db910391f7fe3887b61.svg?invert_in_darkmode&sanitize=true" align=middle width=77.17081184999999pt height=28.310511900000005pt/>, matrix of non-negative coefficients 
* <img src="/tex/61f2fabd3245f89dfe5df5c62516e15e.svg?invert_in_darkmode&sanitize=true" align=middle width=77.258676pt height=26.17730939999998pt/> satisfies <img src="/tex/36b3758fe61b44487e6516f9d2e71001.svg?invert_in_darkmode&sanitize=true" align=middle width=60.47175089999999pt height=27.15900329999998pt/> for each row of C (<img src="/tex/4884e50c3bb19744ca6785efc67fe03c.svg?invert_in_darkmode&sanitize=true" align=middle width=65.97903014999999pt height=21.68300969999999pt/>) and <img src="/tex/be4dba4d5583203f4f9d3bee918a2c9e.svg?invert_in_darkmode&sanitize=true" align=middle width=59.365615649999995pt height=21.68300969999999pt/>
Quadratic and Second Order Cone constraints can also be applied on the optimization variable (<img src="/tex/332cc365a4987aacce0ead01b8bdcc0b.svg?invert_in_darkmode&sanitize=true" align=middle width=9.39498779999999pt height=14.15524440000002pt/>). 

CORDS can also solve linear fractional programs where the minimization objective is replaced with <img src="/tex/e6956383372cf6126c6476af40778928.svg?invert_in_darkmode&sanitize=true" align=middle width=231.45005565pt height=27.94539330000001pt/>. 


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
