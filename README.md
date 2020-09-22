# Convex Optimal Robot Drive Selection (CORDS)

CORDS is a tool for optimal selection of motors and gearboxes for robotic applications. CORDS runs a brute force optimization (phrasing) over motor and gearboxes specified in a [motor/gearbox data base folder](database). 

The database includes this/that and is always getting bigger and we encourage users to add to it (add links), Likely want this to be its own repo hosted in the database folder. 
* Maxon
* Faulhaber 
* Tmotor 
* Allied Motion 

CORDS requires the user to specify a trajectory of robot joint velocities <img src="/tex/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode&sanitize=true" align=middle width=10.82192594999999pt height=14.15524440000002pt/> and to parametrize the desired joint torques through an optimization problem (rephrase). For simple cases where both the joint trajectory and the joint torques are known, check out the CORDS web interface (A LINK). 



## What can CORDS solve?  

a human explanation before math 

interface to second order cone pr

<p align="center"><img src="/tex/1bf1572803d4ac765d162fd8ab2eb758.svg?invert_in_darkmode&sanitize=true" align=middle width=551.53841655pt height=169.57442534999998pt/></p>

where 

also linear fractional programs too where the minimization objective is replaced with <img src="/tex/19ffd9b9832df33f02b2d35e752c83c9.svg?invert_in_darkmode&sanitize=true" align=middle width=73.1978412pt height=37.92139230000001pt/>. 




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

## A simple example - minimizing joule heating 
```
>> code 
>> code 
>> and more code 
>> and even more code
```
soething something 
math on math on math 
```
>> code 
>> code 
>> and more code 
>> and even more code
```


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
