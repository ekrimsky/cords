# Convex Optimal Robot Drive Selection (CORDS)

CORDS is a tool for optimal selection of motors and gearboxes for robotic applications. CORDS runs a brute force optimization (phrasing) over motor and gearboxes specified in a [motor/gearbox data base folder](database). 

The database includes this/that and is always getting bigger and we encourage users to add to it (add links), Likely want this to be its own repo hosted in the database folder. 
* Maxon
* Faulhaber 
* Tmotor 
* Allied Motion 

CORDS requires the user to specify a trajectory of robot joint velocities $\omega$ and to parametrize the desired joint torques through an optimization problem (rephrase). For simple cases where both the joint trajectory and the joint torques are known, check out the CORDS web interface (A LINK). 



## What can CORDS solve?  

a human explanation before math 

interface to second order cone pr

$$
\begin{align*}
   \text{minimize} \quad & I^T Q_0 I + c_0 I + x^T M_0 x + r_0 x + \beta_0      \\
   \text{subject to}\quad &                                                \\
      \text{motor/gearbox output torque}  &=   T x + \tau_c            \\
            I^T Q_j I + c_j^T I +  x^T M_j x + r_j^T x + \beta_j &\leq 0,\quad \text{for}\;j = 1\ldotsm \\
           G_{eq} x + h_{eq} &= 0                                              \\
           G_{ineq} &x + h_{ineq} &\preceq 0                              \\
       x_{lb} &\preceq x &\preceq x_{ub}                         \\
\end{align*}
$$

where 

also linear fractional programs too where the minimization objective is replaced with $\frac{r_{\text{num}}^T x + b_{\text{num}}}{r_{\text{den}}^T x + b_{\text{den}}}$. 




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
