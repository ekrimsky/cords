# Convex Optimal Robot Drive Selection (CORDS)

Explanation of what CORDS does/is 

check out our web interface for problems with blah blah 

## What can we solve 

$$
\begin{align*}
   \text{minimize} \quad & I^T Q_0 I + c_0 I + x^T M_0 x + r_0 x + \beta_0      \\
   \text{subject to}\quad &                                                \\
         T x + \tau_c  & = \text{motor/gearbox output torque}           \\
            I^T Q_j I + c_j^T I + & x^T M_j x + r_j^T x + \beta_j \leq 0,\quad \text{for}\;j = 1\ldotsm \\
           G_{eq} x &+ h_{eq} = 0                                              \\
           G_{ineq} &x + h_{ineq} \preceq 0                              \\
       x_{lb} &\preceq x \preceq x_{ub}                         \\
\end{align*}
$$

where 





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



math on math on math 

some sweet gifs 

## Problem Interfaces
We also provide interfaces to simplify using 
* [minimum power consumption](/src/interfaces/min_power_consumption.m)
* [minimum mass](/src/interfaces/min_mass.m)
* [minimum effective inertia](/src/interfaces/min_effective_inertia.m)
* [minimum peak torque](/src/interfaces/min_peak_torque.m)



Using CORDS in your research cite our not yet extant paper: 
```

formated bib.tex


```
