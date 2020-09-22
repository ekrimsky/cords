# Convex Optimal Robot Drive Selection (CORDS)

test text 

some test math 


$$
\begin{align*}
   \text{minimize} \qquad &I' Q0 I + c0 I + x' M_0 x + r0 x + \beta_0      \\
   \text{subject to}                                                \\
         T x + \tau_c  & = \text{motor/gearbox output torque}           \\
            x_{lb} &\preceq x \preceq x_{ub}                         \\
           G_{eq} x &+ h_{eq} = 0                                              \\
           G_{ineq} &x + h_{ineq} \preceq 0                              \\
     I^T Q_j I + c_j^T I + & x^T M_j x + r_j^T x + \beta_j \leq 0,\quad \text{for} j = 1...m \\
\end{align*}
$$



## What is CORDS? 

## What can we solve 

## Requirements

* Matlab 20something or later
* [Gurobi](https://www.gurobi.com/academia/academic-program-and-licenses/) or ECOS(https://github.com/embotech/ecos)


## Installation

some explanation of how to install 


## Documentation 

some pdf and also embedded in matlab too 



math on math on math 

some sweet gifs 

## Problem Interfaces  (rename)
* [minimum power consumption](min_power_consumption.m)
* [minimum mass](min_mass.m)
* [minimum effective inertia](min_effective_inertia.m)
* [minimum peak torque](min_peak_torque.m)



Using CORDS in your research cite our not yet extant paper: 
```

formated bib.tex


```