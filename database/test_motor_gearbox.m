% TO BE DELETED OR BETTER YET 
% INCORPORATED ELSEWHERE 
% BUT FOR NOW, THIS DEFINES A "TEST" MOTOR AND 
% GEARBOX FOR VALIDATING INPUTS 
close all; clear all; clc; 


test_motor.inertia = 1; 
test_motor.R = 1; 
test_motor.k_e = 0.1;
test_motor.k_t = 0.1;
test_motor.mass = 1; 
test_motor.R_wh = 1; 
test_motor.R_ha = 1; 
test_motor.I_nl = 1e-3; 
test_motor.omega_nl = 1; 
test_motor.I_nom = 100; 
test_motor.omega_max = 1000; 
test_motor.V_nom = 10; % Test voltage in data sheet 
test_motor.Temp_max = 100; % winding temperature 
test_motor.cost = 1000;
test_motor.L = 1e-4; 



test_gearbox.inertia = 1; 
test_gearbox.alpha = 2;
test_gearbox.eta = 0.9; 
test_gearbox.mass = 1; 


save('/home/erez/Documents/MotorSelection/database/test_vals.mat');