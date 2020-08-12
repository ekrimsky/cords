
%
%
%       Big block comment AND problem statement repeated word for word 
%
%
%
%
%
%






clearvars; clc;
% Another idea for the website 
% ... have pictures and jupiter notebook type 
% deal where we walk through each of the examples 

% Get Trajectory Data 



% TRjactrory defined by these things.....




% Then a nice quick animation


% TODO -- package up so can refenerence correctly 
load('example2_input_data.mat',...
         'time', ...
          'theta',...
           'omega',...
         'omega_dot',...
          'tau_des'); 


%
%            Robot Data + Constants 
%
%
%


% all the angles 

g = 9.81; % m/s^2 

J_frame = 
m_frame = 
rho_batt = 1;   % kg/coulomb 
l1 = 

l2 = 



% 




ddot_z = zeros(n, 1);  % TODO (from inputs )
ddot_x = 

% Need robot data -- including firction coefficient 
% and maybe a stiffness for impulse models 

N_mot = 8; % 2 motors for each leg 
%
%
%
%  Optimization Vector Setup 
%
%
%   x = [m_r                - total robot mass (motors + frame + battery)
%        m_b                - Battery Mass 
%        J_r                - total robot inertia about com (scalar)
%        e_gc               - Energy consumed per gait cycle 
%        p  \in R^(8 x N)   - Power consumed at each time point 
%        F_LH_x
%        F_LH_z
%        F_LF_x
%        F_LF_z
%        F_RH_x
%        F_RH_z
%        F_RF_x
%        F_RF_z
%           ]
g = 9.81; 


m_r_idx = 1; 
m_b_idx = 2; 
J_r_idx = 3; 
e_gc_idx = 4; 
p_idxs = e_gc_idx + (1:n);



dim_x = % 








% Plot up here showing how it looks? 


% Define indices 





 % Inputs ddot_z, ddot_x, lbody, ddot_theta_com, theta_com
    % z_com, 


% This part doesnt change so define ones   

% WILL WANT TO PROFILE THIS --
% A LOT OF THIS CODE IS THE SAME EVERY TIME 

%------------------------------------------------
%
%               Equality Constraints 
%
%-----------------------------------------------



%
%
%    Sum Forces in Z - Accounting for total robot mass 
%
%
H_Fz = sparse([], [], [], n, dim_x, 5*n);
H_Fz(:, indices.F_LH_z_idxs) = eye(n); 
H_Fz(:, indices.F_LF_z_idxs) = eye(n); 
H_Fz(:, indices.F_RH_z_idxs) = eye(n); 
H_Fz(:, indices.F_RF_z_idxs) = eye(n); 
H_Fz(:, indices.mass_idx) = -(ddot_z + g);
b_Fz = zeros(n, 1);


%
%
%    Sum Forces in X - Accounting for total robot mass 
%
%
H_Fx = sparse([], [], [], n, dim_x, 5*n);
H_Fx(:, indices.F_LH_x_idxs) = eye(n); 
H_Fx(:, indices.F_LF_x_idxs) = eye(n); 
H_Fx(:, indices.F_RH_x_idxs) = eye(n); 
H_Fx(:, indices.F_RF_x_idxs) = eye(n); 
H_Fx(:, indices.mass_idx) = -ddot_x;
b_Fx = zeros(n, 1);


%
%
%    Sum Moments About X (left and right forces match)
%
%
H_Mx = sparse([], [], [], n, dim_x, 4*n);
H_Mx(:, indices.F_LH_z_idxs) = eye(n); 
H_Mx(:, indices.F_LF_z_idxs) = eye(n); 
H_Mx(:, indices.F_RH_z_idxs) = -eye(n); 
H_Mx(:, indices.F_RF_z_idxs) = -eye(n); 
b_Mx = zeros(n, 1);


%
%
%    Sum Moments About Y - 
%
%
H_My = sparse([], [], [], n, dim_x, 9*n);
H_My(:, indices.F_LH_z_idxs) = diag(-0.5*l_body*cos(theta_com)...
                                                 + del_LH_x); 
H_My(:, indices.F_LF_z_idxs) = diag(0.5*l_body*cos(theta_com)...
                                                 + del_LF_x); 
H_My(:, indices.F_RH_z_idxs) = diag(-0.5*l_body*cos(theta_com)...
                                                 + del_RH_x); 
H_My(:, indices.F_RF_z_idxs) = diag(0.5*l_body*cos(theta_com)...
                                                 + del_RF_x); 
H_My(:, indices.F_LH_x_idxs) = diag(z_com);
H_My(:, indices.F_LF_x_idxs) = diag(z_com);
H_My(:, indices.F_RH_x_idxs) = diag(z_com);
H_My(:, indices.F_RF_x_idxs) = diag(z_com);
b_My = zeros(n, 1);   % TODO -- depends on total inertia 



%
%
%   Normal force is zero when foot not in contact 
%
%
H_normal = sparse([], [], [], n, dim_x, 4*n);
H_normal(:, indices.F_LH_z_idxs) = diag(LH_z);
H_normal(:, indices.F_LF_z_idxs) = diag(LF_z);
H_normal(:, indices.F_RH_z_idxs) = diag(RH_z);
H_normal(:, indices.F_RF_z_idxs) = diag(RF_z);
b_normal = zeros(n, 1); 



% Make energy per gait cycle equal to Integral of power consumption
% using trapezoidal integation 
H_energy = sparse([], [], [], 1, dim_x, n + 1);
H_energy(p_idxs(1):p_idxs(end - 1)) = 0.5*ones(n - 1, 1).*diff(time);
H_energy(p_idxs(2):p_idxs(end)) = H_energy(p_idxs(2):p_idxs(end))...
                                     + 0.5*ones(n - 1, 1).*diff(time);
H_energy(e_gc_idx) = -1;
b_energy = 0; 

% Relate Total Mass to Battery Mass
H_mass = sparse(1, m_r_idx, 1, 1, dim_x);
b_mass = @(motor, gearbox) -(m_frame + 8*(motor.mass + gearbox.mass)); 

% Relate total inertia (about com) to motor &gearbox mass
H_inertia = sparse(1, J_r_idx, 1, 1, dim_x);
b_inertia = @(motor, gearbox) -(J_frame + ...
                        8*(motor.mass + gearbox.mass)*(l_body/2)^2);




%
%
%   Combine the equality constraints on x 
%
%
H = [H_Fz; H_Fx; H_Mx; H_My; H_normal; H_energy; H_mass; H_inertia];
b_const = [b_Fz; b_Fx; b_My; b_normal; b_energy]
b = @(motor, gearbox) [b_const; b_mass(motor, gearbox);...
                                            b_inertia(motor, gearbox)];


%------------------------------------------------
%
%               Inquality Constraints 
%
%-----------------------------------------------


%
%       Nonnegative Normal Forces 
%       Contact can only push up not pull down 
%
%       Can encode as:
%           Foot Height * Normal Force = 0  
%       So normal force is zero unless foot height
%       is zero (ie. foot is in contact with ground)
%
col = [F_RF_z_idxs; F_RH_z_idxs; F_LF_z_idxs; F_LH_z_idxs]; 
vals = -ones(4*n, 1); 
H_ineq_normal = sparse(1:4*n, col, vals, 4*n, dim_x); 
b_ineq_normal = zeros(4*n, 1); 

%
%
%       Friction Cone (fc)
%       Ground reaction force (grf) must lie in friction cone   
%
%                |Fx| <= mu Fz
%
%       Can write as 2 inequalities:
%               Fx - mu Fz <= 0 
%              -Fx - mu Fz <= 0 
%
%
% right front 
H_ineq_fc_rf = sparse([], [], [], 2*n, dim_x, 4*n); 
H_ineq_fc_rf(1:n, F_RF_x_idxs) = 1; 
H_ineq_fc_rf(n+1:end, F_RF_x_idxs) = -1; 
H_ineq_fr_rf(:, F_RF_z_idxs) = -mu; 
b_ineq_fc_rf = zeros(2*n, 1); 

% right hind 
H_ineq_fc_rh = sparse([], [], [], 2*n, dim_x, 4*n); 
H_ineq_fc_rh(1:n, F_RH_x_idxs) = 1; 
H_ineq_fc_rh(n+1:end, F_RH_x_idxs) = -1; 
H_ineq_fr_rh(:, F_RH_z_idxs) = -mu; 
b_ineq_fc_rh = zeros(2*n, 1); 

% left front 
H_ineq_fc_lf = sparse([], [], [], 2*n, dim_x, 4*n); 
H_ineq_fc_lf(1:n, F_LF_x_idxs) = 1; 
H_ineq_fc_lf(n+1:end, F_LF_x_idxs) = -1; 
H_ineq_fr_lf(:, F_LF_z_idxs) = -mu; 
b_ineq_fc_lf = zeros(2*n, 1); 

% left hind 
H_ineq_fc_lh = sparse([], [], [], 2*n, dim_x, 4*n); 
H_ineq_fc_lh(1:n, F_LH_x_idxs) = 1; 
H_ineq_fc_lh(n+1:end, F_LH_x_idxs) = -1; 
H_ineq_fr_lh(:, F_LH_z_idxs) = -mu; 
b_ineq_fc_lh = zeros(2*n, 1); 

% combine 
H_ineq_fc = [H_ineq_fc_rf; H_ineq_fc_rh; H_ineq_fc_lf; H_ineq_fc_lh];
b_ineq_fc = [b_ineq_fc_rf; b_ineq_fc_rh; b_ineq_fc_lf; b_ineq_fc_lh];

% Combine into general inequality matrix 
H_ineq = [H_ineq_normal; H_ineq_fc];
b_ineq = [b_ineq_normal; b_ineq_fc]; 

%-------------------------------------------------------------
%
%       Torques -- Map GRFs and Dynamics into Desired
%               motor torques. 
%
%----------------------------------------------------------------

%% %
%
%  8 Motors 
%       - Right Front Shoulder  (rfs)
%       - Right Front Knee      (rfk)
%       - Right Hind Shoulder   (rhs)
%       - Right Hind Knee       (rhk)
%       - Left Front Shoulder   (lfs)
%       - Left Front Knee       (lfk)
%       - Left Hind Shoulder    (lhs)
%       - Left Hind Knee        (lhk)
%
%
%  We will stack the velocities and accelerations for each 
%  together into omega and omega_dot 


%  Right Front Shoulder (rfs)
T_rfs = sparse([], [], [], n, dim_x, 2*n);
T_rfs(:, F_RF_x_idxs) = -l1 * diag(sin(theta_rfs));
T_rfs(:, F_RF_z_idxs) =  l1 * diag(cos(theta_rfs));
d_rfs = zeros(n, 1); 

% Right Front Knee (rfk)
T_rfk = sparse([], [], [], n, dim_x, 2*n);
T_rfk(:, F_RF_x_idxs) = -l2 * diag(sin(theta_rfk));
T_rfk(:, F_RF_z_idxs) =  l2 * diag(cos(theta_rfk));
d_rfk = zeros(n, 1); 

%  Right Hind Shoulder (rhs)
T_rhs = sparse([], [], [], n, dim_x, 2*n);
T_rhs(:, F_RH_x_idxs) = -l1 * diag(sin(theta_rhs));
T_rhs(:, F_RH_z_idxs) =  l1 * diag(cos(theta_rhs));
d_rhs = zeros(n, 1); 

% Right Hind Knee (rhk)
T_rhk = sparse([], [], [], n, dim_x, 2*n);
T_rhk(:, F_RH_x_idxs) = -l2 * diag(sin(theta_rhk));
T_rhk(:, F_RH_z_idxs) =  l2 * diag(cos(theta_rhk));
d_rhk = zeros(n, 1); 

% Left Front Shoulder (lfs)
T_lfs = sparse([], [], [], n, dim_x, 2*n);
T_lfs(:, F_LF_x_idxs) = -l1 * diag(sin(theta_lfs));
T_lfs(:, F_LF_z_idxs) =  l1 * diag(cos(theta_lfs));
d_lfs = zeros(n, 1); 

% Left Front Knee (lfk)
T_lfk = sparse([], [], [], n, dim_x, 2*n);
T_lfk(:, F_LF_x_idxs) = -l2 * diag(sin(theta_lfk));
T_lfk(:, F_LF_z_idxs) =  l2 * diag(cos(theta_lfk));
d_lfk = zeros(n, 1); 

%  Left Hind Shoulder (lhs)
T_lhs = sparse([], [], [], n, dim_x, 2*n);
T_lhs(:, F_LH_x_idxs) = -l1 * diag(sin(theta_lhs));
T_lhs(:, F_LH_z_idxs) =  l1 * diag(cos(theta_lhs));
d_lhs = zeros(n, 1); 

% Left Hind Knee (lhk)
T_lhk = sparse([], [], [], n, dim_x, 2*n);
T_lhk(:, F_LH_x_idxs) = -l2 * diag(sin(theta_rhk));
T_lhk(:, F_LH_z_idxs) =  l2 * diag(cos(theta_rhk));
d_lhk = zeros(n, 1); 


T = [T_rfs; T_rfk; T_rhs; T_rhk; T_lfs; T_lfk; T_lhs; T_lhk];
d = [d_rfs; d_rfk; d_rhs; d_rhk; d_lfs; d_lfk; d_lhs; d_lhk];


%-------------------------------------------------------
%
%     Power Consumption - In Constraints 
%
%----------------------------------------------------------


% Initialize Cell Arrays 
Q = {}; 
c = {};   % ...etc 
M = {};
r = {}; 
bet = {};


% Standard (SOCP) objective Terms all empty because linear fractional
Q{1, 1} = []; 
c{1, 1} = []; 
M{1, 1} = [];
r{1, 1} = []; 
bet{1, 1} = [];

% Define efficiencies psi, 
Phi = 0.95; % driver board efficienc 
Phi_inv = 1/Phi; 
Psi = 0.8; % regen efficincy psi 
% Fill in general inequality constraints 
for j = 1:n

    % If positive power conumption 
    e_j = zeros(n, 1); 
    e_j(j) = 1; % one hot vector / unit basis vector 

    % NOTE  -- would it be more efficient to divide through by R so no Q dependednce on inputs? Still r and c dependence though 
    spdg_j =  sparse(diag(e_j)); 
    Q{end + 1, 1} = @(motor, ~) Phi_inv*motor.R * spdg_j; 
    c{end + 1, 1} = @(motor, gearbox) Phi_inv*motor.k_t*gearbox.alpha*omega(j)*e_j;  % TODO -- k_e 
    M{end + 1, 1} = []; 
    r{end + 1, 1} = [0; 0; 0; 0; -e_j];  % accounting for the other vars 
    bet{end + 1, 1} = 0; 

    % If negative power consumption
    Q{end + 1, 1} = @(motor, ~) Psi*motor.R * spdg_j; 
    c{end + 1, 1} = @(motor, gearbox) Psi*motor.k_t*gearbox.alpha*omega(j)*e_j;     % TODO-- k_e 
    M{end + 1, 1} = []; 
    r{end + 1, 1} = [0; 0; 0; 0; -e_j];  % accounting for the other vars 
    bet{end + 1, 1} = 0; 
end 

%
%
%       Voltage and Current Limits 
%
%

V_max = 48; % volts  
I_max = 100;  % Amps 

I_u = @(motor, gearbox) min(I_max,...
         (1/motor.R)* min(abs(V_max + motor.k_e*gearbox.alpha.*omega),...
               abs(V_max + motor.k_e*gearbox.alpha.*omega) ) ); 


%
%
%   Linear Fractional Objective
%
%
%
%       Requires an explanaition of both the objective here AND 
%        the general setup for linear fractioal problems 
%
%

























% Block comment at the begining of each of these would be good 

%
%
%       Call the solver!
%
%


%

% Assign all to the same struct 
problem_data.Q = Q;
problem_data.c = c; 
problem_data.M = M; 
problem_data.r = r; 
problem_data.beta = bet; 
problem_data.H = H;    % maybe A instead 
problem_data.b = b; 
problem_data.T = T;
problem_data.d = d;
problem_data.omega = omega; 
problem_data.I_u = I_u; 


r_num = zeros(w, 1); 
r_num(ec_idx) = 1; 
r_den = zeros(w, 1);
r_den(cb_idx) = 1; 
beta_num = 0;
beta_den = 0; 


problem_data.cost_lb = 0; 
problem_data.cost_ub = 10; 
problem_data.r_num = r_num;
problem_data.r_den = r_den; 
problem_data.beta_num = beta_num;
problem_data.beta_den = beta_den; 
% Solve the problem 


prob = MotorSelection(); % instantiate new motor selection problem 

prob.update_problem(problem_data);   % update with the data 

%{
prob.filter_motors 
prob.filter_gears 

% consider whats the best way to do this 
prob.hints = % HOW BEST TO DO THIS? 
%} 

% Need to think about what return types should be 
% What inputs would make sense????? 

sol_struct = prob.optimize(10);

sol = sol_struct(1).sol; 


% Extract Results 
kp = sol.x(kp_idx)
tau_c = sol.x(tau_c_idx);
theta_0 = tau_c/kp; 


%energy = 

% TODO - would be good to return current AND aux var separately 
% also want to return cost -- how to return motors is harder question 
% NOTE: could be good to have a seperate return of motor torque 
% even though it could be calculated externally 


% Plot results 

% show break up of torque 



% define one-hot in src 



