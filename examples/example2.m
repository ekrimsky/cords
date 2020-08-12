
%
%
%       Big block comment AND problem statement repeated word for word 
%
%
%
%
%
%

clearvars; clc; close all; 

%-----------------
%
%       Load in Trajectory and Geometry Data 
%
%--------------------------------------

load('example2_input_data.mat',...
         'time', ...
         'l_body',...   % robot length (front to back shouler)
         'l1',...       % shoulder to knee link length 
         'l2',...       % knee to foot link length 
         'theta_com',... % robot orientation trajectory 
         'theta_com_dot',... % robot angular vel
         'theta_com_ddot',... % robot angular accel
         'x_com',...        % x com trajectory 
         'x_com_dot',...    % robot x vel
         'x_com_ddot',...   % robot x accel
         'z_com',....       % z com trajetory 
         'z_com_dot',...    % robot z vel 
         'z_com_ddot',...   % robot z accel 
         'theta_rfs',... % right front shoulder 
         'theta_rfk',... % right front knee
         'theta_rhs',... % right hind shoulder 
         'theta_rhk',... % right hind knee 
         'theta_lfs',... % left front shoulder 
         'theta_lfk',... % left front knee
         'theta_lhs',... % left hind shoulder 
         'theta_lhk',... % left hind knee 
    ...  % Joint Velocities  
         'omega_rfs',... % right front shoulder 
         'omega_rfk',... % right front knee
         'omega_rhs',... % right hind shoulder 
         'omega_rhk',... % right hind knee 
         'omega_lfs',... % left front shoulder 
         'omega_lfk',... % left front knee
         'omega_lhs',... % left hind shoulder 
         'omega_lhk',... % left hind knee 
    ...  % Joint Accels 
         'omega_dot_rfs',... % right front shoulder 
         'omega_dot_rfk',... % right front knee
         'omega_dot_rhs',... % right hind shoulder 
         'omega_dot_rhk',... % right hind knee 
         'omega_dot_lfs',... % left front shoulder 
         'omega_dot_lfk',... % left front knee
         'omega_dot_lhs',... % left hind shoulder 
         'omega_dot_lhk',... % left hind knee 
    ...  % Foot placements 
         'RF_x',...
         'RF_z',...
         'RH_x',...
         'RH_z',...
         'LF_x',...
         'LF_z',...
         'LH_x',...
         'LH_z',...
    ...  % Foot relative to shoulder for computing moments 
         'del_RF_x',...
         'del_RH_x',...
         'del_LF_x',...
         'del_LH_x'); 



% Combinded joint velocities
omega_full = [omega_rfs; omega_rfk; omega_rhs; omega_rhk;...
                omega_lfs; omega_lfk; omega_lhs; omega_lhk];

% Combined joint accelerations 
omega_dot_full = [omega_dot_rfs; omega_dot_rfk; omega_dot_rhs; omega_dot_rhk;...
                omega_dot_lfs; omega_dot_lfk; omega_dot_lhs; omega_dot_lhk];


n = length(time);  % numbre of time points 
% Then a nice quick animation






%
%            Robot Data + Constants 
%
%
%
g = 9.81; % m/s^2 
mu = 0.5;       % footpad friction coefficeint  % TODO 
m_frame = 3;    % kg                         % TODO 
J_frame = 0;    % kg*m^2 
% Mass/Charge density for Li-FePO4
rho_batt = 360e3;   % J/kg  https://en.wikipedia.org/wiki/Electric_battery
N_motors = 8;       % 2 motors for each leg 


%---------------------------------------------
%
%                   Optimization Vector Setup 
%
%-------------------------------------------
%
%   x = [m_r                - total robot mass (motors + frame + battery)
%        m_b                - Battery Mass 
%        J_r                - total robot inertia about com (scalar)
%        e_gc               - Energy consumed per gait cycle 
%        p_rfs              - Right front shoulder power 
%        p_rfk              - Right
%        p_rhs
%        p_rhk
%        p_lfs              - Left front shoulder power 
%        p_lfk              - Left
%        p_lhs
%        p_lhk
%        p_total 
%        F_RF_x
%        F_RF_z
%        F_RH_x
%        F_RH_z
%        F_LF_x
%        F_LF_z
%        F_LH_x
%        F_LH_z
%                   ]



% Define all the indices 
m_r_idx = 1; 
m_b_idx = 2; 
J_r_idx = 3; 
e_gc_idx = 4; 

p_rfs_idxs = e_gc_idx + (1:n);
p_rfk_idxs = p_rfs_idxs(end) + (1:n);
p_rhs_idxs = p_rfk_idxs(end) + (1:n);
p_rhk_idxs = p_rhs_idxs(end) + (1:n);
p_lfs_idxs = p_rhk_idxs(end) + (1:n);
p_lfk_idxs = p_lfs_idxs(end) + (1:n);
p_lhs_idxs = p_lfk_idxs(end) + (1:n);
p_lhk_idxs = p_lhs_idxs(end) + (1:n);
p_total_idxs = p_lhk_idxs(end) + (1:n);

F_RF_x_idxs = p_total_idxs(end) + (1:n);
F_RF_z_idxs = F_RF_x_idxs(end) + (1:n);

F_RH_x_idxs = F_RF_z_idxs(end) + (1:n);
F_RH_z_idxs = F_RH_x_idxs(end) + (1:n);

F_LF_x_idxs = F_RH_z_idxs(end) + (1:n);
F_LF_z_idxs = F_LF_x_idxs(end) + (1:n);

F_LH_x_idxs = F_LF_z_idxs(end) + (1:n);
F_LH_z_idxs = F_LH_x_idxs(end) + (1:n);


dim_x = F_LH_z_idxs(end); % dimension of x vector 


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
H_Fz(:, F_RF_z_idxs) = eye(n); 
H_Fz(:, F_RH_z_idxs) = eye(n); 
H_Fz(:, F_LF_z_idxs) = eye(n); 
H_Fz(:, F_LH_z_idxs) = eye(n); 
H_Fz(:, m_r_idx) = -(z_com_ddot + g);
b_Fz = zeros(n, 1);


%
%
%    Sum Forces in X - Accounting for total robot mass 
%
%
H_Fx = sparse([], [], [], n, dim_x, 5*n);
H_Fx(:, F_RF_x_idxs) = eye(n); 
H_Fx(:, F_RH_x_idxs) = eye(n); 
H_Fx(:, F_LF_x_idxs) = eye(n); 
H_Fx(:, F_LH_x_idxs) = eye(n); 
H_Fx(:, m_r_idx) = -x_com_ddot;
b_Fx = zeros(n, 1);


%{
%
%
%    Sum Moments About X (left and right forces match)
%
%
H_Mx = sparse([], [], [], n, dim_x, 4*n);
H_Mx(:, F_RF_z_idxs) = -eye(n); 
H_Mx(:, F_RH_z_idxs) = -eye(n); 
H_Mx(:, F_LF_z_idxs) = eye(n); 
H_Mx(:, F_LH_z_idxs) = eye(n); 
b_Mx = zeros(n, 1);
%}
H_Mx = [];  % TODO -- formalize and remove  
b_Mx = [];


%
%
%    Sum Moments About Y - 
%
%
H_My = sparse([], [], [], n, dim_x, 9*n);
H_My(:, F_RF_z_idxs) = diag(0.5*l_body*cos(theta_com)...
                                                 + del_RF_x); 
H_My(:, F_RH_z_idxs) = diag(-0.5*l_body*cos(theta_com)...
                                                 + del_RH_x); 
H_My(:, F_LF_z_idxs) = diag(0.5*l_body*cos(theta_com)...
                                                 + del_LF_x); 
H_My(:, F_LH_z_idxs) = diag(-0.5*l_body*cos(theta_com)...
                                                 + del_LH_x); 
H_My(:, F_RF_x_idxs) = diag(z_com);
H_My(:, F_RH_x_idxs) = diag(z_com);
H_My(:, F_LF_x_idxs) = diag(z_com);
H_My(:, F_LH_x_idxs) = diag(z_com);
H_My(:, J_r_idx) = -theta_com_ddot;
b_My = zeros(n, 1);  



%
%
%   Normal force is zero when foot not in contact 
%
%
%       Can encode as:
%           Foot Height * Normal Force = 0  
%       So normal force is zero unless foot height
%       is zero (ie. foot is in contact with ground)
%
H_normal = sparse([], [], [], 4*n, dim_x, 4*n);
H_normal(       1:n,  F_RF_z_idxs) = diag(RF_z);
H_normal(n +   (1:n), F_RH_z_idxs) = diag(RH_z);
H_normal(2*n + (1:n), F_LF_z_idxs) = diag(LF_z);
H_normal(3*n + (1:n), F_LH_z_idxs) = diag(LH_z);
b_normal = zeros(4*n, 1); 



% Make energy per gait cycle equal to Integral of power consumption
% using trapezoidal integation 
H_energy = sparse([], [], [], 1, dim_x, n + 1);
H_energy(1, p_total_idxs(1):p_total_idxs(end - 1)) = 0.5*ones(1, n - 1).*diff(time');
H_energy(1, p_total_idxs(2):p_total_idxs(end)) = ...
                             H_energy(1, p_total_idxs(2):p_total_idxs(end))...
                                             + 0.5*ones(1, n - 1).*diff(time');
H_energy(1, e_gc_idx) = -1;
b_energy = 0; 

% Relate Total Mass to Battery Mass + Frame Mass + Motor/Gearbox Mass 
H_mass = sparse([1, 1], [m_r_idx, m_b_idx], [1, -1], 1, dim_x);
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

% TODO 
%{
H = [H_Fz; H_Fx; H_Mx; H_My; H_normal; H_energy; H_mass; H_inertia];
b_const = [b_Fz; b_Fx; b_Mx; b_My; b_normal; b_energy];
b = @(motor, gearbox) [b_const; b_mass(motor, gearbox);...
                                            b_inertia(motor, gearbox)];
%}

H = [H_Fz; H_Fx; H_Mx; H_My; H_normal; H_energy; H_mass; H_inertia];
b = @(motor, gearbox) [b_Fz; b_Fx; b_Mx; b_My; b_normal; b_energy;...
                    b_mass(motor, gearbox); b_inertia(motor, gearbox)];

%------------------------------------------------
%
%               Inquality Constraints 
%
%-----------------------------------------------


%
%       Nonnegative Normal Forces 
%       Contact can only push up not pull down 
%

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
H_ineq_fc_rf(1:n, F_RF_x_idxs) = eye(n); 
H_ineq_fc_rf(n+1:end, F_RF_x_idxs) = -eye(n); 
H_ineq_fc_rf(:, F_RF_z_idxs) = -mu*[eye(n); eye(n)]; 
b_ineq_fc_rf = zeros(2*n, 1); 

% right hind 
H_ineq_fc_rh = sparse([], [], [], 2*n, dim_x, 4*n); 
H_ineq_fc_rh(1:n, F_RH_x_idxs) = eye(n); 
H_ineq_fc_rh(n+1:end, F_RH_x_idxs) = -eye(n); 
H_ineq_fc_rh(:, F_RH_z_idxs) = -mu*[eye(n); eye(n)]; 
b_ineq_fc_rh = zeros(2*n, 1); 

% left front 
H_ineq_fc_lf = sparse([], [], [], 2*n, dim_x, 4*n); 
H_ineq_fc_lf(1:n, F_LF_x_idxs) = eye(n); 
H_ineq_fc_lf(n+1:end, F_LF_x_idxs) = -eye(n); 
H_ineq_fc_lf(:, F_LF_z_idxs) = -mu*[eye(n); eye(n)];
b_ineq_fc_lf = zeros(2*n, 1); 

% left hind 
H_ineq_fc_lh = sparse([], [], [], 2*n, dim_x, 4*n); 
H_ineq_fc_lh(1:n, F_LH_x_idxs) = eye(n); 
H_ineq_fc_lh(n+1:end, F_LH_x_idxs) = -eye(n); 
H_ineq_fc_lh(:, F_LH_z_idxs) = -mu*[eye(n); eye(n)];
b_ineq_fc_lh = zeros(2*n, 1); 

% combine 
H_ineq_fc = [H_ineq_fc_rf; H_ineq_fc_rh; H_ineq_fc_lf; H_ineq_fc_lh];
b_ineq_fc = [b_ineq_fc_rf; b_ineq_fc_rh; b_ineq_fc_lf; b_ineq_fc_lh];

%
%
%       Account for recharge inefficiency 
%
%
Psi = 0.85; % regen efficincy Psi 

H_ineq_power = sparse([], [], [], 2*n, dim_x, 2*(N_motors + 1));

H_ineq_power(1:n, p_rfs_idxs) = eye(n); 
H_ineq_power(1:n, p_rfk_idxs) = eye(n); 
H_ineq_power(1:n, p_rhs_idxs) = eye(n); 
H_ineq_power(1:n, p_rhk_idxs) = eye(n); 
H_ineq_power(1:n, p_lfs_idxs) = eye(n); 
H_ineq_power(1:n, p_lfk_idxs) = eye(n); 
H_ineq_power(1:n, p_lhs_idxs) = eye(n); 
H_ineq_power(1:n, p_lhk_idxs) = eye(n); 
H_ineq_power(1:n, p_total_idxs) = -eye(n); 

H_ineq_power(n + (1:n), p_rfs_idxs) = Psi*eye(n); 
H_ineq_power(n + (1:n), p_rfk_idxs) = Psi*eye(n); 
H_ineq_power(n + (1:n), p_rhs_idxs) = Psi*eye(n); 
H_ineq_power(n + (1:n), p_rhk_idxs) = Psi*eye(n); 
H_ineq_power(n + (1:n), p_lfs_idxs) = Psi*eye(n); 
H_ineq_power(n + (1:n), p_lfk_idxs) = Psi*eye(n); 
H_ineq_power(n + (1:n), p_lhs_idxs) = Psi*eye(n); 
H_ineq_power(n + (1:n), p_lhk_idxs) = Psi*eye(n); 
H_ineq_power(n + (1:n), p_total_idxs) = -eye(n); 

b_ineq_power = zeros(2*n, 1); 


% Combine into general inequality matrix 
H_ineq = [H_ineq_normal; H_ineq_fc; H_ineq_power];
b_ineq = [b_ineq_normal; b_ineq_fc; b_ineq_power]; 

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
T_lhk(:, F_LH_x_idxs) = -l2 * diag(sin(theta_lhk));
T_lhk(:, F_LH_z_idxs) =  l2 * diag(cos(theta_lhk));
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

% Drive board efficiency Phi
Phi = 0.98; % driver board efficienc 
Phi_inv = 1/Phi; 

for motor = 1:N_motors % loop through motors 

    switch motor
        case 1     % RFS 
            p_idxs = p_rfs_idxs;
            omega = omega_rfs;
        case 2      % RFK 
            p_idxs = p_rfk_idxs;
            omega = omega_rfk;
        case 3      % RHS
            p_idxs = p_rhs_idxs;
            omega = omega_rhs;
        case 4      % RHK
            p_idxs = p_rhk_idxs;
            omega = omega_rhk;
        case 5      % LFS 
            p_idxs = p_lfs_idxs;
            omega = omega_lfs;
        case 6      % LFK 
            p_idxs = p_lfk_idxs;
            omega = omega_lfk;
        case 7      % LHS
            p_idxs = p_lhs_idxs;
            omega = omega_lhs;
        case 8      % LHK
            p_idxs = p_lhk_idxs;
            omega = omega_lhk;
    end  % switch

    for j = 1:n % loop through time points 
        idx = (motor - 1)*n + j; % corresponding index in omega_full
        e_j = zeros(N_motors*n, 1);  
        % pick out this motor and time point
        e_j(idx) = 1; % one hot vector / unit basis vector 
        spdg_j =  sparse(diag(e_j));  

        r_motor_j = zeros(dim_x, 1);
        r_motor_j(p_idxs(j)) = -1; 
        
        Q{end + 1, 1} = @(motor, ~) Phi_inv*motor.R * spdg_j; 
        c{end + 1, 1} = @(motor, gearbox) Phi_inv*motor.k_e*gearbox.alpha*omega(j)*e_j;  % TODO -- k_e 
        M{end + 1, 1} = []; 
        r{end + 1, 1} = r_motor_j;  % accounting for the other vars 
        bet{end + 1, 1} = 0; 

        % If negative MOTOR power consumption
        Q{end + 1, 1} = @(motor, ~) Phi*motor.R * spdg_j; 
        c{end + 1, 1} = @(motor, gearbox) Phi*motor.k_e*gearbox.alpha*omega(j)*e_j;     % TODO-- k_e 
        M{end + 1, 1} = []; 
        r{end + 1, 1} = r_motor_j;  % accounting for the other vars 
        bet{end + 1, 1} = 0; 
    end 

end 






%-----------------------------------------
%
%       Voltage and Current Limits 
%
%----------------------------------------

V_max = 48;     % Volts 
I_max = 100;    % Amps 

I_u = @(motor, gearbox) min(I_max,...
         (1/motor.R)* min(abs(V_max + motor.k_e*gearbox.alpha.*omega_full),...
               abs(V_max + motor.k_e*gearbox.alpha.*omega_full) ) ); 


%-----------------------------------
%
%   Linear Fractional Objective
%
%------------------------------------------
%
%       Requires an explanaition of both the objective here AND 
%        the general setup for linear fractioal problems 
%
%
%           
%
%

% minimize gait_cycle_energy/battery_energy (1/num_steps)


% If (energy per gait)/(battery energy) = 0, can walk forever
%   --> So cost lower bound as 0 
% If (energy per gait)/(battery energy) = 1, can only walk 1 gait cycle 
%   --> So set 1 as upper bound 

r_num = zeros(dim_x, 1); 
r_num(e_gc_idx) = 1; 
r_den = zeros(dim_x, 1);
r_den(m_b_idx) = rho_batt; 
beta_num = 0;
beta_den = 0; 

cost_lb = 0;
cost_ub = 1; 


% OR 
% minmize negative number of steps 
% minimize -battery_energy/gait_cycle_energy
%
%  upper bound -1 (1 step)
%  2 steps per second, -- 
%{
week_of_steps = 2*60*60*24*7;

r_num = zeros(dim_x, 1); 
r_num(m_b_idx) = -rho_batt; 
r_den = zeros(dim_x, 1);
r_den(e_gc_idx) = 1; 
beta_num = 0;
beta_den = 0; 

cost_lb = -week_of_steps;
cost_ub = -1; 
%}







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
problem_data.H_ineq = H_ineq;
problem_data.b_ineq = b_ineq;
problem_data.T = T;
problem_data.d = d;
problem_data.omega = omega_full; 
problem_data.omega_dot = omega_dot_full; % TODO for inertial compensation
problem_data.I_u = I_u; 


problem_data.cost_lb = cost_lb; 
problem_data.cost_ub = cost_ub; 
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
num_solutions = 3; % number of best combinations to return 
sol_struct = prob.optimize(num_solutions);

sol = sol_struct(1).sol; 


% Nice plots and convert this to a distance 


%------------
%
%       Process Results and index out solutions 
%
%-----------------



%--------------------
%
%       Plot 
%
%
%----------------



