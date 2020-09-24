
%
%
%       Big block comment AND problem statement repeated word for word 
%
%
%
%
%
%
addpath('examples/example_data');

clearvars; clc; close all; 

%-----------------
%
%       Load in Trajectory and Geometry Data 
%
%--------------------------------------

load('quadruped_input_data.mat',...
         't_gait',...
         't_jump',...
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

% Trajectory is walking trajectory with short jumping trajectory added at end 

n_walk = length(t_gait);
n_jump = length(t_jump);
n = n_walk + n_jump;  % total number of time points for combined traj 
% Then a nice quick animation

% Need to add something to make this more reaslistc -- right now high gear ratios and heavy batteries 



%
%            Robot Data + Constants 
%
%
%
g = 9.81; % m/s^2 
mu = 0.5;       % footpad friction coefficeint 
m_frame = 3;    % kg                         
J_frame = 0.5;    % kg*m^2 
% Mass/Charge density for Li-FePO4
rho_batt = 360e3;   % J/kg  https://en.wikipedia.org/wiki/Electric_battery
N_motors = 8;       % 2 motors for each leg 

V_max = 48;     % Volts 
I_max = 100;    % Amps 

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


% Power consumption only on walking portion of trajectory 
p_rfs_idxs = e_gc_idx + (1:n_walk);
p_rfk_idxs = p_rfs_idxs(end) + (1:n_walk);
p_rhs_idxs = p_rfk_idxs(end) + (1:n_walk);
p_rhk_idxs = p_rhs_idxs(end) + (1:n_walk);
p_lfs_idxs = p_rhk_idxs(end) + (1:n_walk);
p_lfk_idxs = p_lfs_idxs(end) + (1:n_walk);
p_lhs_idxs = p_lfk_idxs(end) + (1:n_walk);
p_lhk_idxs = p_lhs_idxs(end) + (1:n_walk);
p_total_idxs = p_lhk_idxs(end) + (1:n_walk);

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
G_Fz = sparse([], [], [], n, dim_x, 5*n);
G_Fz(:, F_RF_z_idxs) = eye(n); 
G_Fz(:, F_RH_z_idxs) = eye(n); 
G_Fz(:, F_LF_z_idxs) = eye(n); 
G_Fz(:, F_LH_z_idxs) = eye(n); 
G_Fz(:, m_r_idx) = -(z_com_ddot + g);
h_Fz = zeros(n, 1);


%
%
%    Sum Forces in X - Accounting for total robot mass 
%
%
G_Fx = sparse([], [], [], n, dim_x, 5*n);
G_Fx(:, F_RF_x_idxs) = eye(n); 
G_Fx(:, F_RH_x_idxs) = eye(n); 
G_Fx(:, F_LF_x_idxs) = eye(n); 
G_Fx(:, F_LH_x_idxs) = eye(n); 
G_Fx(:, m_r_idx) = -x_com_ddot;
h_Fx = zeros(n, 1);


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
G_Mx = [];  % TODO -- formalize and remove  
h_Mx = [];


%
%
%    Sum Moments About Y - 
%
%
G_My = sparse([], [], [], n, dim_x, 9*n);
G_My(:, F_RF_z_idxs) = diag(0.5*l_body*cos(theta_com)...
                                                 + del_RF_x); 
G_My(:, F_RH_z_idxs) = diag(-0.5*l_body*cos(theta_com)...
                                                 + del_RH_x); 
G_My(:, F_LF_z_idxs) = diag(0.5*l_body*cos(theta_com)...
                                                 + del_LF_x); 
G_My(:, F_LH_z_idxs) = diag(-0.5*l_body*cos(theta_com)...
                                                 + del_LH_x); 
G_My(:, F_RF_x_idxs) = diag(z_com);
G_My(:, F_RH_x_idxs) = diag(z_com);
G_My(:, F_LF_x_idxs) = diag(z_com);
G_My(:, F_LH_x_idxs) = diag(z_com);
G_My(:, J_r_idx) = -theta_com_ddot;
h_My = zeros(n, 1);  



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
G_normal = sparse([], [], [], 4*n, dim_x, 4*n);
G_normal(       1:n,  F_RF_z_idxs) = diag(RF_z);
G_normal(n +   (1:n), F_RH_z_idxs) = diag(RH_z);
G_normal(2*n + (1:n), F_LF_z_idxs) = diag(LF_z);
G_normal(3*n + (1:n), F_LH_z_idxs) = diag(LH_z);
h_normal = zeros(4*n, 1); 



% Make energy per gait cycle equal to Integral of power consumption
% using trapezoidal integation 
G_energy = sparse([], [], [], 1, dim_x, n_walk + 1);
G_energy(1, p_total_idxs(1):p_total_idxs(end - 1)) = 0.5*ones(1, n_walk - 1).*diff(t_gait');
G_energy(1, p_total_idxs(2):p_total_idxs(end)) = ...
                             G_energy(1, p_total_idxs(2):p_total_idxs(end))...
                                             + 0.5*ones(1, n_walk - 1).*diff(t_gait');
G_energy(1, e_gc_idx) = -1;
h_energy = 0; 

% Relate Total Mass to Battery Mass + Frame Mass + Motor/Gearbox Mass 
G_mass = sparse([1, 1], [m_r_idx, m_b_idx], [1, -1], 1, dim_x);
h_mass = @(motor, gearbox) -(m_frame + 8*(motor.mass + gearbox.mass)); 

% Relate total inertia (about com) to motor &gearbox mass
G_inertia = sparse(1, J_r_idx, 1, 1, dim_x);
h_inertia = @(motor, gearbox) -(J_frame + ...
                        8*(motor.mass + gearbox.mass)*(l_body/2)^2);

%
%
%   Combine the equality constraints on x 
%
%


G = [G_Fz; G_Fx; G_Mx; G_My; G_normal; G_energy; G_mass; G_inertia];
h = @(motor, gearbox) [h_Fz; h_Fx; h_Mx; h_My; h_normal; h_energy;...
                    h_mass(motor, gearbox); h_inertia(motor, gearbox)];

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
G_ineq_normal = sparse(1:4*n, col, vals, 4*n, dim_x); 
h_ineq_normal = zeros(4*n, 1); 

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
G_ineq_fc_rf = sparse([], [], [], 2*n, dim_x, 4*n); 
G_ineq_fc_rf(1:n, F_RF_x_idxs) = eye(n); 
G_ineq_fc_rf(n+1:end, F_RF_x_idxs) = -eye(n); 
G_ineq_fc_rf(:, F_RF_z_idxs) = -mu*[eye(n); eye(n)]; 
h_ineq_fc_rf = zeros(2*n, 1); 

% right hind 
G_ineq_fc_rh = sparse([], [], [], 2*n, dim_x, 4*n); 
G_ineq_fc_rh(1:n, F_RH_x_idxs) = eye(n); 
G_ineq_fc_rh(n+1:end, F_RH_x_idxs) = -eye(n); 
G_ineq_fc_rh(:, F_RH_z_idxs) = -mu*[eye(n); eye(n)]; 
h_ineq_fc_rh = zeros(2*n, 1); 

% left front 
G_ineq_fc_lf = sparse([], [], [], 2*n, dim_x, 4*n); 
G_ineq_fc_lf(1:n, F_LF_x_idxs) = eye(n); 
G_ineq_fc_lf(n+1:end, F_LF_x_idxs) = -eye(n); 
G_ineq_fc_lf(:, F_LF_z_idxs) = -mu*[eye(n); eye(n)];
h_ineq_fc_lf = zeros(2*n, 1); 

% left hind 
G_ineq_fc_lh = sparse([], [], [], 2*n, dim_x, 4*n); 
G_ineq_fc_lh(1:n, F_LH_x_idxs) = eye(n); 
G_ineq_fc_lh(n+1:end, F_LH_x_idxs) = -eye(n); 
G_ineq_fc_lh(:, F_LH_z_idxs) = -mu*[eye(n); eye(n)];
h_ineq_fc_lh = zeros(2*n, 1); 



%
%
%       With knee at right angle and shoulder
%       at 225 degress, want to be able to produce 
%       ground reaction force -- 
%       but then there are acceleations too 
%       so make a second augmented trajectory 
%       ... also might want it at a lower angle because would jump 
%       with more bent knees       
%








% combine 
G_ineq_fc = [G_ineq_fc_rf; G_ineq_fc_rh; G_ineq_fc_lf; G_ineq_fc_lh];
h_ineq_fc = [h_ineq_fc_rf; h_ineq_fc_rh; h_ineq_fc_lf; h_ineq_fc_lh];

%
%
%       Account for recharge inefficiency 
%
%
Psi = 0.85; % regen efficincy Psi 

G_ineq_power = sparse([], [], [], 2*n_walk, dim_x, 2*(N_motors + 1));

G_ineq_power(1:n_walk, p_rfs_idxs) = eye(n_walk); 
G_ineq_power(1:n_walk, p_rfk_idxs) = eye(n_walk); 
G_ineq_power(1:n_walk, p_rhs_idxs) = eye(n_walk); 
G_ineq_power(1:n_walk, p_rhk_idxs) = eye(n_walk); 
G_ineq_power(1:n_walk, p_lfs_idxs) = eye(n_walk); 
G_ineq_power(1:n_walk, p_lfk_idxs) = eye(n_walk); 
G_ineq_power(1:n_walk, p_lhs_idxs) = eye(n_walk); 
G_ineq_power(1:n_walk, p_lhk_idxs) = eye(n_walk); 
G_ineq_power(1:n_walk, p_total_idxs) = -eye(n_walk); 

G_ineq_power(n_walk + (1:n_walk), p_rfs_idxs) = Psi*eye(n_walk); 
G_ineq_power(n_walk + (1:n_walk), p_rfk_idxs) = Psi*eye(n_walk); 
G_ineq_power(n_walk + (1:n_walk), p_rhs_idxs) = Psi*eye(n_walk); 
G_ineq_power(n_walk + (1:n_walk), p_rhk_idxs) = Psi*eye(n_walk); 
G_ineq_power(n_walk + (1:n_walk), p_lfs_idxs) = Psi*eye(n_walk); 
G_ineq_power(n_walk + (1:n_walk), p_lfk_idxs) = Psi*eye(n_walk); 
G_ineq_power(n_walk + (1:n_walk), p_lhs_idxs) = Psi*eye(n_walk); 
G_ineq_power(n_walk + (1:n_walk), p_lhk_idxs) = Psi*eye(n_walk); 
G_ineq_power(n_walk + (1:n_walk), p_total_idxs) = -eye(n_walk); 
h_ineq_power = zeros(2*n_walk, 1); 


% Combine into general inequality matrix 
G_ineq = [G_ineq_normal; G_ineq_fc; G_ineq_power];
h_ineq = [h_ineq_normal; h_ineq_fc; h_ineq_power]; 

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
tau_c = [d_rfs; d_rfk; d_rhs; d_rhk; d_lfs; d_lfk; d_lhs; d_lhk];


%-------------------------------------------------------
%
%     Power Consumption - In Constraints 
%
%----------------------------------------------------------



% Standard (SOCP) objective Terms all empty because linear fractional
%Q0 = []; 
%c0 = []; 
%M0 = [];
%r0 = []; 
%beta0 = [];
Q = {};
c = {};
M = {};
r = {};
bet = {}; % beta

% Drive board efficiency Phi
Phi = 0.98; % driver board efficiency
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

    for j = 1:n_walk % loop through time points of gait  
        idx = (motor - 1)*n + j; % corresponding index in omega_full
        e_j = zeros(N_motors*n, 1);  
        % pick out this motor and time point
        e_j(idx) = 1; % one hot vector / unit basis vector 
        %spdg_j =  sparse(diag(e_j));  

        r_motor_j = zeros(dim_x, 1);
        r_motor_j(p_idxs(j)) = -1; 
        
        %Q{end + 1, 1} = @(motor, ~) Phi_inv*motor.R * spdg_j; 
        Q{end + 1, 1} = @(motor, ~) Phi_inv*motor.R * e_j; 
        c{end + 1, 1} = @(motor, gearbox) Phi_inv*motor.k_t*gearbox.direction*gearbox.ratio*omega(j)*e_j;  % TODO -- k_e 
        M{end + 1, 1} = []; 
        r{end + 1, 1} = r_motor_j;  % accounting for the other vars 
        bet{end + 1, 1} = 0; 

        % If negative MOTOR power consumption
        %Q{end + 1, 1} = @(motor, ~) Phi*motor.R * spdg_j; 
        Q{end + 1, 1} = @(motor, ~) Phi*motor.R * e_j; 
        c{end + 1, 1} = @(motor, gearbox) Phi*motor.k_t*gearbox.direction*gearbox.ratio*omega(j)*e_j;     % TODO-- k_e 
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




% TODO -- change to simple limits (and move the following line to be internal comp)


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

%
%
%   One way to think about this is to minimize gait_cycle_energy/battery_enrgy
%   because then then the bounds would be 0 for inifinite run time and 1 
%   for runtime of 1 step. However, objectives very close to zero could 
%   lead to numerical issues 
%{
r_num = zeros(dim_x, 1); 
r_num(e_gc_idx) = 1; 
beta_num = 0;

r_den = zeros(dim_x, 1);
r_den(m_b_idx) = rho_batt; 
beta_den = 0; 

cost_lb = 0;
cost_ub = 1; 
%} 

% OR 
% minmize negative number of steps 
% minimize -battery_energy/gait_cycle_energy
%
%  upper bound -1 (1 step)
%  lower bound assume the robot can walk for a whole week 
steps_per_sec = 1/t_gait(end);
week_of_steps = steps_per_sec*60*60*24*7;

r_num = zeros(dim_x, 1); 
r_num(m_b_idx) = -rho_batt; 
beta_num = 0;

r_den = zeros(dim_x, 1);
r_den(e_gc_idx) = 1; 
beta_den = 0; 


cost_lb = -week_of_steps; 
cost_ub = -1; 



% Block comment at the begining of each of these would be good 

%
%
%       Call the solver!
%
%



%

% SOMETHING SEEMS NOT RIGHT HERE 7.63 * 10^-6 too low 
% thought it was  somewhere around 300
% is this all from inertial?????
% --- even without inertial --- so maybe something up with problem setup
% need to investigate (investigat wiht inertia turned off )


% Assign all to the same struct 
problem_data.T = T;
problem_data.tau_c = tau_c;

problem_data.Q = Q;
problem_data.c = c; 
problem_data.M = M; 
problem_data.r = r; 

problem_data.beta = bet; 
problem_data.G = G;    
problem_data.h = h; 
problem_data.G_ineq = G_ineq;
problem_data.h_ineq = h_ineq;

problem_data.omega = omega_full;  % all concat 
problem_data.omega_dot = omega_dot_full; % TODO for inertial compensation
problem_data.I_max = I_max; 
problem_data.V_max = V_max; 

%% Inputs specific to linear-fractional problems 
problem_data.cost_lb = cost_lb; 
problem_data.cost_ub = cost_ub; 
problem_data.r_num = r_num;
problem_data.r_den = r_den; 
problem_data.beta_num = beta_num;
problem_data.beta_den = beta_den; 



robot_dims.l1 = l1;
robot_dims.l2 = l2;
robot_dims.l_body = l_body; 

q = struct('theta', num2cell(theta_com), 'x', num2cell(x_com),...
                 'z', num2cell(z_com), ...
                'theta_rfs', num2cell(theta_rfs),...
                'theta_rfk', num2cell(theta_rfk),...
                'theta_rhs', num2cell(theta_rhs),...
                'theta_rhk', num2cell(theta_rhk),...
                'theta_lfs', num2cell(theta_lfs),...
                'theta_lfk', num2cell(theta_lfk),...
                'theta_lhs', num2cell(theta_lhs),...
                'theta_lhk', num2cell(theta_lhk) );


%{
ax = axes; 
axis equal;
zlim([-0.1, 0.8]);
xlabel('X');
zlabel('Z');
view(0,0);
ax = initialize_robot(ax, robot_dims, q(1));
dt = t_gait(2) - t_gait(1); 
for j = 1:10
    for i = 1:length(t_gait)
        ax = update_robot(ax, robot_dims, q(i)); 

        pause(dt); drawnow; 
    end 
end 
%} 

% Solve the problem 

%settings.solver = 'ecos';
settings.solver = 'gurobi';

prob = cords('settings', settings); % instantiate new motor selection problem 

prob.update_problem(problem_data);   % update with the data 

%{
prob.filter_motors 
prob.filter_gears 

% consider whats the best way to do this 
prob.hints = % HOW BEST TO DO THIS? 
%} 

% Need to think about what return types should be 
% What inputs would make sense????? 
num_solutions = 2; % number of best combinations to return 
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





%
%
%       Robot Plot things 
% 
%
%

% Initialize Robot 




% Update Robot 


% NOTE on cords 

function [ax] = initialize_robot(ax, robot_dims, q)

    % Initial state vectors 
    rd = robot_dims;

    % Transform at COM for Robot 
    robot_transform = hgtransform('tag', 'robot_transform', 'Parent', ax);

    robot_transform.Matrix = makehgtform('translate', [q.x, 0, q.z], 'yrotate', q.theta); 

    %robot = hggroup('tag', 'robot', 'Parent', 'robot_transform');
    %{
    robot_body = rectangle('Position', rd.l_body*[-0.5, -0.05, 1, 0.1],...
                                  'Curvature', [0.6, 0.6], 'tag', 'body',...
                                         'Parent', robot_transform);
    %} 
    robot_body = line([-rd.l_body/2, rd.l_body/2], [0, 0], [0, 0],...
                             'Parent', robot_transform);


    %% Add the ground (gray patch OR plane )

    %
    %       Right Front 
    %
    rf_thigh_tf = hgtransform('tag', 'rf_thigh_tf', 'Parent', robot_transform);
    rf_thigh_tf.Matrix = makehgtform('translate', [rd.l_body/2, 0, 0,],...
                                         'yrotate', -q.theta_rfs);
    rf_shank_tf = hgtransform('tag', 'rf_shank_tf', 'Parent', rf_thigh_tf);
    rf_shank_tf.Matrix = makehgtform('translate', [rd.l1, 0, 0],...
                     'yrotate', -(q.theta_rfk - q.theta_rfs)); 
    rf_thigh = line([0, rd.l1], [0, 0], [0, 0], 'Parent', rf_thigh_tf, 'Color', [0 1 0]);
    rf_shank = line([0, rd.l2], [0, 0], [0, 0], 'Parent', rf_shank_tf, 'Color', [1 0 0]);

    ff = plot3(0, 0, 0, 'k.', 'Parent', rf_shank_tf); % add a foot too 
    view(0,0)
    % RFK (Right Front Knee)

    %
    %    Right Hind
    %
    rh_thigh_tf = hgtransform('tag', 'rh_thigh_tf', 'Parent', robot_transform);
    rh_thigh_tf.Matrix = makehgtform('translate', [-rd.l_body/2, 0, 0,],...
                                         'yrotate', -q.theta_rhs);
    rh_shank_tf = hgtransform('tag', 'rh_shank_tf', 'Parent', rh_thigh_tf);
    rh_shank_tf.Matrix = makehgtform('translate', [rd.l1, 0, 0],...
                     'yrotate', -(q.theta_rhk - q.theta_rhs)); 
    rh_thigh = line([0, rd.l1], [0, 0], [0, 0], 'Parent', rh_thigh_tf, 'Color', [0 1 0]);
    rh_shank = line([0, rd.l2], [0, 0], [0, 0], 'Parent', rh_shank_tf, 'Color', [1 0 0]);



    %
    %       Left Front 
    %
    lf_thigh_tf = hgtransform('tag', 'lf_thigh_tf', 'Parent', robot_transform);
    lf_thigh_tf.Matrix = makehgtform('translate', [rd.l_body/2, 0, 0,],...
                                         'yrotate', -q.theta_lfs);
    lf_shank_tf = hgtransform('tag', 'lf_shank_tf', 'Parent', lf_thigh_tf);
    lf_shank_tf.Matrix = makehgtform('translate', [rd.l1, 0, 0],...
                     'yrotate', -(q.theta_lfk - q.theta_lfs)); 
    lf_thigh = line([0, rd.l1], [0, 0], [0, 0], 'Parent', lf_thigh_tf, 'Color', [0 1 0]);
    lf_shank = line([0, rd.l2], [0, 0], [0, 0], 'Parent', lf_shank_tf, 'Color', [1 0 0]);


    %
    %    Left Hind
    %
    lh_thigh_tf = hgtransform('tag', 'lh_thigh_tf', 'Parent', robot_transform);
    lh_thigh_tf.Matrix = makehgtform('translate', [-rd.l_body/2, 0, 0,],...
                                         'yrotate', -q.theta_lhs);
    lh_shank_tf = hgtransform('tag', 'lh_shank_tf', 'Parent', lh_thigh_tf);
    lh_shank_tf.Matrix = makehgtform('translate', [rd.l1, 0, 0],...
                     'yrotate', -(q.theta_lhk - q.theta_lhs)); 
    lh_thigh = line([0, rd.l1], [0, 0], [0, 0], 'Parent', lh_thigh_tf, 'Color', [0 1 0]);
    lh_shank = line([0, rd.l2], [0, 0], [0, 0], 'Parent', lh_shank_tf, 'Color', [1 0 0]);
end 

function [ax] = update_robot(ax, robot_dims, q)

    robot_transform = findobj(ax.Children, 'tag', 'robot_transform');
    robot_transform.Matrix = makehgtform('translate', [q.x, 0, q.z], 'yrotate', q.theta); 
    

    % Right Front 
    rf_thigh_tf = findobj(robot_transform.Children, 'tag', 'rf_thigh_tf');
    rf_shank_tf = findobj(rf_thigh_tf.Children, 'tag', 'rf_shank_tf');
    rf_thigh_tf.Matrix(1:3, 1:3) = roty(-rad2deg(q.theta_rfs));
    rf_shank_tf.Matrix(1:3, 1:3) = roty(-rad2deg(q.theta_rfk - q.theta_rfs));

    % Right Hind 
    rh_thigh_tf = findobj(robot_transform.Children, 'tag', 'rh_thigh_tf');
    rh_shank_tf = findobj(rh_thigh_tf.Children, 'tag', 'rh_shank_tf');
    rh_thigh_tf.Matrix(1:3, 1:3) = roty(-rad2deg(q.theta_rhs));
    rh_shank_tf.Matrix(1:3, 1:3) = roty(-rad2deg(q.theta_rhk - q.theta_rhs));


    % Left Front 
    lf_thigh_tf = findobj(robot_transform.Children, 'tag', 'lf_thigh_tf');
    lf_shank_tf = findobj(lf_thigh_tf.Children, 'tag', 'lf_shank_tf');
    lf_thigh_tf.Matrix(1:3, 1:3) = roty(-rad2deg(q.theta_lfs));
    lf_shank_tf.Matrix(1:3, 1:3) = roty(-rad2deg(q.theta_lfk - q.theta_lfs));

    % Left Hind 
    lh_thigh_tf = findobj(robot_transform.Children, 'tag', 'lh_thigh_tf');
    lh_shank_tf = findobj(lh_thigh_tf.Children, 'tag', 'lh_shank_tf');
    lh_thigh_tf.Matrix(1:3, 1:3) = roty(-rad2deg(q.theta_lhs));
    lh_shank_tf.Matrix(1:3, 1:3) = roty(-rad2deg(q.theta_lhk - q.theta_lhs));
end 


%{
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

%} 