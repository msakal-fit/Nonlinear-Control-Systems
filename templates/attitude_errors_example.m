function [X_dot, logs] = dynamics_template(t, X, params)
% --------------------------------------------------------%
% Inputs: 
% t: time (scalar)
% X: state (n by 1 array, n:# of states)
% params: external parameters (structure)

% Outputs: 
% X_dot: state derivative (n by 1 array, n:# of states)
% logs: results to save (structure)

% --------------------------------------------------------%

%% declare persistent variables 

persistent phi_d_prev theta_d_prev psi_d_prev t_prev


%% declare constants

m = params.m; % example


%%  desired trajectories (translational)

 ...

%% tracking errors (translational)

...


%% Control law (translational)

u = ...

%% desired trajectories (attitude)

psi_d = 0; 
[phi_d, theta_d] = angles_from_force(u, psi_d);

euler_angles_d = [phi_d,theta_d,psi_d]'; % desired attitude angles

if isempty(phi_d_prev)
    phi_d_prev = phi_d;
    theta_d_prev = theta_d;
    psi_d_prev = psi_d;
    t_prev = 0;
end 

%% tracking errors (attitude)

phi = % from your state vector
theta = % from your state vector 
psi = % from your state vector
omega = % from your state vector

dt = %% your simulation time step 


[sigma, omega_tilde, sigma_dot, omega_tilde_dot, omega_d, omega_d_dot] = ...
    compute_error_states_and_derivatives(phi, theta, psi, omega, ...
                                       phi_d, theta_d, psi_d, ...
                                       phi_d_prev, theta_d_prev, psi_d_prev, ...
                                       dt); % attitude error states

%% control law (attitude)

tau = ... 

%% Equations of Motion (translation + attitude)

X = ... 

%% Logs 

data2save = sin(t); % example
logs.data2save = data2save; % example


%% update variables for next step 

if t > t_prev
    phi_d_prev = phi_d;
    theta_d_prev = theta_d;
    psi_d_prev = psi_d;
    t_prev = t;
end

end
