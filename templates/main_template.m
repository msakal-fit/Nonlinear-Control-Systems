clear all 
close all 
clc 

%% constants 

params.m = 5; % example [kg]

%% Initial Conditions

X0 = []; % your initial state vector [m, m/s]

%% integrate EoMs 

TSPAN = [0, 30]; % initial and final times [s]
t_step = 0.1; % simulation time step [s]

[TOUT, XOUT, logs] = RK4(@dynamics_template, TSPAN, X0, t_step, params); % propagate using RK4

%% (optional) extract logs & plot 

saved_data = logs.data2save; % example
plot(TOUT, saved_data)


%% Animate quadcopter 

pos = XOUT(..); % position data from simulation, must be Nx3 (N: # of samples) [m]
L = 0.5 ; % quadcopter arm length (for visualization purposes) [m]
pos_d = [];  % desired position, must be Nx3 (N: # of samples), if empty, set to [m]
speed = 2; % tune animation speed 

animate_quadcopter(pos, L, pos_d, TOUT, speed);

