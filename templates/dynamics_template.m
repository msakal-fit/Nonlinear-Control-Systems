function [X_dot, logs] = quadcopter_dynamics(t, X, params)
% --------------------------------------------------------%
% Inputs: 
% t: time (scalar)
% X: state (n by 1 array, n:# of states)
% params: external parameters (structure)

% Outputs: 
% X_dot: state derivative (n by 1 array, n:# of states)
% logs: results to save (structure)

% --------------------------------------------------------%

%% declare constants

m = params.m; % example


%% Equations of Motion 


X_dot = []; % your system's dynamics (in the form X_dot = f(X,t))

%% Logs 

logs.data2save = sin(t); % example


end
