% file: hw31_main.m

clear; close all; clc;

%% -------------------- Parameters --------------------
params.m  = 5;                         % [kg]
params.Kd = 0.03;                      % [kg/m]
params.g  = [0; 0; -9.81];             % [m/s^2]
Omega     = 0.2*pi;                    % [rad/s]

% ----- Desired trajectory xd, vd, ad -----
xd_fun = @(t) [ 0; 6*sin(Omega*t + pi/4); 10 ];

vd_fun = @(t) [ 0; 6*Omega*cos(Omega*t + pi/4); 0 ];

ad_fun = @(t) [ 0; -6*Omega^2*sin(Omega*t + pi/4); 0 ];

% Save xd, vd for e2 computation in controller
params.xd_fun = xd_fun;
params.vd_fun = vd_fun;

% ----- RISE gains -----
ctrl.alpha1 = diag([1.5 1.5 3.0]); % chose > 1/2
ctrl.alpha2 = diag([1.0 1.0 2.0]); % chose > 1/2
ctrl.ks     = 1.0;                 % chose > 0
ctrl.beta   = 1.5;                 % beta > d + d_dot/||alpha2|| (d, d_dot are disturbance bounds)

% initialize e2 for mu(t) formula
x0 = [0;0;2];
v0 = [0;0;0];
t0 = 0;
e1_0 = xd_fun(t0) - x0;
e2_0 = vd_fun(t0) - v0 + ctrl.alpha1*e1_0;
ctrl.e2_0 = e2_0;

params.ctrl = ctrl;

% ----- disturbance function
params.d_fun = @(t) [-0.2*sin(t); 0.1*sin(0.5*t); 0];

% ----- Controller handle (feedback linearization) -----
params.u = @(t,X) rise_tracking_u(t,X,xd_fun,vd_fun,ad_fun,ctrl,params);


%% -------------------- ICs and integration --------------------
I_mu0 = zeros(3,1);     % RISE integral state at 0
X0 = [x0; v0; I_mu0];   % initial state

TSPAN  = [0 30];
t_step = 0.01;          % smaller step for integration

[TOUT, XOUT, logs] = RK4(@quadcopter_dynamics_rise, TSPAN, X0, t_step, params);

%% -------------------- Build pos_d for animation --------------------
N = numel(TOUT);
pos_d = zeros(N,3);
for k = 1:N
    pos_d(k,:) = xd_fun(TOUT(k)).';
end

%% -------------------- Plots --------------------
pos = XOUT(:,1:3);
vel = XOUT(:,4:6);
spd = vecnorm(vel,2,2);

U       = cat(2, logs.u);         % 3×N (each logs(k).u is 3×1)
u_mag   = vecnorm(U,2,1).';       % N×1
drag_mag= [logs.drag_N].';        % N×1

% ------------------- Desired attitude angles from thrust vector -------------------
% part (e) and (f)
psi_d = 0; 
N     = numel(TOUT);
phi_d = zeros(N,1);
theta_d = zeros(N,1);
psi_log = psi_d* ones(N,1);

% U is 3*N 
% load and map force -> (phi, theta) at each time
for k = 1:N
    uk = U(:,k); % thrust vector in N
    % angles_from_force expects desired thrust direction & yaw angle
    [phi_d(k), theta_d(k)] = angles_from_force(uk, psi_d);
end

% desired Euler angles for animation
EAs_d = [phi_d, theta_d, psi_log]; % [roll, pitch, yaw]

figure('Name','Desired Attitude Angles (rad)');
plot(TOUT, phi_d, 'LineWidth', 2); grid on; hold on;
plot(TOUT, theta_d, 'LineWidth', 2);
plot(TOUT, psi_log, 'LineWidth', 2);
xlabel('t [s]'); ylabel('angle [rad]');
legend('\phi_d (roll)','\theta_d (pitch)','\psi_d (yaw)','Location','best');
title('Desired Attitude Angles from Thrust Direction (rad)');

% plot in degrees
phi_deg   = rad2deg(phi_d);
theta_deg = rad2deg(theta_d);
psi_deg   = rad2deg(psi_log);

figure('Name','Desired Attitude Angles (deg)');
plot(TOUT, phi_deg, 'LineWidth', 1.2); grid on; hold on;
plot(TOUT, theta_deg, 'LineWidth', 1.2);
plot(TOUT, psi_deg, 'LineWidth', 1.2);
xlabel('t [s]'); ylabel('angle [deg]');
legend('\phi_d (roll)','\theta_d (pitch)','\psi_d (yaw)','Location','best');
title('Desired Attitude Angles from Thrust Direction (deg)');

figure('Name','Thrust magnitude');
plot(TOUT, u_mag); grid on; xlabel('t [s]'); ylabel('|u| [N]');
title('Thrust magnitude (RISE)');

figure('Name','Control Input Components');
plot(TOUT, U(1,:),'LineWidth', 2); grid on; hold on;
title('Control Input Components');
plot(TOUT, U(2,:),'LineWidth', 2); grid on
ylabel('u [N]');
plot(TOUT, U(3,:),'LineWidth', 2); grid on
xlabel('t [s]'); 
legend('u_x','u_y','u_z');

% compute and plot tracking error
err = pos - pos_d;
err_norm = vecnorm(err,2,2);
figure('Name','Position Tracking Error');
plot(TOUT, err_norm); grid on;
xlabel('t [s]'); ylabel('||e|| [m]');
title('Position Tracking Error Norm');

%% -------------------- Animate --------------------
L = 0.5;              % arm length for viz [m]
anim_speed = 2;       % animation rate
%animate_quadcopter(pos, L, pos_d, TOUT, anim_speed);
animate_quadcopter(pos, L, [pos_d EAs_d], TOUT, anim_speed);
