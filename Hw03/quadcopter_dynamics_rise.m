% file: quadcopter_dynamics_rise.m

function [X_dot, logs] = quadcopter_dynamics_rise(t, X, params)
% Quadcopter dynamics with RISE controller

x = X(1:3); % position
v = X(4:6); % velocity
I_mu = X(7:9); % RISE integral state

% ------ constants
m = params.m; Kd = params.Kd; g = params.g(:);

u     = params.u(t,X);                 % thrust in N, z-up

if isfield(params,'d_fun') && ~isempty(params.d_fun)
    d = params.d_fun(t);
else
    d = zeros(3,1);
end

% ------ Equations of Motion 
speed = norm(v);
drag = Kd*speed*v; % drag force

% dynamics
accel  = g + (u - drag + d)/m; % xddot = g + (1/m)u - (Kd/m)||v||v

% ----- RISE integral ODE: d/dt(mu) = (ks+1)alpha2*e2 + beta*sign(e2)
xd = params.xd_fun(t);
vd = params.vd_fun(t);
e1 = xd - x;
de1 = vd - v;
e2 = de1 + params.ctrl.alpha1 * e1;

I_mu_dot = (params.ctrl.ks + 1) * params.ctrl.alpha2*e2 ...
            + params.ctrl.beta * sign(e2);

X_dot = [v; accel; I_mu_dot]; % state derivative

% ------ logs
logs = struct();
logs.u      = u;          % thrust
logs.drag_N = norm(drag); % drag magnitude
logs.speed  = speed;      % speed
logs.accel  = accel(:);   % acceleration
logs.d      = d(:);       % disturbance
logs.e1      = e1(:);
logs.e2      = e2(:);
logs.I_mu    = I_mu(:);

end
