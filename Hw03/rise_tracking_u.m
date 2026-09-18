% filename: rise_tracking_u.m

function u = rise_tracking_u(t, X, xd_fun, vd_fun, ad_fun, ctrl, params)
% RISE controller for quadcopter position tracking

% unpack state
x = X(1:3); % position
v = X(4:6); % velocity
I_mu = X(7:9); % RISE integral state

% reference trajectory
xd = xd_fun(t);
vd = vd_fun(t);
ad = ad_fun(t);

% errors
e1 = xd - x;
de1 = vd - v;
e2 = de1 + ctrl.alpha1 * e1;

% params
m = params.m;
Kd = params.Kd;
g = params.g(:);

% RISE mu
mu = (ctrl.ks+1)*e2 - (ctrl.ks+1)*ctrl.e2_0 + I_mu;

% drag
Fdrag = Kd * norm(v) * v;

% control law
u = m*ad - m*g + Fdrag + m*( ctrl.alpha1*de1 + ctrl.alpha2*e2 ) + mu;
end