% file: fbl_tracking_r_u.m

function u = fbl_tracking_r_u(t,X,xd_fun,vd_fun,ad_fun,ctrl,params)
% r := e_dot + alpha e controller for: m xdd + Kd||v||v = m g + u

% unpack
x  = X(1:3); 
v  = X(4:6);
xd = xd_fun(t); 
vd = vd_fun(t); 
ad = ad_fun(t);

% auxiliary errors
e   = x - xd; 
ed  = v - vd;
r   = ed + ctrl.alpha * e;             % r = e_dot + alpha e

% dynamics params
m  = params.m; 
Kd = params.Kd; 
g  = params.g(:);

% assume known drag 
Fdrag = Kd * norm(v) * v;

% ---- control law ----
% u = -m g + Kd||v||v + m ( ad - (alpha I + K) r )
%u = -m*g + Fdrag + m*( ad - (ctrl.alpha*eye(3) + ctrl.K) * r );

% u = -mg + + Kd||v||v + m(ad - alpha*edot - K*r - I*e) 
u = -m*g + Fdrag + m*(ad - ctrl.alpha*ed - ctrl.K * r - eye(3)*e);


end
