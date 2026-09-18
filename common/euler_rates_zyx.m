function eadot = euler_rates_zyx(phi,theta,~,omega)

% omega = [p;q;r] (body rates)
% p = omega(1); q = omega(2); r = omega(3);

ct = cos(theta); st = sin(theta); sp = sin(phi); cp = cos(phi);

E = (1/ct) * [  ct,     st*sp,   st*cp;
                0,      ct*cp,  -ct*sp;
                0,         sp,       cp ];
eadot = E * omega;

end