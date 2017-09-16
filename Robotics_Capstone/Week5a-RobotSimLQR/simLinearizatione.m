
params = struct();

params.g = 9.81;
params.mr = 0.25;
params.ir = 0.0001;
params.d = 0.1;
params.r = 0.02;
qdd = eom(params,th,phi,dth,dphi,u);
% 1. Learn how to use the symbolic toolbox in MATLAB
% you will need the state variables, and control input to be declared as symbolic
f = [dth;dphi;qdd];
res = diff(f,th);
A = jacobian(f,[th,phi,dth,dphi])
b = jacobian(f, u)
% 2. call your "eom" function to get \ddot{q} symbolically
A_l = double((subs(A,{th,phi,dth,dphi,u},{0,0,0,0,0})))
% 0.6324    0.0975    0.2785    0.5469
b_l = double((subs(b, {u,phi}, {0,0})))
% 3. Linearize the system at 0 (as shown in lecture)
% You should end up with A (4x4), and b (4x1)
Co = ctrb(A_l,b_l)
unco = length(A_l) - rank(Co)
k = lqr(A_l,b_l,0.08*eye(4),1000,0)
% 4. Check that (A,b) is  controllable
% Number of uncontrollable states should return 0

% 5. Use LQR to get K as shown in the lecture
