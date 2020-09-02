% Initialize Robot IRB6700 parameters
[ex,ey,ez,~,P,q,H,type] = robotParams();

% position of obstacles in z (m)
pos_obj = 1.30142;

Kp = 0.0003;

% spring stiffness in N/m
Ks = 750000; 

% desired force in z (N)
Fdz = 1000;

% desired motion in y (m/s)
vdy = 0.006;

% sampling rate (s)
dt = 0.004;

% time vector
t = 0:dt:5;

% number of samples
siz = size(t);
n = siz(2);

% desired output
desired = zeros(6, n);

% form the desired output by ud
for i = 1:n
    ud = [0,0,0,0,vdy,Fdz]';
    desired(:, i) = ud;
end

n_iter = 20;

alpha = 0.1;

a = 1;
%initial joint angles
q0 = [0,0,0,0,pi/2,0]';

% gradient type ILC
qin = force_motion_gradient(q0, desired, desired, n_iter, alpha, t, n, a, Ks, Kp, pos_obj, dt, H, P, type);

