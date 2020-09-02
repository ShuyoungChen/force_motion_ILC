function [ex,ey,ez,n,P,q,H,type] = robotParams()

I3 = eye(3);
ex = I3(:,1);ey = I3(:,2);ez = I3(:,3);
h1 = ez; h2 = ey; h3 = ey; h4 = ex; h5 = ey; h6 = ex;
P = [0,0,0;0.32, 0, 0.78;0, 0, 1.135;1.1825, 0, 0.2;0, 0, 0;0, 0, 0;0.2,0,0]';
q = [0 0 0 0 0 0]';
H = [h1 h2 h3 h4 h5 h6];
type = [0 0 0 0 0 0];
n = 6;
end