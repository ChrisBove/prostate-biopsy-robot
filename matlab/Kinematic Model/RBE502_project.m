dPhi = 0.001;
v = 0.5;
theta = pi/6;
iterations = 100;
t = 1:iterations;

x = zeros(length(t),1);
y = zeros(length(t),1);
z = zeros(length(t),1);
phi_yx = zeros(length(t),1);
phi_yz = zeros(length(t),1);

%initial conditions
x(1,1)= 0;
y(1,1)= 0;
z(1,1)= 0;
phi_yx(1,1)= 0;
phi_yz(1,1)= 0;

%Non linear system
for i = 2:iterations
    x(i,1)= x(i-1,1) + sin(phi_yx(i-1,1))*v*i;
    z(i,1)= z(i-1,1) + sin(phi_yz(i-1,1))*v*i;
    y(i,1)= y(i-1,1) + cos((phi_yx(i-1,1)))*cos((phi_yz(i-1,1)))*v*i;
    phi_yx(i,1)= phi_yx(i-1,1) + cos(theta*i)*dPhi;
    phi_yz(i,1)= phi_yz(i-1,1) + sin(theta*i)*dPhi;
end

plot3(x,z,y);

%Linearized System
for i = 2:iterations
    x(i,1)= x(i-1,1) + (phi_yx(i-1,1))*v*i;
    z(i,1)= z(i-1,1) + (phi_yz(i-1,1))*v*i;
    y(i,1)= y(i-1,1) + v*i;
    phi_yx(i,1)= phi_yx(i-1,1) + dPhi;
    phi_yz(i,1)= phi_yz(i-1,1) + theta*dPhi;
end

plot3(x,z,y,'LineWidth',5);
title('Needle Kinematic Model - theta=pi/6');

%% Webster Non-Holonomic model

%Metric Units

clear all;
%syms u1 u2 

u1 = 0.005;
u2 = 0.004;

l1  = 0.04; %Distance B to C
phi = deg2rad(10.18);
l2 = 0.023775;
k = tan(phi)/l1;
e1 = [1; 0; 0];
e3 = [0; 0; 1];
V1 = [e3;k*e1];
V2 = [0;0;0;e3];
V1_hat = [isomorphic(V1(4:6)),V1(1:3);0,0,0,0];
V2_hat = [isomorphic(V2(4:6)),V2(1:3);0,0,0,0];
T = 0.1; % seconds per step

g_ab = [1 0 0 0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

n = zeros(3,1);

for i = 1:300
    g_ab(:,:,i+1) = g_ab(:,:,i)*expm((u1*V1_hat + u2*i*V2_hat)*T);
    n(:,i) = g_ab(1:3,1:3,i)*l2*e3 + g_ab(1:3,4,i);
    
end

plot3(n(3,:),n(1,:),n(2,:),'LineWidth',5);
axis([0 0.2 -0.02 0.02 -0.02 0.02]);
title('Needle Kinematic Model');
daspect([1 1 1]);