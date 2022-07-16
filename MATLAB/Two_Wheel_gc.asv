%%%%%%TWO WHEEL Gen Coord%%%%%
clc;
clear all;
close all;
%% Init
%define parameters
syms m_w m_l m_b r l_0 d Gamma_w Gamma_l Gamma_b

assume(m_b, 'real');
assume(m_w, 'real');
assume(m_l, 'real');

assume(d, 'real');
assume(l_0, 'real');
assume(r, 'real');

assume(Gamma_w, 'real');
assume(Gamma_l, 'real');
assume(Gamma_b, 'real');



%define u_dot parameter  
% syms v v_z omega_x omega_y omega_z p z theta_x theta_y theta_z 

syms t v(t) v_z(t) omega_x(t) omega_y(t) omega_z(t) p(t) z(t) theta_x(t) theta_y(t) theta_z(t) 

% assume(v,'real');
% assume(v_z,'real');
% 
% assume(omega_x,'real');
% assume(omega_y,'real');
% assume(omega_z,'real');
% 
% assume(z, 'real');
% assume(d,'real');
% 
% assume(theta_x,'real');
% assume(theta_y,'real');
% assume(theta_z,'real');

%% Dynamic
% u_dot = [v; v_z; omega_x; omega_y; omega_z];
% u = [p; z; theta_x; theta_y; theta_z];
% 
% v_R = v + omega_z*sqrt(d^2 + d*tan(theta_x)^2)/2;
% omega_WR = [omega_x; omega_y+v_R/r; omega_z];
% K_1 = 1/2 * m_w * v_R^2 + 1/2*omega_WR'*Gamma_w*omega_WR;
% 
% v_L = v - omega_z*sqrt(d^2 + d*tan(theta_x)^2)/2;
% omega_WL = [omega_x; omega_y+v_L/r; omega_z];
% K_2 = 1/2 * m_w * v_L^2 + 1/2*omega_WL'*Gamma_w*omega_WL;
% 
% omega = [omega_x; omega_y; omega_z];
% K_3 = 1/2 * m_l * v_R^2 + 1/2 * omega' * Gamma_l * omega;
% 
% omega = [omega_x; omega_y; omega_z];
% K_4 = 1/2 * m_l * v_L^2 + 1/2 * omega' * Gamma_l * omega;
% 
% p_B = [0 0 z]';
% v_B = sqrt(norm([v; 0; v_z] + cross(p_B,omega)));
% K_5 = 1/2 * m_b * v_B^2 + 1/2*omega' * Gamma_b * omega;
u = [p(t); z(t); theta_x(t); theta_y(t); theta_z(t)];
u_dot = [v(t); v_z(t); omega_x(t); omega_y(t); omega_z(t)];
u_ddot = arrayfun(@(ii) diff(u_dot(ii),t), 1:5);
u_ddot = conj(u_ddot');

v_R = v(t) + omega_z(t)*sqrt(d^2 + d*tan(theta_x(t))^2)/2;
omega_WR = [omega_x(t); omega_y(t)+v_R/r; omega_z(t)];
K_1 = 1/2 * m_w * v_R^2 + 1/2*omega_WR'*Gamma_w*omega_WR;

v_L = v(t) - omega_z(t)*sqrt(d^2 + d*tan(theta_x(t))^2)/2;
omega_WL = [omega_x(t); omega_y(t)+v_L/r; omega_z(t)];
K_2 = 1/2 * m_w * v_L^2 + 1/2*omega_WL'*Gamma_w*omega_WL;

omega = [omega_x(t); omega_y(t); omega_z(t)];
K_3 = 1/2 * m_l * v_R^2 + 1/2 * omega' * Gamma_l * omega;

omega = [omega_x(t); omega_y(t); omega_z(t)];
K_4 = 1/2 * m_l * v_L^2 + 1/2 * omega' * Gamma_l * omega;

p_B = [0 0 z(t)]';
v_B = sqrt(norm([v(t); 0; v_z(t)] + cross(p_B,omega)));
K_5 = 1/2 * m_b * v_B^2 + 1/2*omega' * Gamma_b * omega;

K = K_1 + K_2 + K_3 + K_4 + K_5;
U = l_0*sin(theta_x(t))*sin(theta_y(t)) * m_l * 9.81 + z * sin(theta_x(t)) * sin(theta_y(t)) * m_b * 9.81; 

K = simplify(K);
K = collect(K,u_dot);

L = K - U;
L = simplify(L);
L = collect(L,u_dot);

for n = 1:5
    dL_du_dot(n) = diff(L, u_dot(n));
    dL_du(n) = diff(L, u(n));
end
DE = diff(dL_du_dot,t);
for n = 1:4
    B(n,:) = coeffs(DE(n), u_ddot); 
end
tmp = coeffs(DE(5), u_ddot(5));
B(5,:) = [0 0 0 0 tmp(1,2)];
EL = simplify(diff(dL_du_dot,t) - dL_du);

% X = {p v z v_z theta_x omega_x theta_y omega_y theta_z omega_z };
% par = {m_w m_l m_b r l_0 d Gamma_w Gamma_l Gamma_b};
% Qi = {0 0 0 0 0};
% Qe = {0 0 0 0 0};
% R = 0;
% System = EulerLagrange(L,X,Qi,Qe,R,par);
% save('eq_sys.mat');


