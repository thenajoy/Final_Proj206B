% prms_gen.m
% by Guofan Wu(gwu)
% Generate parameters for the 3d-moving quadrotor
% sys_prms: parameters of the system's mass properties 
% ref_prms: parameters of the reference
% All the units are in SI 
clear all;

% system parameters & input saturation
sys_prms.m = 0.520;  % total mass of this quadrotor 
sys_prms.width = 0.25;
sys_prms.g = 9.83;
sys_prms.J = diag([2.32e-3, 2.32e-3, 7.60e-3]); % inertia matrix in principal axis 
sys_prms.Fmax = 1e2;
sys_prms.Mmax  = 5e2 * ones(3, 1); 

% reference parameters 
% position reference: 
ref_prms.A = [0; 0; 0];
ref_prms.omega = [1.0; 1.0; 1.5];
ref_prms.phi0 = [pi/2; 0; 0];
ref_prms.pos_a0 = [0; 0; 1.5];
ref_prms.pos_a1 = [0; 0.75; 0];
ref_prms.pos_a2 = [0; 0; 0];

% yaw angle reference: 
ref_prms.psi_a0 = pi/6;
ref_prms.psi_a1 = 0;
ref_prms.psi_a2 = 0;


% save to file
save('prms.mat');

