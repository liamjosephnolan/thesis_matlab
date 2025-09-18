%% Matrice Comparsion Script
% Comparison script for sanity checking the state space matrices from Liam
% Nolans incredible thesis. Matrices were estimated from a roll step
% response at 0,0 using 2nd order transfer fucntion

clear all 
close all
clc

% The estimated state space matrices from ssest() matlab documentation
% states this is numerically optimized state space matrix format
A_no = [-6.1202 -4.0720; 4.000 0];
B_no = [2;0];
C_no = [2.0213 -1.0414];
D_no = 0;

% Create the state-space object
no = ss(A_no, B_no, C_no, D_no);

% Convert to canonical companion format
ocf = canon(no, 'companion');

