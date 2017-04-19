%% Author: Francois Hogan
%% Date: 12/08/2016
%% Clear
clear all;
close all;
clc;
%Setup externals
run('setup.m');

%% Simulation Parameters
t0 = 0;
tf = .02;
h_step = 0.01;
%% Build Simulation object
p = Simulation('linePusherSim'); 
c = Controller();

%% Simulation Parameters
p.NumSim = 1;

%% Initial conditions
xc_0{1} = [0.;0.05;15*pi/180;0]; 
xs_0{1} = c.coordinateTransformCS(xc_0{1});

%% Euler integration
lv1=1;
p.EulerIntegration( t0, tf, h_step, xs_0{1}, lv1);

%% Graphics
% p.Plot(0)
% p.Plot(1)
% p.Plot(2)
p.Animate(1)

%% Post-Processing
save(p.FileName, 'p');

