%%% 
% Description: Generate the system model and expressions.
%              Move these to appropriate folders / gen sfun
%
% Author: Jenna Reher, jreher@caltech.edu
%
% Date: March 25, 2019
% ________________________________________
% Generate path to MATLAB description
frost_addpath;
addpath(genpath('MATLAB'));

% Load the robot's articulated tree from URDF
is_rigid = false;
is_planar = false;
robot = Cassie_v4('urdf/cassie_v4.urdf', is_planar, is_rigid); % Model including compliant spring elements

% Use the internal functions to generate files
robot.exportAll;