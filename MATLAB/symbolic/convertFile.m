%% Function Name: convertFile
%
% Description: A function for calling the frostConv.py python script. This
% will generate a matlab function corresponding to raw c code.
%   
% Inputs:
%   name: String, name of the function to be converted
%   module: String, name of the function type
%
% Author: Jenna Reher, jreher@caltech.edu
%
% Date: Oct 3, 2018
% ________________________________________

function [] = convertFile(name, src_path, mat_path)

% Root location of the symbolic codegen
dir = fileparts(which('frostConv.py'));

% Source file path
src_file = [src_path, '/', name];

% Destination file path
export_file = [mat_path, '/', name];

% Run python
systemCommand = ['python ' dir '/frostConv.py ', src_file, ' ', export_file];
system(systemCommand);

end