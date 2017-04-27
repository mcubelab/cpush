%Setup external functions, libraries, and directories

% PATH
addpath('../software/externals/jsonlab-1.0');
addpath(genpath('System'))
%add gurobi path
if ismac()
    run('/Users/Francois/Dropbox (MIT)/Matlab/MIT/Drake/drake/addpath_drake');
    addpath('/Library/gurobi650/mac64/matlab');
    gurobi_setup;
elseif isunix() && ~ismac()
    run('/home/mcube10/software/drake-v0.9.11-linux/drake/addpath_drake');
    addpath('/home/mcube10/software/gurobi702/linux64/matlab');
    gurobi_setup;
elseif ispc()
    addpath('C:\gurobi701\win64\matlab');
end