%Setup external functions, libraries, and directories
addpath('../software/externals/jsonlab-1.0');
run('/Users/Francois/Dropbox (MIT)/Matlab/MIT/Drake/drake/addpath_drake');
addpath(genpath('System'))
%add gurobi path
if ismac()
    addpath('/Library/gurobi650/mac64/matlab');
    gurobi_setup;
elseif isunix() && ~ismac()
    addpath(strcat(getenv('GUROBI_HOME'),'/matlab'));
    gurobi_setup;
elseif ispc()
    addpath('C:\gurobi701\win64\matlab');
end