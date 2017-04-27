classdef Simulation < dynamicprops & PusherSliderSystem
    properties (Constant)
        %Euler Integration
        num_states = 6;
        %Optimization Program
        h_opt = 0.03;
        steps = 50; 
        NumFam = 3;
        num_int = 1;
        num_vars = 4;
        num_xvars = 3;
        num_dvars = 1;
        num_inputs = 5;
    end
    
    properties
        t;
        xs;
        us;
        xc;
        uc;
        xs_state;
        us_state;
        xc_state;
        uc_state;
        uc_eq;
        xs_star_state;
        us_star_state;
        xc_star_state;
        uc_star_state;
        N;
        FileName;
        SimName;
        FilePath;
        NumSim;
        Ani;
    end
    
    
    methods
        
        %% Constructor
        function obj = Simulation(SimName)  
            %% Save data in new folder
            obj.SimName = SimName;
            if ismac()
                tempName = strcat('/Users/Francois/Dropbox (MIT)/Data/',obj.SimName);
            elseif isunix() && ~ismac()
                tempName = strcat('/home/mcube10/Data/cpush/',obj.SimName);
            end
            mkdir(tempName);
            obj.FilePath = tempName;
            obj.FileName = strcat(obj.FilePath,'/',obj.SimName);
            %Nominal Trajectory  #Frank hack
            %Initialize variables
            c = Controller();
            s = Simulator();
            obj.xs_state = zeros(obj.N, s.num_xsStates);
            obj.us_state = zeros(obj.N, s.num_usStates);
            obj.xc_state = zeros(obj.N, c.num_xcStates);
            obj.uc_state = zeros(obj.N, c.num_ucStates);
            obj.xs_star_state= zeros(obj.N, s.num_xsStates);
            obj.us_star_state= zeros(obj.N, s.num_usStates);
            obj.xc_star_state= zeros(obj.N, c.num_xcStates);
            obj.uc_star_state= zeros(obj.N, c.num_ucStates);

        end
    end
end