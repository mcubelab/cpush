%% Euler-Integration
function obj = EulerIntegration(obj, t0, tf, h, xs0, SimulationCounter)
    %% Build objects
    sys = PusherSliderSystem();
    f = Friction();
    c = Controller();
    s = Simulator();
    %Build variables
    obj.N = (1/h)*(tf-t0);
    t = zeros(obj.N,1);
    %Initialize variables
    obj.xs_state(1,:) = xs0';   

    for i1=1:obj.N
        disp(t(i1));
        %Define loop variables
        xs = obj.xs_state(i1,:)';
        %Controller
        [xcStar, ucStar, xsStar, usStar] = c.getStateNominal(t(i1));
        xc = c.coordinateTransformSC(xs);
        [uc] = c.solveMPC(xc, t(i1));
        us = c.force2Velocity(xc,uc); 
        %Simulator
        dxs = s.lineSimulator(xs,us);
        %%
        %Euler Integration
        if i1<obj.N
            t(i1+1)  = t(i1) + h;
            obj.xs_state(i1+1, :) = obj.xs_state(i1, :) + h*dxs';       
        end
        %Save data 
        obj.xs_star_state(i1,:) = xsStar';
        obj.us_star_state(i1,:) = usStar';
        obj.xc_star_state(i1,:) = xcStar';
        obj.uc_star_state(i1,:) = ucStar';
        obj.us_state(i1,:) = us;
        obj.uc_state(i1,1:5) = uc;
    end     
    %Post-Processing Data (Animation and plots)
    obj.t = t;
    obj.xs{SimulationCounter+1} = obj.xs_state;  
    obj.us{SimulationCounter+1} = obj.us_state; 
    obj.uc{SimulationCounter+1} = obj.uc_state; 
    obj.xs{1} = c.xs_star();  
    obj.us{1} = c.us_star(); 
end