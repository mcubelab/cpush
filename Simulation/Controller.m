classdef Controller < dynamicprops & PusherSliderSystem & Friction
    properties (Constant)
        h = 0.03;
        steps=50;
        num_xcStates = 4;
        num_ucStates = 5;
        uc_eq= [0.1634;0.1634;0;0;0]; %For Linearization purposes
        xc_eq = [0;0;0;0]; %For Linearization purposes
        Q = 10*diag([1,3,1,0]);
        Qf= 2000*diag([1,3,1,0]);
        R = 1*diag([1,1,1,1,0]);
    end
    
    properties
        A_linear;
        B_linear;
        A_bar;
        B_bar;
        t_star;
        xc_star;
        uc_star
        xs_star;
        us_star;
    end
   
    methods
        %% Constructor
        function obj = Controller()  
            obj.symbolicLinearize;
            obj.buildNominalTrajectory;
        end
        %Coordinate transform
        function [xc] = coordinateTransformSC(obj, xs)
            % Transform coordinates from simulator state coordinates to controller
            %  state coordinates 
            %
            %   USAGE
            %   [xc] = coordinateTransform(xs)
            %
            %   Parameters:
            %   xs        -   6x1 simulator pose matrix [x;y;theta;xp;yp;thetap]. (pose of slider AND pose of pusher in inertial coordinates)
            %   xc        -   4x1 controller state [x;y;theta;ry]. (pose of slider AND relative pose of center of pusher)
            %Extract variables from xs
            ribi = xs(1:2);
            theta = xs(3);
            ripi = xs(4:5);
            thetap = xs(6);
            %Direction Cosine Matrices 
            Cbi = Helper.C3_2d(theta);
            Cpi = Helper.C3_2d(thetap);   
            %Convert state to body frame
            rbbi = Cbi*ribi;
            rbpi = Cbi*ripi;
            %Build xc
            rbpb = rbpi - rbbi;
            ry = rbpb(2);
            %Output
            xc = [ribi;theta;ry];
        end
        %Coordinate transform
        function [xs] = coordinateTransformCS(obj, xc)
            % Transform coordinates from controller state coordinates to
            % simulator states coordinates
            %
            %   USAGE
            %   [xs] = coordinateTransform(xc)
            %
            %   Parameters:
            %   xs        -   6x1 simulator pose matrix [x;y;theta;xp;yp;thetap]. (pose of slider AND pose of pusher in inertial coordinates)
            %   xc        -   4x1 controller state [x;y;theta;ry]. (pose of slider AND relative pose of center of pusher)
            %Extract variables
            ribi = xc(1:2);
            theta = xc(3);
            ry = xc(4);
            %Direction cosine matrices
            Cbi = Helper.C3_2d(theta);
            %Convert state to body frame
            rbbi = Cbi*ribi;
            rbpb = [-obj.a/2;ry];
            ripb = Cbi'*rbpb;
            ripi = ribi+ripb;
            %Forces
            xs = [ribi;theta;ripi;theta];
        end
        
        function [xc,uc] = fullTransformSC(obj,xs,us); 
            % Transform coordinates and inputs states from simulator to
            % controller description
            %  state coordinates 
            %
            %   USAGE
            %   [xc,uc] = fullTransformSC(xs,us)
            %
            %   Parameters:
            %   xs        -   6x1 simulator pose matrix [x;y;theta;xp;yp;thetap]. (pose of slider AND pose of pusher in inertial coordinates)
            %   xc        -   4x1 controller state [x;y;theta;ry]. (pose of slider AND relative pose of center of pusher)
            %   us        -   3x1 pusher velocity (in world frame)
            %   uc        -   5x1 input matrix [fn1,fn2,ft1,ft2,dry]
            s = Simulator();
            [dxs, Fgen] = s.lineSimulator(xs,us);
            xc = obj.coordinateTransformSC(xs);
            uc = [Fgen(1:2); Fgen(3)-Fgen(4); Fgen(5)-Fgen(6); Fgen(7)*sign(Fgen(3)-Fgen(4))];
        end
        
        function [xs,us] = fullTransformCS(obj, xc,uc); 
            % Transform coordinates and inputs states from controller to
            % simulator description
            %
            %   USAGE
            %   [xs,us] = fullTransformCS(xc,uc)
            %
            %   Parameters:
            %   xs        -   6x1 simulator pose matrix [x;y;theta;xp;yp;thetap]. (pose of slider AND pose of pusher in inertial coordinates)
            %   xc        -   4x1 controller state [x;y;theta;ry]. (pose of slider AND relative pose of center of pusher)
            %   us        -   3x1 pusher velocity (in world frame)
            %   uc        -   5x1 input matrix [fn1,fn2,ft1,ft2,dry]
            us = obj.force2Velocity(xc,uc);
            xs = obj.coordinateTransformCS(xc);
        end
        %
        function [us] = force2Velocity(obj, xc, uc)
            % Transform coordinates and inputs from controller state coordinates to
            % simulator states coordinates
            %
            %   USAGE
            %   [us] = force2Velocity(obj, xc, uc)
            %
            %   Parameters:
            %   us        -   3x1 pusher velocity (in world frame)
            %   uc        -   5x1 input matrix [fn1,fn2,ft1,ft2,dry]
            %   xc        -   4x1 controller state [x;y;theta;ry]. (pose of slider AND relative pose of center of pusher)
            %Extract variables
            ribi = xc(1:2);
            theta = xc(3);
            ry = xc(4);
            %Direction cosine matrices
            Cbi = Helper.C3_2d(theta);
            %Convert state to body frame
            rbbi = Cbi*ribi;
            rbpb = [-obj.a/2;ry];
            ripb = Cbi'*rbpb;
            %Forces
            fn = uc(1:2);
            ft = uc(3:4);
            dry = uc(5);
            drbpb = [0;dry];
            %Find rx, ry: In body frame   
            rbap = [0;obj.d];
            rbcp = [0;-obj.d];    
            rbab = rbpb + rbap;
            rbcb = rbpb + rbcp;
            %kinematics
            npa_b = [1;0];
            tpa_b = [0;1];
            Tpa_b = [tpa_b];
            Jpa = [1 0 -rbab(2);...
                   0 1 rbab(1)];
            %kinematics
            npc_b = [1;0];
            tpc_b = [0;1];
            Tpc_b = [tpc_b];
            Jpc = [1 0 -rbcb(2);...
                   0 1 rbcb(1)];
            %build useful matrices pt a
            N{1} = npa_b'*Jpa;
            T{1} = Tpa_b'*Jpa;
            %build useful matrices pt a
            N{2} = npc_b'*Jpc;
            T{2} = Tpc_b'*Jpc;
            %concatenate matrices
            N_tot= [];
            T_tot=[];
            for lv1=1:2
                N_tot = [N_tot;N{lv1}];
                T_tot = [T_tot;T{lv1}];
            end
            %Compute twist (in body frame)
            twist_body_b = obj.A_ls*(N_tot'*fn+T_tot'*ft);
            %convert to inertial frame
            S_ib = [Cbi' [0;0];0 0 1];
            twist_body_i = S_ib*twist_body_b;
            %Compute twist of pusher
            twist_pusher_i = zeros(3,1);
            twist_pusher_i(1:2) = twist_body_i(1:2) + Cbi'*drbpb + Helper.cross3d(twist_body_i(3), ripb);
            twist_pusher_i(3) = twist_body_i(3);
            %output
            us = twist_pusher_i;
        end
        %Build nominal trajectory
        function obj = buildNominalTrajectory(obj)
            %  Build 4 matrices that encode the nominal trajectory as a
            %  function of time
            %   Parameters:
            %   xs_star        -   Nx6 state matrix nominal trajectory in simulator coordinates
            %   us_star        -   Nx3 input matrix nominal trajectory in simulator coordinates
            %   xc_star        -   Nx4 state matrix nominal trajectory in controller coordinates
            %   uc_star        -   Nx5 input matrix nominal trajectory in controller coordinates
            v_eq = 0.05;
            tf = 20;
            t0 = 0;
            h_star = 0.01;
            N_star = (1/h_star)*(tf-t0);
            obj.t_star = zeros(N_star,1);
            for lv1=1:N_star
                %Define nominal values (simulator coordinates)
                xsStarTemp = [v_eq*obj.t_star(lv1) 0 0 v_eq*obj.t_star(lv1)-obj.a/2 0 0]';
                usStarTemp = [v_eq 0 0]';
                %Define nominal values (controller coordinates)
                xcStarTemp = obj.coordinateTransformSC(xsStarTemp);
                ucStarTemp = obj.uc_eq;
                %Build control matrices A and B (symbolic linearization of motion
                %equations)
                obj.xs_star(lv1,:) = xsStarTemp';
                obj.us_star(lv1,:) = usStarTemp';
                obj.xc_star(lv1,:) = xcStarTemp';
                obj.uc_star(lv1,:) = ucStarTemp';
                if lv1<N_star
                    obj.t_star(lv1+1)  = obj.t_star(lv1) + h_star;
                end
            end
        end
        function obj = symbolicLinearize(obj)
            % Builds A and B linear matrices
            %
            %   Parameters:
            %   A       -   Continuous linear state matrix
            %   B       -   Continuous linear input matrix
            %   A_bar   -   Discrete linear state matrix
            %   B_bar   -   Discrete linear input matrix
            %   A,B:    -   dx_bar = A*x_bar + B*u_bar
            %   A_bar,B_bar:    -   x_bar(i+1) = A_bar*x_bar(i) + B_bar*u_bar(i)
            %% Symbolic variables
            syms x y theta a xp yp ry
            syms fx fy m fn1 fn2 ft1 ft2 ry_dot
            %% Build states       
            xc = [x;y;theta;ry];
            uc = [fn1;fn2;ft1; ft2; ry_dot];
            fn = [fn1;fn2];
            ft = [ft1; ft2];
            %% DCM Matrices
            Cbi = Helper.C3_2d(theta);
            %% Kinematics
            sign_vec = [1 -1]*1;
            rx = -obj.a/2;
            for lv1=1:2 %lv1 represents contact point 1 and 2
                rb{lv1} = [rx*1;ry*1]+sign_vec(lv1)*[0;obj.d*1]; %position of contact point lv1
                Jb{lv1} = [1 0 -rb{lv1}(2);... 
                             0 1 rb{lv1}(1)];
                for lv2=1:2 %lv2 represents left of right border (FC/MC)
                    n{lv1} = [1;0];
                    t{lv1} = [0;1];
                    N{lv1} = transpose(n{lv1})*Jb{lv1};
                    T{lv1} = transpose(t{lv1})*Jb{lv1};
                end
            end
            %Motion equations (nonlinear)
            N_tot = [N{1};N{2}];
            T_tot = [T{1};T{2}];
            Vb = obj.A_ls*(transpose(N_tot)*fn + transpose(T_tot)*ft );
            C_tilde = [Cbi 0;0 0 1];
            f_non1 = C_tilde*Vb;
            f_non2 = ry_dot;
            f_non = [f_non1;f_non2];
            %Linearization
            A = jacobian(f_non,xc);
            B = jacobian(f_non,uc);
            % Substitute equilibrium states
            A = subs(A,{x,y,theta,ry},{obj.xc_eq(1),obj.xc_eq(2),obj.xc_eq(3), obj.xc_eq(4)});
            B = subs(B,{x,y,theta,ry},{obj.xc_eq(1),obj.xc_eq(2),obj.xc_eq(3), obj.xc_eq(4)});
            A = subs(A,{fn1,fn2,ft1,ft2,ry_dot},{obj.uc_eq(1),obj.uc_eq(2),obj.uc_eq(3),obj.uc_eq(4),obj.uc_eq(5)});
            B = subs(B,{fn1,fn2,ft1,ft2,ry_dot},{obj.uc_eq(1),obj.uc_eq(2),obj.uc_eq(3),obj.uc_eq(4),obj.uc_eq(5)});
            %Convert to double type
            A=double(A);
            B=double(B);
            %Set properties
            obj.A_linear = double(A);
            obj.B_linear = double(B);
            obj.A_bar = double(eye(4)+obj.h*A);
            obj.B_bar = double(obj.h*B);
        end
        %Get nominal trajectory values at time T
        function [xcStar, ucStar, xsStar, usStar] = getStateNominal(obj, t)
            % Get nominal trajectory values at time T
            %
            %   Parameters:
            %   t       -   Time at which the nominal state is evaluated
            vecDif = t - obj.t_star;
            [valDif, indexDif] = min(abs(vecDif));
            xcStar = obj.xc_star(indexDif,:)';
            ucStar = obj.uc_star(indexDif,:)'; 
            xsStar = obj.xs_star(indexDif,:)';
            usStar = obj.us_star(indexDif,:)'; 
        end
        %solve MPC optimization problem
        function [uc] = solveMPC(obj, xc, t)
            % Returns controller input (force control) given initial
            % condition and time index
            %
            %   USAGE
            %   [uc] = solveMPC(obj, xc, t)
            %
            %   Parameters:
            %   t         -   1x1 simulation time index
            %   xc        -   4x1 controller state [x;y;theta;ry]. (pose of slider AND relative pose of center of pusher)
            
            %Define variables
            x = xc(1);
            y = xc(2);
            theta = xc(3);
            ry = xc(4);
            %Nominal coordinates
            [xcStar, ucStar, usStar, usStar] = obj.getStateNominal(t);
            %Build error state vector
            delta_xc = [xc - xcStar];
            %Loop through family of modes
            fVal = [];
            for lv2=1:3
            %Build optimization program
            Opt{lv2} = obj.buildProgram();
                %Loop through steps of MPC
                for lv1=1:obj.steps
                    %Add cost
                    Opt{lv2} = obj.buildCost(Opt{lv2}, lv1);
                    %Add dynamic constraints
                    Opt{lv2} = obj.buildDynConstraints(Opt{lv2}, lv1);
                    %Add mode independent constraints
                    Opt{lv2} = obj.buildModeIndepConstraints(Opt{lv2}, lv1);
                    if lv1==1
                        if lv2==1 
                            %Add mode dependant constraints
                            Opt{lv2} = obj.buildModeDepConstraints(Opt{lv2}, lv1, 1);%1: Sticking, 2: Sliding left(up), 3: Slide right(down)
                        elseif lv2==2 
                            %Add mode dependant constraints
                            Opt{lv2} = obj.buildModeDepConstraints(Opt{lv2}, lv1, 2);%1: Sticking, 2: Sliding left(up), 3: Slide right(down)
                        elseif lv2==3 
                            %Add mode dependant constraints
                            Opt{lv2} = obj.buildModeDepConstraints(Opt{lv2}, lv1, 3);%1: Sticking, 2: Sliding left(up), 3: Slide right(down)
                        end
                    else
                        %Add mode dependant constraints
                        Opt{lv2} = obj.buildModeDepConstraints(Opt{lv2}, lv1, 1);%1: Sticking, 2: Sliding left(up), 3: Slide right(down)
                    end  
                end
                %Update initial conditions
                Opt{lv2}.beq(1:4) = zeros(4,1);
                Opt{lv2}.beq(1:4) = [obj.A_bar*delta_xc];
                % Solve Opt Program   
                options = optimoptions('quadprog','Display','none');
                [Opt{lv2}, solvertime{lv2}, fval{lv2}] = Opt{lv2}.solve;
                out_delta_u{lv2} = Opt{lv2}.vars.u.value';
                out_delta_x{lv2} = Opt{lv2}.vars.x.value';
                fVal = [fVal; fval{lv2}];
            end
            %Find mode schedule with lowest cost
            [minFOM indexFOM] = min(fVal);
            %Return first element of control sequence
            delta_u = out_delta_u{indexFOM}(1,1:5)';
            %Add feedforward and feedback controls together
            uc = delta_u + ucStar;
        end

       %% Build optimization program and constraints
        function Opt = buildProgram(obj)
            Opt = MixedIntegerConvexProgram(false);
            Opt = Opt.addVariable('u', 'C', [obj.num_ucStates, obj.steps], -1000*ones(obj.num_ucStates,obj.steps), 1000*ones(obj.num_ucStates,obj.steps));
            Opt = Opt.addVariable('x', 'C', [obj.num_xcStates, obj.steps], -1000*ones(obj.num_xcStates,obj.steps), 1000*ones(obj.num_xcStates,obj.steps));
        end
        %Build cost matrix
        function Opt = buildCost(obj, Opt, lv1)
            %Initialize matrices
            H = zeros(Opt.nv, Opt.nv);
            % State Cost
            if lv1<obj.steps
                H(Opt.vars.x.i(1:length(obj.Q),lv1), Opt.vars.x.i(1:length(obj.Q),lv1)) = obj.Q;
            else
                H(Opt.vars.x.i(1:length(obj.Qf),lv1), Opt.vars.x.i(1:length(obj.Qf),lv1)) = obj.Qf;
            end
            %Inputs Cost
            H(Opt.vars.u.i(1:length(obj.R),lv1), Opt.vars.u.i(1:length(obj.R),lv1)) = obj.R;
            %Add cost to program
            Opt = Opt.addCost(H, [], []);
        end
        %Build dynamic constraints
        function Opt = buildDynConstraints(obj, Opt, lv1)
            numConstraints = obj.num_xcStates;
            Aeq = zeros(numConstraints, Opt.nv);
            %Special case of initial conditions
            if lv1 ~=1
            Aeq(:,Opt.vars.x.i(1:obj.num_xcStates,lv1-1))= -obj.A_bar;
            end
            Aeq(:,Opt.vars.x.i(1:obj.num_xcStates,lv1))  = eye(obj.num_xcStates);
            Aeq(:,Opt.vars.u.i(1:obj.num_ucStates,lv1))=  -obj.B_bar;
            beq = zeros(obj.num_xcStates,1);
            Opt = Opt.addLinearConstraints([], [], Aeq, beq);
        end
        %Build mode independant constraints
        function Opt = buildModeIndepConstraints(obj, Opt, lv1)
            %positive normal force(s)
            numConstraints = 2;
            uIndex = [1,2];
            Bleft = [];
            cLeft  = [];
            Bright  = eye(2);
            cRight = [];
            Opt = obj.addPusherConstraints(Bleft, Bright, cLeft, cRight, numConstraints, uIndex, lv1, '<=', 'star', Opt);
            clear Bleft cLeft Bright cRight
            %friction cone clamping of frictional force(s) Contact Point 1
            numConstraints = 1;
            uIndex = [1,3];
            Bleft = [0 1];
            cLeft  = [];
            Bright  = [obj.nu_p 0];
            cRight = [];
            Opt = obj.addPusherConstraints(Bleft, Bright, cLeft, cRight, numConstraints, uIndex, lv1, '|<=|', 'star', Opt);
            clear Bleft cLeft Bright cRight
            %friction cone clamping of frictional force(s) Contact Point 2
            numConstraints = 1;
            uIndex = [2,4];
            Bleft = [0 1];
            cLeft  = [];
            Bright  = [obj.nu_p 0];
            cRight = [];
            Opt = obj.addPusherConstraints(Bleft, Bright, cLeft, cRight, numConstraints, uIndex, lv1, '|<=|', 'star', Opt);
            clear Bleft cLeft Bright cRight
        end
        %Build mode independant constraints
        function Opt = buildModeDepConstraints(obj, Opt, lv1, mode)
            if mode==1
                % Sticking Constraint
                numConstraints = 1;
                uIndex = [5];
                Bleft  = [1];
                cLeft  = [0];
                Bright = [0];
                cRight = [0];
                Opt = obj.addPusherConstraints(Bleft, Bright, cLeft, cRight, numConstraints, uIndex, lv1, '==', 'star', Opt);
                clear Bleft cLeft Bright cRight
            elseif mode==2
                %Sliding left constraint
                numConstraints = 1;
                uIndex = [5];
                Bleft  = [1];
                cLeft  = [0];
                Bright = [0];
                cRight = [0];
                Opt = obj.addPusherConstraints(Bleft, Bright, cLeft, cRight, numConstraints, uIndex, lv1, '>', 'star', Opt);
                clear Bleft cLeft Bright cRight
                %friction cone edge constraints
                numConstraints = 1;
                uIndex = [1,3];
                Bleft  = [0 1];
                cLeft  = [0];
                Bright = [obj.nu_p 0];
                cRight = [0];
                Opt = obj.addPusherConstraints(Bleft, Bright, cLeft, cRight, numConstraints, uIndex, lv1, '==', 'star', Opt);
                clear Bleft cLeft Bright cRight
                %friction cone edge constraints
                numConstraints = 1;
                uIndex = [2,4];
                Bleft  = [0 1];
                cLeft  = [0];
                Bright = [obj.nu_p 0];
                cRight = [0];
                Opt = obj.addPusherConstraints(Bleft, Bright, cLeft, cRight, numConstraints, uIndex, lv1, '==', 'star', Opt);
                clear Bleft cLeft Bright cRight
            elseif mode==3
%                 Sliding left constraint
                numConstraints = 1;
                uIndex = [5];
                Bleft  = [1];
                cLeft  = [0];
                Bright = [0];
                cRight = [0];
                Opt = obj.addPusherConstraints(Bleft, Bright, cLeft, cRight, numConstraints, uIndex, lv1, '<=', 'star', Opt);
                clear Bleft cLeft Bright cRight
                %friction cone edge constraints
                numConstraints = 1;
                uIndex = [1,3];
                Bleft  = [0 1];
                cLeft  = [0];
                Bright = [-obj.nu_p 0];
                cRight = [0];
                Opt = obj.addPusherConstraints(Bleft, Bright, cLeft, cRight, numConstraints, uIndex, lv1, '==', 'star', Opt);
                clear Bleft cLeft Bright cRight
%                 friction cone edge constraints
                numConstraints = 1;
                uIndex = [2,4];
                Bleft  = [0 1];
                cLeft  = [0];
                Bright = [-obj.nu_p 0];
                cRight = [0];
                Opt = obj.addPusherConstraints(Bleft, Bright, cLeft, cRight, numConstraints, uIndex, lv1, '==', 'star', Opt);
                clear Bleft cLeft Bright cRight
            end
        end
        
        function Opt = addPusherConstraints(obj, Bleft, Bright, cLeft, cRight, numConstraints, I, lv1, equalFlag, nominalFlag, Opt)
        %       if strcmp(flag,'leq')
            if isempty(Bleft)
                Bleft = zeros(numConstraints);
            end
            if isempty(Bright)
                Bright = zeros(numConstraints);
            end
            if isempty(cLeft)
                cLeft = zeros(numConstraints,1);
            end
            if isempty(cRight)
                cRight = zeros(numConstraints,1);
            end
            %% If lower than equal condition
            if ~strcmp(equalFlag,'|<=|') || ~strcmp(equalFlag,'|>=|')
                %Compute A
                A = Bleft-Bright;
                b = cRight-cLeft;
                Amatrix = zeros(numConstraints, Opt.nv);
                Amatrix(:,Opt.vars.u.i(I,lv1)) = A;
                %Compute b
                if strcmp(nominalFlag,'star')
                    u_star = [];
                    for lv3=1:length(I)
                        u_star = [u_star;obj.uc_eq(I(lv3))];
                    end
                    bmatrix = - A*u_star;
                else
                    bmatrix = b;
                end
                %Add constraint
                epsilon=0.0001;
                if strcmp(equalFlag,'<=')
                    Opt =  Opt.addLinearConstraints(Amatrix, bmatrix, [], []);
                elseif strcmp(equalFlag,'<')
                    Opt =  Opt.addLinearConstraints(Amatrix, bmatrix-epsilon, [], []);
                elseif strcmp(equalFlag,'>=')
                    Opt =  Opt.addLinearConstraints(-Amatrix, -bmatrix, [], []);
                elseif strcmp(equalFlag,'>')
                    Opt =  Opt.addLinearConstraints(-Amatrix, -bmatrix-epsilon, [], []);
                elseif strcmp(equalFlag,'==')
                    Opt =  Opt.addLinearConstraints([], [], Amatrix, bmatrix);
                end
            end
            %% If lower than absolute value condition
            if strcmp(equalFlag,'|<=|')
                aSignVec = [-1,1];
                eqSignVec = [1,-1];

                for lv4=1:2
                    %Compute A
                    A = eqSignVec(lv4)*(Bleft+aSignVec(lv4)*Bright);
                    b = (cRight+aSignVec(lv4)*cLeft);
                    Ain = zeros(numConstraints, Opt.nv);
                    Ain(:,Opt.vars.u.i(I,lv1)) = A;
                    %Compute b
                    if strcmp(nominalFlag,'star')
                        u_star = [];
                        for lv3=1:length(I)
                            u_star = [u_star;obj.uc_eq(I(lv3))];
                        end
                        bin = -A*u_star;

                    else
                        bin = b;
                        disp(bin);
                    end
                    Opt =  Opt.addLinearConstraints(Ain, bin, [], []);
                end
            end
       end
     
    end
end