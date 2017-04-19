classdef Simulator < dynamicprops & Friction & PusherSliderSystem
    % Class used during Euler integration to determine time derivative of
    % slider and pusher coordinates in world frame

    properties (Constant)
        num_xsStates = 6;
        num_usStates = 3;
    end
    
    properties
    end
   
    methods
        %% Constructor
        function obj = Simulator()  

        end
        %% line Simulator
        function [dxs, F_gen] = lineSimulator(obj,xs,us)
        % Quasi-Static Pushing Simulator 
        %
        % USAGE
        %   dxs = s.lineSimulator(xs,us);
        %
        %   Parameters:
        %   xs        -   6x1 pose matrix [x;y;theta;xp;yp;thetap]. (pose of slider AND pose of pusher in inertial coordinates)
        %   us        -   3x1 pusher velocity (in world frame)

        %Extract variables from xs
        ribi = xs(1:2);
        theta = xs(3);
        ripi = xs(4:5);
        thetap = xs(6);
        %Extract variables from us
        vipi = us(1:2);
        dthetap = us(3);
        %Direction Cosine Matrices 
        Cbi = Helper.C3_2d(theta);
        Cpi = Helper.C3_2d(thetap);   
        %Convert state to body frame
        rbbi = Cbi*ribi;
        rbpi = Cbi*ripi;
        vbpi = Cbi*vipi;
        %Find rx, ry: In body frame   
        rpap = [0;obj.lp/2];
        rpcp = [0;-obj.lp/2];
        rbap = Cbi*Cpi'*rpap;
        rbcp = Cbi*Cpi'*rpcp;
        rbab = rbpi + rbap - rbbi;
        rbcb = rbpi + rbcp - rbbi;
        rbai = rbpi + rbap;
        rbci = rbpi + rbcp;
        rax = rbab(1);
        ray = rbab(2);
        rcx = rbcb(1);
        rcy = rbcb(2);
        rb{1} = rbab;
        rb{2} = rbcb;
        %kinematics
        npa_b = [1;0];
        tpa_b = [0;1];
        Dpa_b = [tpa_b -tpa_b];
        Jpa = [1 0 -rbab(2);...
               0 1 rbab(1)];
        vpa = vbpi + Helper.cross3d(dthetap, rbap); 
        %kinematics
        npc_b = [1;0];
        tpc_b = [0;1];
        Dpc_b = [tpc_b -tpc_b];
        Jpc = [1 0 -rbcb(2);...
               0 1 rbcb(1)];
        vpc = vbpi + Helper.cross3d(dthetap, rbcp);
        %find distances
        rbzi = rbbi-[obj.a/2;0];
        rbaz = rbai-rbzi;
        rbcz = rbci-rbzi;
        d{1} = -rbaz'*npa_b;
        d{2} = -rbcz'*npc_b;
        %build useful matrices pt a
        N{1} = npa_b'*Jpa;
        L{1} = Dpa_b'*Jpa;
        E{1} = [1;1];
        aMat{1} = -npa_b'*vpa;
        bMat{1} = -Dpa_b'*vpa;
        %build useful matrices pt a
        N{2} = npc_b'*Jpc;
        L{2} = Dpc_b'*Jpc;
        E{2} = [1;1];
        aMat{2} = -npc_b'*vpc;
        bMat{2} = -Dpc_b'*vpc;
        %collision check
        counter = 1;
        index_vec = [];
        for lv1=1:2
            if d{lv1}<0.0001 && abs(rb{lv1}(2))<obj.a/2
                index_vec(counter) = lv1;
                counter = counter+1;
            end
        end
        %number if contact points
        m=length(index_vec);
        %concatenate matrices
        N_tot= [];
        L_tot=[];
        E_tot=[];
        a_tot = [];
        b_tot = [];
        nu_tot = [];
        F_gen = [];
        for lv1=1:m
            N_tot = [N_tot;N{index_vec(lv1)}];
            L_tot = [L_tot;L{index_vec(lv1)}];
            E_tot = blkdiag(E_tot,E{index_vec(lv1)});
            a_tot = [a_tot;aMat{index_vec(lv1)}];
            b_tot = [b_tot;bMat{index_vec(lv1)}];
            nu_tot = blkdiag(nu_tot,obj.nu_p);
        end
        %solve LCP
        if m
            %LCP matrices
            M = [N_tot*obj.A_ls*N_tot' N_tot*obj.A_ls*L_tot' zeros(m,m);...
                L_tot*obj.A_ls*N_tot' L_tot*obj.A_ls*L_tot' E_tot;...
                nu_tot -E_tot' zeros(m,m)];
            q = [a_tot;b_tot;zeros(m,1)];
            F_gen = LCP(M,q);
            fn = F_gen(1:m);
            ft = F_gen(m+1:m+1+2*m-1);
            twist = obj.A_ls*(N_tot'*fn + L_tot'*ft);
        else
            twist=[0;0;0];
        end
        %Compute twist (in body frame)
        twist_body_b = twist;
        %convert to inertial frame
        S_ib = [Cbi' [0;0];0 0 1];
        twist_body_i = S_ib*twist_body_b;
        %Return combined velocites of slider and pusher
        dxs = [twist_body_i; us];
end
    end
end

