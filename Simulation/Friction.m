classdef Friction < dynamicprops & PusherSliderSystem
    % Class that defines the limit surface (A_ls), used in controller
    % design AND simulator
    properties (Constant)
        %Pusher constants
        nu = 0.35;
        nu_p = 0.3;
    end
    
    properties
       c;
       m_max;
       f_max; 
       A_ls;
    end
    
    
    methods
        %% Constructor
        function obj = Friction()  
            %Compute f_max and m_max
            obj.f_max = (obj.nu*obj.m*Helper.g);
            obj.m_max = obj.m_max_funct(obj.nu, obj.m);
            obj.c = obj.m_max/obj.f_max; 
            %Build limit surface representation
            syms fx fy m 
            F  = [fx;fy;m];
            %% 1) Limit surface
            H = fx^2/obj.f_max^2 + fy^2/obj.f_max^2 + m^2/obj.m_max^2;
            obj.A_ls = double(hessian(H,F));
        end
        %Implement polar coordinates version
        function n_f = m_max_funct(obj, nu, m)     
            n_f_integrand = @(p1,p2) (nu*m*Helper.g/obj.A) * sqrt([p1;p2;0]'*[p1;p2;0]);
            n_f = Helper.DoubleGaussQuad(n_f_integrand,-obj.a/2,obj.a/2,-obj.b/2,obj.b/2);
        end
    end
end