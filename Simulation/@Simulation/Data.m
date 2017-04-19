%% Animation Data
function AniOutput = Data(obj, ite, Dataset)  
%         Dataset    
    x_state = obj.xs{Dataset};
    u_state = obj.us{Dataset};
    %Declare variables
    x     = x_state(ite, 1);
    y     = x_state(ite, 2);
    theta = x_state(ite, 3);
    xp     = x_state(ite, 4);
    yp     = x_state(ite, 5);
    thetap = x_state(ite, 6);
    u1 = u_state(ite, 1);
    u2 = u_state(ite, 2);
    u3 = u_state(ite, 3);
    %Rotation matrix
    Rb = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    Rp = [cos(thetap) -sin(thetap); sin(thetap) cos(thetap)];
    %kinematics
    Cbi = Helper.C3_2d(theta);
    Cpi = Helper.C3_2d(thetap);
    %position vectors (inertial frame)
    ribi = [x;y];
    ripi = [xp;yp];
    ripb = ripi-ribi;
    %position vectors (other frames)
    rbbi = Cbi*ribi;
    rpap = [0;obj.d];
    rpcp = [0;-obj.d];
    riap = Cpi'*rpap;
    ricp = Cpi'*rpcp;
    rbpb = Cbi*ripb;
    rbpi = Cbi*ripi;
    rbap = Cbi*riap;
    rbcp = Cbi*ricp;
    %find distances
    rbab = rbpi + rbap - rbbi;
    rbcb = rbpi + rbcp - rbbi;
    rbai = rbpi + rbap;
    rbci = rbpi + rbcp;
    rbzi = rbbi-obj.a/2;
    rbaz = rbai-rbzi;
    rbcz = rbci-rbzi;
    npa_b = [1;0];
    npc_b = [1;0];
    rb{1} = rbab;
    rb{2} = rbcb;
    
    %% Calculate distances
    d{1} = -rbaz'*npa_b;
    d{2} = -rbcz'*npc_b;
    %% contact boolean variables
    contact = {};
    for lv1=1:2
        if d{lv1}<0.001 && abs(rb{lv1}(2))<obj.a/2
            contact{lv1} = 1;
        else
            contact{lv1} = 0;
        end
    end
%     [d{1} d{2};contact{1} contact{2}]
    %% Slider square
    % Vertices of Polygon (slider)
    P1b = [x; y] + Rb*[-obj.a/2; -obj.b/2];
    P2b = [x; y] + Rb*[obj.a/2; -obj.b/2];
    P3b = [x; y] + Rb*[obj.a/2; obj.b/2];
    P4b = [x; y] + Rb*[-obj.a/2; obj.b/2];
    %Build vector of vertices (slider)
    x1b = [P1b(1) P2b(1) P3b(1) P4b(1)];
    y1b = [P1b(2) P2b(2) P3b(2) P4b(2)];
    %% Pusher circles geometry
    Radius = 0.0035;
    numPoints=100;
    theta_vec=linspace(0,2*pi,numPoints); %100 evenly spaced points between 0 and 2pi
    rho=ones(1,numPoints)*Radius; %Radius should be 1 for all 100 points
    [X,Y] = pol2cart(theta_vec,rho); 
    %Point a (pusher)
    posa = ripi + Rp*(rpap) + 1*Rb*([-Radius;0]);
    X_circle_a = X+posa(1);
    Y_circle_a = Y+posa(2);
    %Point c (pusher)
    posc = ripi + Rp*(rpcp) + 1*Rb*([-Radius;0]);
    X_circle_c = X+posc(1);
    Y_circle_c = Y+posc(2);
    %Compute velocity vectors
    vbpi = [u1;u2];
    dthetap = u3;
    vipi = Cbi'*vbpi;
    %Set ouputs
    AniOutput.x1b = x1b; 
    AniOutput.y1b = y1b; 
    AniOutput.X_circle_a = X_circle_a;
    AniOutput.Y_circle_a = Y_circle_a;
    AniOutput.X_circle_c = X_circle_c;
    AniOutput.Y_circle_c = Y_circle_c;
    AniOutput.posa = posa;
    AniOutput.posc = posc;
    AniOutput.contact = contact;
end