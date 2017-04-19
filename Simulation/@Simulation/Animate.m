%% Main Animation 
function obj = Animate(obj, flag)
%Initialize figure
obj.Ani = figure('Color', 'w', 'OuterPosition', [0, 0, 960, 1080], 'PaperPosition', [0, 0, 11, (6/8)*11]);
%figure properties
font_size  = 25;
line_size  = 15;
line_width = 2;
set(gcf,'Renderer','OpenGL');
set(gca,'FontSize',20)
axis equal
xlabel('x(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
ylabel('y(m)','fontsize',font_size,'Interpreter','latex', 'FontSize', font_size);
xlim([-.1 .3]);
ylim([-0.15 0.2]);
%Animation parameters
tf = obj.t(end);
N = length(obj.t);
accFactor = 5;
x_state = obj.xs{1};
%create movie file
videoname = strcat(obj.FilePath,'/',(obj.SimName),'.avi');
v = VideoWriter(videoname);
fps = int64(N/(accFactor*tf));
fps = double(fps);
v.FrameRate = fps;
open(v);
%% initialize plots
%nominal trajectory (red)
lv1=1;
Data{lv1} = obj.Data(1,lv1);
Slider{lv1} = patch(Data{lv1}.x1b, Data{lv1}.y1b,'r', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', 'r','FaceColor','NONE','LineWidth',0.1);
hold on 
Pusher_a{lv1} = patch(Data{lv1}.X_circle_a,Data{lv1}.Y_circle_a,'r', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1);
hold on
Pusher_c{lv1} = patch(Data{lv1}.X_circle_c,Data{lv1}.Y_circle_c,'r', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1);
hold on
%actual trajectories (blue)
for lv1=2:obj.NumSim+1
    Data{lv1} = obj.Data(1,lv1); 
    hold on 
    Slider{lv1} = patch(Data{lv1}.x1b, Data{lv1}.y1b,'red', 'EdgeAlpha', 1,'FaceAlpha', 1,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',3.0);
    hold on 
    Pusher_a{lv1} = patch(Data{lv1}.X_circle_a,Data{lv1}.Y_circle_a,'r', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1);
    hold on
    Pusher_c{lv1} = patch(Data{lv1}.X_circle_c,Data{lv1}.Y_circle_c,'r', 'EdgeAlpha', 1,'FaceAlpha', 1, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1);
    hold on
end
%% update figures
%nominal trajectory (red)
for i1=1:accFactor:length(obj.t)
    lv1=1;
    %update data
    Data{lv1} = obj.Data(i1,lv1);
    %update figures
    Slider{lv1}.XData = Data{lv1}.x1b;
    Slider{lv1}.YData = Data{lv1}.y1b;
    Pusher_a{lv1}.XData = Data{lv1}.X_circle_a;
    Pusher_a{lv1}.YData = Data{lv1}.Y_circle_a;
    Pusher_c{lv1}.XData = Data{lv1}.X_circle_c;
    Pusher_c{lv1}.YData = Data{lv1}.Y_circle_c;
%     %actual trajectories (blue)
    for lv1=2:obj.NumSim+1
        Data{lv1} = obj.Data(i1,lv1);
        Slider{lv1}.XData = Data{lv1}.x1b;
        Slider{lv1}.YData = Data{lv1}.y1b;
        Pusher_a{lv1}.XData = Data{lv1}.X_circle_a;
        Pusher_a{lv1}.YData = Data{lv1}.Y_circle_a;
        Pusher_c{lv1}.XData = Data{lv1}.X_circle_c;
        Pusher_c{lv1}.YData = Data{lv1}.Y_circle_c;
        Slider_thin{lv1} = patch(Data{lv1}.x1b, Data{lv1}.y1b,'red', 'FaceAlpha', .2,'EdgeAlpha', .2,'EdgeColor', [0,0,1]*0.3,'FaceColor','NONE','LineWidth',0.1);
        hold on 
        Pusher_thin_a{lv1} = patch(Data{lv1}.X_circle_a,Data{lv1}.Y_circle_a,'red', 'EdgeAlpha', .2,'FaceAlpha', .2, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1);
        hold on 
        Pusher_thin_c{lv1} = patch(Data{lv1}.X_circle_c,Data{lv1}.Y_circle_c,'red', 'EdgeAlpha', .2,'FaceAlpha', .2, 'EdgeColor', [0,0,1]*0.3,'FaceColor',[1,0,0]*0.5,'LineWidth',0.1);
    end
    %update and save frame
    frame = getframe(obj.Ani);
    writeVideo(v,frame);
end
close(v);



